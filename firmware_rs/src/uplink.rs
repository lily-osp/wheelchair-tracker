//! PPP dial + HTTP POST uplink over the modem's AT UART.
//!
//! The module's AT data channels are broken (`AT+CIPSTART` etc. return
//! ERROR), but the PPP data dial (`ATD*99***1#`) is a separate path and is
//! the route to internet access. The ESP32's own radio stays unused.
//!
//! Sequence per report wake:
//!   1. (AT context) `AT+CDNSGIP` resolves the server host - after dialing,
//!      the AT channel is gone, so DNS must happen first.
//!   2. Dial: `ATD*99***1#` until `CONNECT`.
//!   3. `select3` of the PPP runner, the embassy-net runner and the POST
//!      future, under one deadline. No spawned tasks: the UART is handed to
//!      the PPP runner by mutable reference and the borrow ends with the
//!      select, which terminates the PPP session cleanly.
//!   4. Escape back to AT context (`+++` with guard, then `ATH0`).

use core::fmt::Write as _;
use core::net::Ipv4Addr;

use embassy_futures::select::{Either3, select3};
use embassy_net::{
    ConfigV4, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4,
    tcp::TcpSocket,
};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Write as _;
use heapless::String;

use crate::config;
use crate::sim7600::{ModemError, PppUart, Sim7600};

pub const SOCKETS: usize = 3;
pub const PPP_RX: usize = 8;
pub const PPP_TX: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UplinkError {
    /// Host did not resolve in AT context.
    Dns,
    /// Dial failed or timed out.
    Dial(ModemError),
    /// IPCP never came up or the POST ran past the deadline.
    Timeout,
    /// TCP connect/write/read failure.
    Tcp,
    /// IPCP negotiated no address.
    NoAddress,
}

/// Resolves `host` with `AT+CDNSGIP` while still in AT context. Returns the
/// first resolved address, or None.
pub async fn resolve_host(sim: &mut Sim7600<'_>, host: &str) -> Option<Ipv4Address> {
    if !crate::at::valid_host(host) {
        return None;
    }
    let mut cmd = String::<64>::new();
    let _ = write!(cmd, "AT+CDNSGIP=\"{}\"", host);
    let out = sim
        .command(cmd.as_str(), Some(Duration::from_secs(10)))
        .await;
    if !out.res.ok {
        return None;
    }
    // "+CDNSGIP: 0,1,\"203.0.113.7\"" -> field 2.
    let ip = out.value("+CDNSGIP", 2);
    parse_ip(ip.as_str())
}

fn parse_ip(s: &str) -> Option<Ipv4Address> {
    s.trim().trim_matches('"').parse::<Ipv4Addr>().ok()
}

/// What the caller must hold for the duration of one HTTP report.
/// Statics via `static_cell`/`StaticCell`, because the stack requires
/// `'static` lifetimes.
pub struct UplinkResources {
    pub ppp_state: embassy_net_ppp::State<PPP_RX, PPP_TX>,
    pub net_resources: StackResources<SOCKETS>,
}

impl UplinkResources {
    pub const fn new() -> Self {
        Self {
            ppp_state: embassy_net_ppp::State::new(),
            net_resources: StackResources::new(),
        }
    }
}

/// Dials and POSTs one report. Borrows the modem's UART for the duration;
/// the borrow ends with the deadline or the POST, whichever is first.
pub async fn dial_and_post(
    sim: &mut Sim7600<'_>,
    host_ip: Ipv4Address,
    path: &str,
    body: &[u8],
    res: &'static mut UplinkResources,
) -> Result<(), UplinkError> {
    sim.dial(Duration::from_secs(20))
        .await
        .map_err(UplinkError::Dial)?;

    let (ppp_device, mut ppp_runner) = embassy_net_ppp::new(&mut res.ppp_state);
    let (stack, mut net_runner) = embassy_net::new(
        ppp_device,
        embassy_net::Config::default(), // empty: IPCP fills it in
        &mut res.net_resources,
        seed(),
    );

    // IPCP callback: feed the negotiated address into the stack. `Stack` is
    // Copy, so the closure takes its own handle.
    let on_ipv4_up = move |status: embassy_net_ppp::Ipv4Status| {
        let Some(addr) = status.address else {
            return;
        };
        let mut dns: heapless::Vec<Ipv4Address, 3> = heapless::Vec::new();
        for d in status.dns_servers.into_iter().flatten() {
            let _ = dns.push(Ipv4Address::from(d));
        }
        let _ = stack.set_config_v4(ConfigV4::Static(StaticConfigV4 {
            address: Ipv4Cidr::new(Ipv4Address::from(addr), 32),
            gateway: status.peer_address.map(Ipv4Address::from),
            dns_servers: dns,
        }));
    };

    let ppp_uart = PppUart::new(sim.uart_mut());

    let ppp_run = ppp_runner.run(
        ppp_uart,
        embassy_net_ppp::Config {
            username: b"",
            password: b"",
        },
        on_ipv4_up,
    );
    let post = async {
        match with_timeout(Duration::from_secs(25), wait_ip(&stack)).await {
            Ok(true) => {}
            _ => return Err(UplinkError::Timeout),
        }
        post_request(&stack, host_ip, config::SERVER_PORT, path, body).await
    };

    // select3, not join3: the net runner never completes (`-> !`), so a
    // join would hang forever. select3 returns when the first future does -
    // normally the POST, or the PPP runner on a serial error. Dropping the
    // incomplete runners terminates the PPP session and frees the UART.
    let deadline = Duration::from_millis(config::UPLINK_DEADLINE_MS as u64);
    let outcome =
        with_timeout(deadline, select3(ppp_run, net_runner.run(), post)).await;
    // The borrows have ended; the modem may still be in data mode.
    hangup(sim).await;

    match outcome {
        Ok(Either3::Third(result)) => result,
        // Deadline fired, or the PPP runner bailed early.
        _ => Err(UplinkError::Timeout),
    }
}

/// Waits for the stack to receive an IPv4 config from IPCP.
async fn wait_ip(stack: &Stack<'_>) -> bool {
    let deadline = embassy_time::Instant::now() + Duration::from_secs(20);
    loop {
        if stack.is_config_up() && stack.is_link_up() {
            return true;
        }
        if embassy_time::Instant::now() >= deadline {
            return false;
        }
        Timer::after_millis(100).await;
    }
}

/// Plain HTTP POST. No TLS: the body travels in the clear (documented
/// limitation; server-side hardening is an infrastructure task).
async fn post_request(
    stack: &Stack<'_>,
    ip: Ipv4Address,
    port: u16,
    path: &str,
    body: &[u8],
) -> Result<(), UplinkError> {
    let mut rx = [0u8; 1024];
    let mut tx = [0u8; 1024];
    let mut socket = TcpSocket::new(*stack, &mut rx, &mut tx);

    if socket.connect((ip, port)).await.is_err() {
        return Err(UplinkError::Tcp);
    }

    let mut req = String::<768>::new();
    let _ = write!(
        req,
        "POST {} HTTP/1.1\r\nHost: {}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        path, config::SERVER_HOST, body.len()
    );
    socket.write_all(req.as_bytes()).await.map_err(|_| UplinkError::Tcp)?;
    socket.write_all(body).await.map_err(|_| UplinkError::Tcp)?;
    socket.flush().await.map_err(|_| UplinkError::Tcp)?;

    // The status line is enough; a tracker does not need the response body.
    let mut sink = [0u8; 256];
    let mut head = String::<128>::new();
    loop {
        let n = socket.read(&mut sink).await.map_err(|_| UplinkError::Tcp)?;
        if n == 0 {
            break;
        }
        for &b in &sink[..n] {
            if (b as char).is_ascii() && head.push(b as char).is_err() {
                break;
            }
        }
        if head.len() >= 12 && head.as_bytes()[9..12] == *b"200" {
            return Ok(());
        }
        if head.contains("\r\n\r\n") {
            break;
        }
    }
    Err(UplinkError::Tcp)
}

/// Escapes the PPP data mode and returns the module to AT command context.
/// Best-effort: the next wake drains and re-syncs regardless.
async fn hangup(sim: &mut Sim7600<'_>) {
    // The escape sequence requires ~1 s of silence on both sides.
    Timer::after_millis(config::PPP_ESCAPE_GUARD_MS as u64).await;
    let _ = sim.uart_mut().write(b"+++");
    Timer::after_millis(config::PPP_ESCAPE_GUARD_MS as u64).await;
    let _ = sim.uart_mut().write(b"ATH0\r\n");
    let _ = sim.command("", Some(Duration::from_secs(2))).await;
    sim.drain();
}

/// Hardware TRNG seed for the smoltcp interface.
fn seed() -> u64 {
    let mut s = [0u8; 8];
    let _ = esp_hal::rng::Rng::new().read(&mut s);
    u64::from_le_bytes(s)
}
