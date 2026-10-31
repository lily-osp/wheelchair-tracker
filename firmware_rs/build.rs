use std::path::Path;

fn main() {
    // Generate Adafruit IO constants from adafruit.toml for every target
    // (firmware and host tests), so secrets live in a config file instead
    // of source.
    generate_aio_config();

    // The linker args only apply to the firmware target; host tests link
    // against the system toolchain and must not see them.
    if std::env::var("CARGO_CFG_TARGET_ARCH").as_deref() != Ok("riscv32") {
        return;
    }
    linker_be_nice();
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                what if what.starts_with("_defmt_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`"
                    );
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                what if what.starts_with("esp_rtos_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `esp-radio` has no scheduler enabled. Make sure you have initialized `esp-rtos` or provided an external scheduler."
                    );
                    eprintln!();
                }
                "embedded_test_linker_file_not_added_to_rustflags" => {
                    eprintln!();
                    eprintln!(
                        "💡 `embedded-test` not found - make sure `embedded-test.x` is added as a linker script for tests"
                    );
                    eprintln!();
                }
                "free"
                | "malloc"
                | "calloc"
                | "get_free_internal_heap_size"
                | "malloc_internal"
                | "realloc_internal"
                | "calloc_internal"
                | "free_internal" => {
                    eprintln!();
                    eprintln!(
                        "💡 Did you forget the `esp-alloc` dependency or didn't enable the `compat` feature on it?"
                    );
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}

fn generate_aio_config() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let config_path = Path::new(&manifest_dir).join("adafruit.toml");
    let content = std::fs::read_to_string(&config_path).unwrap_or_else(|_| {
        panic!(
            "missing {} - copy adafruit.toml.example and fill in your Adafruit IO key",
            config_path.display()
        )
    });

    let mut host: Option<String> = None;
    let mut port: Option<u16> = None;
    let mut key: Option<String> = None;
    let mut user: Option<String> = None;
    let mut location_feed: Option<String> = None;
    let mut time_feed: Option<String> = None;
    let mut section = String::new();

    for raw in content.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            section = line[1..line.len() - 1].trim().to_string();
            continue;
        }
        if section != "aio" {
            continue;
        }
        let Some((k, v)) = line.split_once('=') else {
            continue;
        };
        let k = k.trim();
        let v = v.trim();
        let val = if v.starts_with('"') && v.ends_with('"') && v.len() >= 2 {
            v[1..v.len() - 1].to_string()
        } else {
            v.to_string()
        };
        match k {
            "host" => host = Some(val),
            "port" => port = Some(val.parse().expect("aio.port must be a number")),
            "key" => key = Some(val),
            "user" => user = Some(val),
            "location_feed" => location_feed = Some(val),
            "time_feed" => time_feed = Some(val),
            _ => {}
        }
    }

    let host = host.expect("aio.host missing in adafruit.toml");
    let port = port.unwrap_or(80);
    let key = key.expect("aio.key missing in adafruit.toml");
    let user = user.expect("aio.user missing in adafruit.toml");
    let location_feed = location_feed.unwrap_or_else(|| "location".to_string());
    let time_feed = time_feed.unwrap_or_else(|| "time".to_string());

    fn esc(s: &str) -> String {
        s.replace('\\', "\\\\").replace('"', "\\\"")
    }

    let code = format!(
        "pub const CFG_AIO_HOST: &str = \"{}\";\n\
         pub const CFG_AIO_PORT: u16 = {};\n\
         pub const CFG_AIO_KEY: &str = \"{}\";\n\
         pub const CFG_AIO_USER: &str = \"{}\";\n\
         pub const CFG_AIO_FEED_LOCATION: &str = \"{}\";\n\
         pub const CFG_AIO_FEED_TIME: &str = \"{}\";\n",
        esc(&host),
        port,
        esc(&key),
        esc(&user),
        esc(&location_feed),
        esc(&time_feed)
    );

    let out_dir = std::env::var("OUT_DIR").unwrap();
    std::fs::write(Path::new(&out_dir).join("aio_config.rs"), code).unwrap();
    println!("cargo:rerun-if-changed={}", config_path.display());
}
