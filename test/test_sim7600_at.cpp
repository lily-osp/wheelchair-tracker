/**
 * Host-side harness for Sim7600At.cpp. Not part of the firmware.
 *
 * Drives the transport with scripted module replies and asserts on what it
 * parses, so the C++ port can be validated before a C3 is wired up.
 *
 * Note the ordering below: begin() calls drain(), which consumes whatever the
 * stream offers. So the modem must be constructed and begun BEFORE any reply is
 * queued, or the script gets eaten by the drain.
 */

#include "Sim7600At.h"
#include <cstdio>
#include <vector>
#include <cmath>
#include <string>

static int g_pass = 0, g_fail = 0;

static void check(const char *label, bool ok, const std::string &got = "") {
  printf("  %s  %-36s %s\n", ok ? "PASS" : "FAIL", label, got.c_str());
  fflush(stdout);
  ok ? g_pass++ : g_fail++;
}

// A Stream that replays scripted replies as bytes become "available", and
// records what the driver wrote.
class ScriptedStream : public Stream {
public:
  std::string outbox;
  std::vector<std::string> pending;
  std::string current;
  size_t currentPos = 0;

  void push(const std::string &reply) { pending.push_back(reply); }

  int available() override {
    if (currentPos < current.size()) return 1;
    if (!pending.empty()) {
      current = pending.front();
      pending.erase(pending.begin());
      currentPos = 0;
      return 1;
    }
    return 0;
  }
  int read() override {
    if (currentPos >= current.size()) return -1;
    return (unsigned char)current[currentPos++];
  }
  size_t write(uint8_t c) override {
    outbox += (char)c;
    return 1;
  }
};

// Every test: construct, begin (which drains), THEN queue replies.
static Sim7600At *modem(ScriptedStream &s) {
  Sim7600At *m = new Sim7600At(s);
  m->begin();
  return m;
}

int main() {
  // ---- reply parsing ---------------------------------------------------
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\nOK\r\n");
    AtResult r = m->command("AT");
    check("bare OK", r.ok && !r.timedOut);
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CME ERROR: 10\r\n");
    AtResult r = m->command("AT+CPIN?");
    check("+CME ERROR 10", !r.ok && r.cmeCode == 10,
          "code=" + std::to_string(r.cmeCode));
  }
  {
    // ATE0 replies to AT+CGMI/AT+CGMM are BARE values with blank lines around
    // them and no "+CGMI:" prefix. A parser that only looks for prefixed lines
    // returns nothing here.
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\nSIMCOM INCORPORATED\r\n\r\nOK\r\n");
    s.push("\r\nSIMCOM_SIM7600CE-T\r\n\r\nOK\r\n");
    s.push("\r\n+CGMR: LE20B05SIM7600M22\r\n\r\nOK\r\n");
    String mfg, model, fw;
    bool ok = m->info(mfg, model, fw);
    check("bare identity parse", ok && model == "SIMCOM_SIM7600CE-T" &&
          mfg == "SIMCOM INCORPORATED" && fw == "LE20B05SIM7600M22",
          std::string(model.c_str()) + " / " + fw.c_str());
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    // No reply queued: must return empty, not garbage or a crash.
    String imei = m->imei();
    check("no reply -> empty string", imei.length() == 0);
  }
  {
    // +CEREG must NOT be classified as a URC, or the reply to the query that
    // asked for it is discarded.
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CEREG: 0,1\r\n\r\nOK\r\n");
    bool reg = m->isRegistered();
    check("+CEREG not eaten as URC", reg, reg ? "registered" : "lost");
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CPIN: READY\r\n\r\nOK\r\n");
    String pin = m->simStatus();
    check("+CPIN not eaten as URC", pin == "READY", pin.c_str());
  }
  {
    // +CMTI is genuinely unsolicited and must be captured separately from
    // the payload of the command in flight.
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CMTI: \"SM\",4\r\n+CPIN: READY\r\n\r\nOK\r\n");
    String st = m->simStatus();
    check("URC separated from payload",
          st == "READY" && m->lastUrcCount() == 1,
          "urcs=" + std::to_string(m->lastUrcCount()));
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CSQ: 19,99\r\n\r\nOK\r\n");
    uint8_t ber = 0;
    int dbm = m->signal(ber);
    check("CSQ -> dBm", dbm == -75 && ber == 99,
          std::to_string(dbm) + " dBm ber " + std::to_string(ber));
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CSQ: 99,99\r\n\r\nOK\r\n");
    uint8_t ber = 0;
    check("CSQ 99 -> unknown, not a false signal", m->signal(ber) == -999);
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+COPS: 0,0,\"Telkomsel\",7\r\n\r\nOK\r\n");
    String name, tech; uint8_t st = 0;
    bool ok = m->operatorInfo(name, tech, st);
    check("COPS tech code 7 -> LTE",
          ok && name == "Telkomsel" && tech == "LTE" && st == 0,
          std::string(name.c_str()) + "/" + tech.c_str());
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CGPADDR: 1,10.22.160.54\r\n\r\nOK\r\n");
    String ip = m->ipAddress();
    check("CGPADDR -> ip", ip == "10.22.160.54", ip.c_str());
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CGPADDR: 1,0.0.0.0\r\n\r\nOK\r\n");
    check("0.0.0.0 -> empty, not a bogus ip", m->ipAddress().length() == 0);
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CDNSGIP: 1,\"example.com\",\"172.66.147.243\"\r\n\r\nOK\r\n");
    String ip = m->resolve("example.com");
    check("CDNSGIP field 2 is the IP", ip == "172.66.147.243", ip.c_str());
  }

  // ---- SMS -------------------------------------------------------------
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push(">");                            // prompt
    s.push("\r\n+CMGS: 42\r\n\r\nOK\r\n");   // after body + Ctrl-Z
    uint16_t ref = 0;
    bool ok = m->smsSend("+6281234567890", "pump fired", ref);
    check("smsSend +CMGS ref", ok && ref == 42, "ref=" + std::to_string(ref));
    check("smsSend body + CR + Ctrl-Z",
          s.outbox.find("pump fired\r\x1a") != std::string::npos);
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    uint16_t ref = 0;
    bool ok = m->smsSend("+62\r\nAT+CGSN", "x", ref);
    check("smsSend rejects CRLF injection",
          !ok && s.outbox.find("AT+CGSN") == std::string::npos);
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    uint16_t ref = 0;
    check("smsSend rejects short number", !m->smsSend("+1", "x", ref));
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    uint16_t ref = 0;
    check("smsSend rejects >160 char body",
          !m->smsSend("+6281234567890", std::string(200, 'a').c_str(), ref));
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CMGL: 6,\"REC UNREAD\",\"+628\",\"\",\"26/10/09,15:30:14+28\"\r\n"
           "+CMGL: 7,\"REC UNREAD\",\"+628\",\"\",\"26/10/09,15:31:00+28\"\r\n\r\nOK\r\n");
    SmsMessage msgs[8];
    uint8_t n = m->smsList("REC UNREAD", msgs, 8);
    check("smsList parses entries",
          n == 2 && msgs[0].index == 6 && String(msgs[0].from) == "+628" &&
          String(msgs[0].status) == "REC UNREAD" && msgs[1].index == 7,
          "n=" + std::to_string(n));
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CMGR: \"REC UNREAD\",\"+628\",\"\",\"26/10/09,15:30:14+28\"\r\n"
           "hello world\r\nOK\r\n");
    SmsMessage msg;
    bool ok = m->smsRead(6, msg);
    check("smsRead body is after the header",
          ok && msg.text == "hello world" && msg.index == 6,
          msg.text.c_str());
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    SmsMessage msgs[8];
    check("smsList rejects unknown folder", m->smsList("BOGUS", msgs, 8) == 0);
  }

  // ---- GNSS ------------------------------------------------------------
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\nOK\r\n");
    s.push("\r\nOK\r\n");
    bool ok = m->gnssOn(1);
    check("gnssOn port 1", ok &&
          s.outbox.find("AT+CGPSNMEAPORTCFG=1") != std::string::npos &&
          s.outbox.find("AT+CGPS=1") != std::string::npos);
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    // Port 3 routes NMEA onto the AT port and wedges the command channel.
    check("gnssOn rejects port 3", !m->gnssOn(3) &&
          s.outbox.find("NMEAPORTCFG=3") == std::string::npos);
  }
  {
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CGPS: 1,1\r\n\r\nOK\r\n");
    bool on = false; uint8_t mode = 0;
    m->gnssStatus(on, mode);
    check("gnssStatus on", on && mode == 1);
  }
  {
    // A bare '+CGPS: 0' arrives while the receiver winds down.
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CGPS: 0\r\n+CGPS: 0,1\r\n\r\nOK\r\n");
    bool on = true; uint8_t mode = 0;
    m->gnssStatus(on, mode);
    // The bare '+CGPS: 0' is the stale on-state; the following full
    // line carries the authoritative mode, which must survive.
    check("gnssStatus prefers the full line", !on && mode == 1,
          "on=" + std::to_string(on) + " mode=" + std::to_string(mode));
  }

  // ---- error handling ---------------------------------------------------
  {
    ScriptedStream s;                       // never replies
    Sim7600At m(s);
    m.begin(115200, 40, 0);                 // no retries, short timeout
    AtResult r = m.command("AT");
    check("no reply -> timedOut", !r.ok && r.timedOut);
  }
  {
    ScriptedStream s;
    Sim7600At m(s);
    m.begin(115200, 200, 2);                // retries configured
    s.push("\r\nERROR\r\n");
    AtResult r = m.command("AT+CIPSOCK=6");
    check("ERROR is not retried", !r.ok && !r.timedOut);
    size_t firstErr = s.outbox.find("ERROR");
    check("no repeat of a deterministic failure",
          s.outbox.find("AT+CIPSOCK=6", firstErr) == std::string::npos);
  }
  {
    // Line split across two reads must still assemble.
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CSQ: 19,99\r");
    s.push("\n\r\nOK\r\n");
    uint8_t ber = 0;
    check("line split across reads", m->signal(ber) == -75);
  }
  {
    // A payload line containing '+' must not be mistaken for a URC.
    ScriptedStream s; Sim7600At *m = modem(s);
    s.push("\r\n+CMGR: \"REC READ\",\"+62\",\"\",\"26/10/09\"\r\n"
           "+62 halo\r\nOK\r\n");
    SmsMessage msg;
    m->smsRead(1, msg);
    check("payload '+' is not swallowed as URC", msg.text == "+62 halo",
          msg.text.c_str());
  }

  printf("\n%d/%d passed\n", g_pass, g_pass + g_fail);
  return g_fail ? 1 : 0;
}