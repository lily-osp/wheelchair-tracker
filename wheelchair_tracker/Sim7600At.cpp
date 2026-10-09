/**
 * Sim7600At.cpp - see Sim7600At.h for the firmware quirks this implements.
 *
 * Buffer is a fixed 1 KB ring-less buffer: the module is a line protocol and a
 * runaway stream must not be allowed to allocate on a 4 MB part. Overflow is
 * counted rather than silently truncating the tail, because a corrupted line
 * is worse than a missing one.
 */

#include "Sim7600At.h"

// Unrequested result codes. Deliberately narrow: anything else beginning with
// '+' is payload. Query responses sharing a prefix with a URC (+CPIN, +CEREG,
// +CREG, +CGREG, +CMSS, +CDEL) are absent on purpose - filtering those as
// unsolicited discards the reply to the query that asked for them.
static const char *URC_PREFIXES[] = {
    "+CMTI", "+CMT", "+CUSD", "+CGEV", "+CIEV", "+CRING", "+RING", "+POLL",
    "+PBREADY", "+CTS", "+CERR",
};
static const uint8_t URC_PREFIX_COUNT = sizeof(URC_PREFIXES) / sizeof(*URC_PREFIXES);

// "ERROR" first: plain ERROR is the base form, +CME/+CMS are qualified
// variants and must be matched by prefix from it.
static const char *TERMINAL_ERR[] = {
    "ERROR", "+CME ERROR", "+CMS ERROR", "NO CARRIER", "ABORTED", "BUSY",
};
static const uint8_t TERMINAL_ERR_COUNT =
    sizeof(TERMINAL_ERR) / sizeof(*TERMINAL_ERR);

// ---- small string helpers (no String::c_str churn on the hot path) --------

static bool startsWith(const String &s, const char *prefix) {
  size_t n = strlen(prefix);
  return s.length() >= n && strncmp(s.c_str(), prefix, n) == 0;
}

static bool equalsNoCase(const String &s, const char *other) {
  return strcasecmp(s.c_str(), other) == 0;
}

static String trim(const String &s) {
  const char *p = s.c_str();
  size_t a = 0, b = s.length();
  while (a < b && (isspace((unsigned char)p[a]) || p[a] == '\r' || p[a] == '\0')) a++;
  while (b > a && (isspace((unsigned char)p[b - 1]) || p[b - 1] == '\r')) b--;
  return s.substring((int)a, (int)b);
}

const String &AtResult::first(const char *prefix) const {
  static const String empty;
  for (uint8_t i = 0; i < count; i++) {
    if (startsWith(lines[i], prefix)) return lines[i];
  }
  return empty;
}

bool AtResult::has(const char *prefix) const {
  for (uint8_t i = 0; i < count; i++) {
    if (startsWith(lines[i], prefix)) return true;
  }
  return false;
}

// Split one specific line into comma fields, quotes stripped. Kept separate
// from AtResult::value because some replies repeat a prefix across lines
// (+CGPS: 0 then +CGPS: 0,1) and the caller needs to choose between them
// rather than always take the first.
static String fieldOf(const String &line, int index) {
  int colon = line.indexOf(':');
  if (colon < 0) return String();
  String body = line.substring(colon + 1);

  String fields[12];
  uint8_t n = 0;
  int start = 0;
  while (n < 12) {
    int comma = body.indexOf(',', start);
    if (comma < 0) {
      fields[n++] = trim(body.substring(start));
      break;
    }
    fields[n++] = trim(body.substring(start, comma));
    start = comma + 1;
  }
  if (n == 0) return String();
  if (index < 0) index += n;          // negative means from the end
  if (index < 0 || index >= n) return String();
  String v = fields[index];
  v.replace("\"", "");
  return v;
}

String AtResult::value(const char *prefix, int index) const {
  const String &line = first(prefix);
  if (line.length() == 0) return String();
  return fieldOf(line, index);
}

// ---- lifecycle ------------------------------------------------------------

void Sim7600At::begin(uint32_t baud, uint16_t timeoutMs, uint8_t retries) {
  _baud = baud;
  _timeoutMs = timeoutMs;
  _retries = retries;
  drain();
}

void Sim7600At::drain() {
  _rxLen = 0;
  _dropped = 0;
  _urcCount = 0;
  while (_uart.available()) _uart.read();
}

bool Sim7600At::sync(uint8_t attempts, uint16_t delayMs) {
  // Cost is attempts x (retries + 1) x 1 s, not attempts x 1 s: command()
  // retries a timeout internally, so sync(30, 500) with the default 2 retries
  // blocks for ~90 s before giving up. Measured on the bench. Pass fewer
  // attempts, or lower retries, when a quick give-up is wanted.
  for (uint8_t i = 0; i < attempts; i++) {
    AtResult r = command("AT", 1000);
    if (r.ok) return true;
    drain();
    delay(delayMs);
  }
  return false;
}

bool Sim7600At::bringUp(const char *apn, uint8_t ctx) {
  drain();
  if (!command("AT").ok) return false;
  if (!command("ATE0").ok) return false;        // echo off: bare replies follow
  if (!command("AT+CMEE=2").ok) return false;   // numeric +CME/+CMS errors
  if (!command("AT+CMGF=1").ok) return false;   // SMS text mode
  if (!command("AT+CSCS=\"GSM\"").ok) return false;
  if (!command("AT+CNMI=2,1").ok) return false; // new SMS as +CMTI on AT port
  if (!command("AT+CSCLK=0").ok) return false;  // never sleep under us
  if (apn && *apn) return setApn(apn, ctx);
  return true;
}

// ---- transport ------------------------------------------------------------

bool Sim7600At::_send(const char *cmd) {
  _uart.print(cmd);
  _uart.print("\r\n");
  return true;
}

int Sim7600At::_readLine(uint32_t deadline, String &out) {
  while (true) {
    for (uint16_t i = 0; i < _rxLen; i++) {
      if (_rx[i] == '\n') {
        out = String(_rx, i);
        memmove(_rx, _rx + i + 1, _rxLen - i - 1);
        _rxLen -= (uint16_t)(i + 1);
        out = trim(out);
        return 1;
      }
    }
    if ((int32_t)(millis() - deadline) >= 0) return 0;   // deadline, no line

    if (_uart.available()) {
      char c = (char)_uart.read();
      if (_rxLen < sizeof(_rx) - 1) {
        _rx[_rxLen++] = c;
      } else {
        _dropped++;      // overflow: drop the byte, keep counting
      }
    } else {
      delayMicroseconds(200);
    }
  }
}

int Sim7600At::_classify(const String &line) {
  if (equalsNoCase(line, "OK")) return 1;                 // ok
  for (uint8_t i = 0; i < TERMINAL_ERR_COUNT; i++) {
    if (line.startsWith(TERMINAL_ERR[i])) return 2;        // err
  }
  if (line.startsWith("+")) {
    for (uint8_t i = 0; i < URC_PREFIX_COUNT; i++) {
      if (line.startsWith(URC_PREFIXES[i])) return 3;      // urc
    }
  }
  return 0;                                               // data
}

bool Sim7600At::_readToEnd(uint16_t timeoutMs, const char *expect, AtResult &res) {
  uint32_t deadline = millis() + timeoutMs;
  res.clear();
  _urcCount = 0;

  while (true) {
    String line;
    if (!_readLine(deadline, line)) {
      res.timedOut = true;
      return false;
    }
    if (line.length() == 0) continue;

    if (expect && line == expect) {
      res.ok = true;
      return true;
    }

    int kind = _classify(line);
    if (kind == 1) {
      res.ok = true;
      return true;
    }
    if (kind == 2) {
      if (line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR")) {
        int colon = line.indexOf(':');
        res.cmeCode = colon >= 0 ? (uint8_t)atoi(line.substring(colon + 1).c_str()) : 0;
      }
      res.ok = false;
      return false;
    }
    if (kind == 3) {
      if (_urcCount < 8) _urc[_urcCount] = line;
      _urcCount++;
      continue;
    }
    if (res.count < 24) res.lines[res.count++] = line;
  }
}

AtResult Sim7600At::command(const char *cmd, uint16_t timeoutMs, const char *expect) {
  uint16_t t = timeoutMs ? timeoutMs : _timeoutMs;
  AtResult res;

  for (uint8_t attempt = 0; attempt <= _retries; attempt++) {
    _send(cmd);
    if (_readToEnd(t, expect, res)) return res;
    if (!res.timedOut) return res;   // a real ERROR will not fix itself

    if (attempt == _retries) break;
    // A timed-out command leaves the module mid-reply. A bare CR resyncs the
    // stream before retrying.
    delay(20);
    _rxLen = 0;
    _uart.print("\r\n");
    delay(50);
    _rxLen = 0;
    delay(100 * (attempt + 1));
  }
  return res;
}

bool Sim7600At::_promptAndBody(const char *cmd, const char *body,
                               const char *terminator, AtResult &res) {
  // Phase 1: send, wait for the '>' prompt.
  _send(cmd);
  uint32_t deadline = millis() + 20000;
  while (true) {
    String line;
    if (!_readLine(deadline, line)) return false;
    if (line == ">") break;
    if (_classify(line) == 2) return false;
  }
  // Phase 2: body plus terminator (Ctrl-Z ends an SMS).
  _uart.print(body);
  _uart.print(terminator);
  // Phase 3: normal terminating result code.
  return _readToEnd(20000, nullptr, res);
}

// ---- validation -----------------------------------------------------------
// AT is a line protocol: an unvalidated value containing CR, LF or a quote
// would let a caller append arbitrary commands. These whitelist rather than
// escape.

bool Sim7600At::_validNumber(const char *number, String &clean) {
  clean = "";
  for (const char *p = number; *p; p++) {
    if (isdigit((unsigned char)*p)) {
      clean += *p;
    } else if (*p == '+' && clean.length() == 0) {
      clean += *p;
    } else if (*p == ' ' || *p == '-' || *p == '(' || *p == ')') {
      continue;
    } else {
      return false;
    }
  }
  String digits = clean;
  if (digits.startsWith("+")) digits = digits.substring(1);
  return digits.length() >= 7 && digits.length() <= 15;
}

bool Sim7600At::_validApn(const char *apn) {
  size_t n = strlen(apn);
  if (n == 0 || n > 64) return false;
  for (const char *p = apn; *p; p++) {
    if (!isalnum((unsigned char)*p) && *p != '.' && *p != '_' && *p != '-') return false;
  }
  return true;
}

bool Sim7600At::_validHost(const char *host) {
  size_t n = strlen(host);
  if (n == 0 || n > 255) return false;
  for (const char *p = host; *p; p++) {
    if (!isalnum((unsigned char)*p) && *p != '.' && *p != '-' && *p != ':') return false;
  }
  return true;
}

bool Sim7600At::_validBody(const char *text) {
  if (text == nullptr || *text == '\0') return false;
  if (strlen(text) > 160) return false;   // one GSM-7 segment
  for (const char *p = text; *p; p++) {
    if ((unsigned char)*p < 0x20 || (unsigned char)*p == 0x7F) return false;
  }
  return true;
}

// ---- identity -------------------------------------------------------------

bool Sim7600At::info(String &manufacturer, String &model, String &firmware) {
  // With echo off these reply bare, so take the first non-empty payload line
  // rather than looking for a "+CGMI:"-style prefix.
  AtResult mfg = command("AT+CGMI");
  AtResult mdl = command("AT+CGMM");
  AtResult fw = command("AT+CGMR");
  if (!mfg.ok || !mdl.ok || !fw.ok) return false;

  manufacturer = mfg.ok ? trim(mfg.lines[0]) : String();
  model = mdl.ok ? trim(mdl.lines[0]) : String();
  firmware = fw.value("+CGMR");
  return model.length() > 0;
}

String Sim7600At::imei() {
  AtResult r = command("AT+CGSN");
  if (!r.ok || r.count == 0) return "";
  return trim(r.lines[0]);
}

String Sim7600At::iccid() {
  AtResult r = command("AT+CIMI");
  if (!r.ok || r.count == 0) return "";
  return trim(r.lines[0]);
}

String Sim7600At::simStatus() {
  return command("AT+CPIN?").value("+CPIN", 0);
}

bool Sim7600At::unlockSim(const char *pin) {
  if (!pin) return false;
  size_t n = strlen(pin);
  if (n < 4 || n > 8) return false;
  for (const char *p = pin; *p; p++) {
    if (!isdigit((unsigned char)*p)) return false;
  }
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "AT+CPIN=\"%s\"", pin);
  return command(cmd).ok;
}

// ---- network --------------------------------------------------------------

int Sim7600At::signal(uint8_t &ber) {
  AtResult r = command("AT+CSQ");
  if (!r.ok) return -999;
  String v = r.value("+CSQ", 0);
  ber = (uint8_t)atoi(r.value("+CSQ", 1).c_str());
  int rssi = atoi(v.c_str());
  return rssi == 99 ? -999 : -113 + 2 * rssi;
}

bool Sim7600At::operatorInfo(String &name, String &tech, uint8_t &status) {
  // AT+COPS=0,2 asks for the long format; this firmware accepts and ignores
  // it, so there is nothing to switch. The name is text either way; only the
  // technology is numeric.
  static const struct { const char *code; const char *name; } TECH[] = {
    {"7", "LTE"}, {"8", "EC-GSM"}, {"0", "GSM"}, {"1", "GSM Compact"},
    {"3", "GSM w/EGPRS"}, {"4", "UMTS"}, {"9", "HSDPA"}, {"10", "HSUPA"},
    {"11", "HSPA"}, {"13", "E-UTRA"}, {"12", "UMTS"}, {"14", "HSDPA"},
  };
  AtResult r = command("AT+COPS?");
  if (!r.ok) return false;

  String f[4];
  uint8_t n = 0;
  int colon = r.first("+COPS").indexOf(':');
  if (colon < 0) return false;
  String body = r.first("+COPS").substring(colon + 1);
  int start = 0;
  while (n < 4) {
    int comma = body.indexOf(',', start);
    if (comma < 0) {
      f[n++] = trim(body.substring(start));
      break;
    }
    f[n++] = trim(body.substring(start, comma));
    start = comma + 1;
  }
  if (n < 4) return false;

  status = (uint8_t)atoi(f[0].c_str());
  name = f[2];
  name.replace("\"", "");
  tech = f[3];
  for (auto &t : TECH) {
    if (tech == t.code) {
      tech = t.name;
      break;
    }
  }
  return true;
}

bool Sim7600At::isRegistered() {
  AtResult r = command("AT+CEREG?");
  if (!r.ok) return false;
  int stat = atoi(r.value("+CEREG", 1).c_str());
  return stat == 1 || stat == 5;      // home or roaming
}

String Sim7600At::systemInfo() {
  return command("AT+CPSI?").value("+CPSI", 0);
}

bool Sim7600At::isAttached() {
  return command("AT+CGATT?").value("+CGATT", 0) == "1";
}

bool Sim7600At::setApn(const char *apn, uint8_t ctx) {
  if (!_validApn(apn)) return false;
  char cmd[96];
  snprintf(cmd, sizeof(cmd), "AT+CGDCONT=%u,\"IP\",\"%s\"", ctx, apn);
  return command(cmd).ok;
}

String Sim7600At::apn(uint8_t ctx) {
  AtResult r = command("AT+CGDCONT?");
  if (!r.ok) return "";
  for (uint8_t i = 0; i < r.count; i++) {
    if (!r.lines[i].startsWith("+CGDCONT:")) continue;
    int colon = r.lines[i].indexOf(':');
    String body = r.lines[i].substring(colon + 1);
    String f[3];
    uint8_t n = 0;
    int start = 0;
    while (n < 3) {
      int comma = body.indexOf(',', start);
      if (comma < 0) {
        f[n++] = trim(body.substring(start));
        break;
      }
      f[n++] = trim(body.substring(start, comma));
      start = comma + 1;
    }
    if (n >= 3 && f[0] == String(ctx)) {
      String a = f[2];
      a.replace("\"", "");
      return a;
    }
  }
  return "";
}

String Sim7600At::ipAddress(uint8_t ctx) {
  // AT+CIFSR returns ERROR on this firmware even with the context up.
  AtResult r = command("AT+CGPADDR");
  if (!r.ok) return "";
  for (uint8_t i = 0; i < r.count; i++) {
    if (!r.lines[i].startsWith("+CGPADDR:")) continue;
    int colon = r.lines[i].indexOf(':');
    String body = r.lines[i].substring(colon + 1);
    int comma = body.indexOf(',');
    if (comma < 0) continue;
    String cid = trim(body.substring(0, comma));
    String ip = trim(body.substring(comma + 1));
    if (cid == String(ctx)) {
      return (ip == "0.0.0.0" || ip.length() == 0) ? "" : ip;
    }
  }
  return "";
}

String Sim7600At::resolve(const char *host) {
  if (!_validHost(host)) return "";
  char cmd[300];
  snprintf(cmd, sizeof(cmd), "AT+CDNSGIP=\"%s\"", host);
  return command(cmd, 15000).value("+CDNSGIP", 2);
}

// ---- SMS ------------------------------------------------------------------

uint8_t Sim7600At::smsCount(uint16_t &used, uint16_t &capacity) {
  AtResult r = command("AT+CPMS?");
  if (!r.ok) return 0;
  used = (uint16_t)atoi(r.value("+CPMS", 1).c_str());
  capacity = (uint16_t)atoi(r.value("+CPMS", 2).c_str());
  return r.ok;
}

bool Sim7600At::smsSend(const char *number, const char *text, uint16_t &ref) {
  String clean;
  if (!_validNumber(number, clean)) return false;
  if (!_validBody(text)) return false;

  char cmd[48];
  snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", clean.c_str());

  AtResult res;
  // SMS bodies end with CR before Ctrl-Z; without the CR some firmware
  // includes it in the message.
  char body[192];
  snprintf(body, sizeof(body), "%s\r", text);
  if (!_promptAndBody(cmd, body, "\x1a", res)) return false;

  ref = (uint16_t)atoi(res.value("+CMGS").c_str());
  return true;
}

uint8_t Sim7600At::smsList(const char *folder, SmsMessage *out, uint8_t max) {
  static const char *VALID[] = {
      "REC READ", "REC UNREAD", "STO UNSENT", "STO SENT", "ALL"};
  bool okFolder = false;
  for (const char *v : VALID) {
    if (strcmp(v, folder) == 0) { okFolder = true; break; }
  }
  if (!okFolder) return 0;

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CMGL=\"%s\"", folder);
  AtResult r = command(cmd, 8000);
  if (!r.ok) return 0;

  uint8_t n = 0;
  for (uint8_t i = 0; i < r.count && n < max; i++) {
    if (!r.lines[i].startsWith("+CMGL:")) continue;
    int colon = r.lines[i].indexOf(':');
    String body = r.lines[i].substring(colon + 1);
    String f[6];
    uint8_t k = 0;
    int start = 0;
    while (k < 6) {
      int comma = body.indexOf(',', start);
      if (comma < 0) { f[k++] = trim(body.substring(start)); break; }
      f[k++] = trim(body.substring(start, comma));
      start = comma + 1;
    }
    // Fields arrive quoted; strip them before copying into the fixed buffers.
    for (uint8_t j = 0; j < k; j++) f[j].replace("\"", "");
    SmsMessage &m = out[n++];
    m.index = atoi(f[0].c_str());
    snprintf(m.status, sizeof(m.status), "%s", k > 1 ? f[1].c_str() : "");
    snprintf(m.from, sizeof(m.from), "%s", k > 2 ? f[2].c_str() : "");
    snprintf(m.timestamp, sizeof(m.timestamp), "%s", k > 4 ? f[4].c_str() : "");
  }
  return n;
}

bool Sim7600At::smsRead(uint8_t index, SmsMessage &msg) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "AT+CMGR=%u", index);
  AtResult r = command(cmd, 8000);
  if (!r.ok) return false;

  const String &hdr = r.first("+CMGR");
  if (hdr.length() == 0) return false;

  int colon = hdr.indexOf(':');
  String body = hdr.substring(colon + 1);
  String f[4];
  uint8_t k = 0;
  int start = 0;
  while (k < 4) {
    int comma = body.indexOf(',', start);
    if (comma < 0) { f[k++] = trim(body.substring(start)); break; }
    f[k++] = trim(body.substring(start, comma));
    start = comma + 1;
  }
  for (uint8_t j = 0; j < k; j++) f[j].replace("\"", "");
  msg.index = index;
  snprintf(msg.status, sizeof(msg.status), "%s", k > 0 ? f[0].c_str() : "");
  snprintf(msg.from, sizeof(msg.from), "%s", k > 1 ? f[1].c_str() : "");
  snprintf(msg.timestamp, sizeof(msg.timestamp), "%s", k > 3 ? f[3].c_str() : "");

  // Body is everything after the header line. A message containing a bare "OK"
  // would be read as a terminator - a property of the chat protocol.
  msg.text = "";
  bool inBody = false;
  for (uint8_t i = 0; i < r.count; i++) {
    if (!inBody) {
      if (r.lines[i].startsWith("+CMGR:")) inBody = true;
      continue;
    }
    if (msg.text.length()) msg.text += "\n";
    msg.text += r.lines[i];
  }
  return true;
}

bool Sim7600At::smsDelete(uint8_t index) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "AT+CMGD=%u", index);
  return command(cmd, 8000).ok;
}

// ---- GNSS -----------------------------------------------------------------

bool Sim7600At::gnssOn(uint8_t nmeaPort) {
  if (nmeaPort > 2) return false;   // port 3 wedges the AT command channel
  char cmd[40];
  snprintf(cmd, sizeof(cmd), "AT+CGPSNMEAPORTCFG=%u", nmeaPort);
  if (!command(cmd).ok) return false;
  return command("AT+CGPS=1").ok;
}

bool Sim7600At::gnssOff(uint16_t settleMs) {
  bool ok = command("AT+CGPS=0").ok;
  // The receiver takes seconds to wind down; reading status sooner returns the
  // stale on-state plus a bare "+CGPS: 0" line.
  if (settleMs) delay(settleMs);
  return ok;
}

bool Sim7600At::gnssStatus(bool &on, uint8_t &fixMode) {
  AtResult r = command("AT+CGPS?");
  if (!r.ok) return false;

  // While the receiver winds down the module emits a bare '+CGPS: 0' before
  // the full '+CGPS: <on>,<mode>'. Prefer the most complete line so a settling
  // read does not report mode 0.
  on = false;
  fixMode = 0;
  bool found = false;
  for (uint8_t i = 0; i < r.count; i++) {
    if (!r.lines[i].startsWith("+CGPS:")) continue;
    found = true;
    String a = fieldOf(r.lines[i], 0);
    String b = fieldOf(r.lines[i], 1);
    if (b.length() > 0) {
      on = a == "1";
      fixMode = (uint8_t)atoi(b.c_str());
      return true;                     // full line wins
    }
    on = a == "1";
  }
  return found;
}

// ---- power ----------------------------------------------------------------

bool Sim7600At::reboot(uint32_t timeoutMs) {
  // The module usually drops the port before answering, so a failure here is
  // expected and not itself an error.
  command("AT+CRESET", 5000);
  _rxLen = 0;
  _urcCount = 0;
  return sync((uint8_t)(timeoutMs / 500));
}

bool Sim7600At::radioOff() {
  return command("AT+CFUN=0").ok;
}

bool Sim7600At::radioOn() {
  return command("AT+CFUN=1", 10000).ok && sync();
}

// ---- capabilities ---------------------------------------------------------

Sim7600At::Capabilities Sim7600At::capabilities() {
  Capabilities c;
  c.sms = command("AT+CMGF=1").ok;
  c.dns = command("AT+CDNSGIP=\"example.com\"", 15000).ok;
  c.gnss = command("AT+CGPS?").ok;
  // All three of these return ERROR on this firmware build.
  c.tcp = command("AT+CIPSOCK=6").ok;
  c.http = command("AT+CHTTPCREATE").ok;
  c.mqtt = command("AT+CMQNEW?").ok;
  if (c.tcp) command("AT+CIPCLOSE=6");
  return c;
}

bool Sim7600At::requireFeature(const char *feature) {
  Capabilities c = capabilities();
  if (strcmp(feature, "sms") == 0) return c.sms;
  if (strcmp(feature, "dns") == 0) return c.dns;
  if (strcmp(feature, "gnss") == 0) return c.gnss;
  if (strcmp(feature, "tcp") == 0) return c.tcp;
  if (strcmp(feature, "http") == 0) return c.http;
  if (strcmp(feature, "mqtt") == 0) return c.mqtt;
  return false;
}