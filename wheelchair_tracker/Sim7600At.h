/**
 * Sim7600At.h - AT transport for the SIM7600CE-T, verified against firmware
 * LE20B05SIM7600M22 (AT+CSUB: MDM9x07_LE20_S_22_V1.14_240221).
 *
 * Ported from the MicroPython driver that was validated against a live module
 * over USB. Third-party SIM7600/TinyGSM libraries assume a working
 * AT+CIPSOCK/AT+CIPSTART; both return ERROR on this firmware, so their network
 * paths are dead on arrival. This transport implements what the module
 * actually does.
 *
 * Hardware notes baked in from that testing:
 *   - GNSS is AT+CGPS / AT+CGPSNMEAPORTCFG, NOT AT+CGNSPWR.
 *   - AT+CGPSNMEAPORTCFG=3 puts NMEA on the AT port and wedges the command
 *     channel. Use 1.
 *   - AT+CIFSR is broken; use AT+CGPADDR.
 *   - With echo off, action-form commands return BARE values (AT+CGMM ->
 *     "SIM7600CE-T"), with no "+CGMM:" prefix and leading blank lines.
 *   - +CPIN/+CEREG share a prefix with URCs and must NOT be filtered as
 *     unsolicited, or the reply to the query is discarded.
 */

#ifndef SIM7600_AT_H
#define SIM7600_AT_H

#include <Arduino.h>

// 3GPP TS 27.007 numeric mobile termination errors this code acts on.
enum Sim7600Error : uint8_t {
  SIM7600_OK = 0,
  SIM7600_ERR_TIMEOUT = 0xFF,
  SIM7600_ERR_UART = 0xFE,
};

// Result of one command.
struct AtResult {
  bool ok = false;
  bool timedOut = false;
  uint8_t cmeCode = 0;     // numeric reason from +CME ERROR, 0 if none
  String lines[24];       // payload lines, terminator and URCs removed
  uint8_t count = 0;

  void clear() {
    ok = false; timedOut = false; cmeCode = 0; count = 0;
  }

  const String &first(const char *prefix) const;
  // By value, not by reference: the field is a temporary, and returning a
  // reference to it would dangle at the end of the full expression.
  String value(const char *prefix, int index = -1) const;
  bool has(const char *prefix) const;
};

// SMS message as returned by AT+CMGL / AT+CMGR.
struct SmsMessage {
  int index = 0;
  char status[16] = {0};
  char from[24] = {0};
  char timestamp[24] = {0};
  String text;
};

class Sim7600At {
public:
  Sim7600At(Stream &uart) : _uart(uart) {}

  void begin(uint32_t baud = 115200, uint16_t timeoutMs = 6000,
             uint8_t retries = 2);

  // Low-level. Returns false on timeout or ERROR; check result.timedOut/cmeCode.
  AtResult command(const char *cmd, uint16_t timeoutMs = 0,
                   const char *expect = nullptr);

  // Wait for the module to answer AT. Call after pulsing PWRKEY.
  bool sync(uint8_t attempts = 30, uint16_t delayMs = 500);

  // Bring-up: echo off, verbose errors, SMS text mode, new-SMS notification.
  bool bringUp(const char *apn = nullptr, uint8_t ctx = 1);

  // Drain buffered input. After a baud change or a hard reset.
  void drain();

  // ---- identity -------------------------------------------------------
  bool info(String &manufacturer, String &model, String &firmware);
  String imei();
  String iccid();
  String simStatus();                       // "READY", "SIM PIN", ...
  bool unlockSim(const char *pin);

  // ---- network --------------------------------------------------------
  // RSSI in dBm, or -999 when the modem reports 99 (unknown). ber via out.
  int signal(uint8_t &ber);
  bool operatorInfo(String &name, String &tech, uint8_t &status);
  bool isRegistered();
  String systemInfo();                      // AT+CPSI
  bool isAttached();
  bool setApn(const char *apn, uint8_t ctx = 1);
  String apn(uint8_t ctx = 1);
  String ipAddress(uint8_t ctx = 1);
  String resolve(const char *host);         // AT+CDNSGIP, cellular DNS

  // ---- SMS ------------------------------------------------------------
  bool smsSend(const char *number, const char *text, uint16_t &ref);
  uint8_t smsCount(uint16_t &used, uint16_t &capacity);
  uint8_t smsList(const char *folder, SmsMessage *out, uint8_t max);
  bool smsRead(uint8_t index, SmsMessage &msg);
  bool smsDelete(uint8_t index);

  // ---- GNSS -----------------------------------------------------------
  // nmeaPort 0..2. Port 3 is rejected: it routes NMEA onto the AT port.
  bool gnssOn(uint8_t nmeaPort = 1);
  bool gnssOff(uint16_t settleMs = 4000);
  bool gnssStatus(bool &on, uint8_t &fixMode);

  // ---- power ----------------------------------------------------------
  bool reboot(uint32_t timeoutMs = 30000);
  bool radioOff();
  bool radioOn();

  // Probe what this firmware implements. sms/dns/gnss/tcp/http/mqtt.
  struct Capabilities {
    bool sms = false, dns = false, gnss = false;
    bool tcp = false, http = false, mqtt = false;
  };
  Capabilities capabilities();
  bool requireFeature(const char *feature);

  // Unrequested result codes seen during the last command.
  uint8_t lastUrcCount() const { return _urcCount; }
  const String &lastUrc(uint8_t i) const { return _urc[i]; }

private:
  Stream &_uart;
  uint32_t _baud = 115200;
  uint16_t _timeoutMs = 6000;
  uint8_t _retries = 2;

  char _rx[1024];
  uint16_t _rxLen = 0;
  uint16_t _dropped = 0;      // overflow counter, exposed via overflowCount()

  String _urc[8];
  uint8_t _urcCount = 0;

  bool _send(const char *cmd);
  int _readLine(uint32_t deadline, String &out);
  int _classify(const String &line);
  bool _readToEnd(uint16_t timeoutMs, const char *expect, AtResult &res);
  bool _promptAndBody(const char *cmd, const char *body,
                      const char *terminator, AtResult &res);
  bool _validNumber(const char *number, String &clean);
  bool _validApn(const char *apn);
  bool _validHost(const char *host);
  bool _validBody(const char *text);
};

#endif  // SIM7600_AT_H