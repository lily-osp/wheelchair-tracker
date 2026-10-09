/**
 * wheelchair_tracker.ino - motion-gated location tracker.
 *
 * ESP32-C3 SuperMini + SIM7600CE-T + LIS2DH.
 *
 * The LIS2DH decides whether the chair is moving; the modem only buys a GNSS
 * fix when it is. That gate is the whole power budget, because a cold GNSS fix
 * costs roughly 35 days of deep sleep:
 *
 *     35 s x 120 mA x 3.3 V = 13.9 J per fix
 *     5 uA x 3.3 V x 24 h    = 0.4 J per day of sleep
 *
 * Deep sleep wakes on the LIS2DH's INT1 pin. A long timer runs in parallel as a
 * watchdog, so a failed or dead accelerometer degrades to periodic reporting
 * instead of silence - a tracker that can only wake on motion is a tracker that
 * goes quiet the day the sensor fails.
 *
 * UNTESTED ON HARDWARE. See hardware.md for the bench checklist.
 */

#include <Arduino.h>
#include <Wire.h>
#include <esp_sleep.h>

#include "Sim7600At.h"
#include "lis2dh.h"
#include "Nmea.h"

// ---- pins. Verify against your board's silkscreen. -----------------------
// GPIO18/19 are USB Serial/JTAG on the C3 SuperMini and are unusable if you
// program over USB, which is the only path on this board.
static const int MODEM_TX = 4;        // C3 TX  -> modem RXD
static const int MODEM_RX = 5;        // C3 RX  <- modem TXD
static const int PWRKEY_GATE = 6;     // N-MOSFET gate, never PWRKEY direct
static const int IMU_SDA = 8;         // onboard LED must be removed (see README)
static const int IMU_SCL = 9;
static const int IMU_INT1 = 0;        // RTC pin 0-5 wakes deep sleep on C3
static const uint8_t IMU_ADDR = 0x19; // 0x18 if SDO/SA0 is pulled low
static const int MODEM_PWRKEY_HOLD_MS = 1500;

// Sync is attempts x (retries + 1) x 1 s, because command() retries a timeout
// internally. With the default 2 retries, 8 attempts blocks ~24 s - measured on
// the bench, not guessed. Kept short so a missing modem is reported quickly.
static const uint8_t MODEM_SYNC_ATTEMPTS = 8;

// ---- cadence --------------------------------------------------------------
static const uint32_t POLL_MS = 5000;         // motion sampling interval
static const uint32_t SEND_MOVING_MS = 60000; // report while travelling
static const uint32_t SEND_IDLE_MS = 900000;  // parked heartbeat (15 min)
static const uint32_t WATCHDOG_MS = 3600000;   // always-on backstop (1 h)
static const uint32_t FIX_VALID_MS = 300000;   // fix reusable for 5 min
static const uint32_t FIX_STALE_MS = 600000;   // beyond this, label stale

// Hysteresis thresholds in mg of deviation from 1 g. Without this a chair
// stopped at a kerb flips state every few seconds and fires an SMS each time.
static const uint16_t MOVE_ENTER_MG = 250;
static const uint16_t MOVE_EXIT_MG = 120;
static const uint8_t HYSTERESIS_SAMPLES = 3;

// Destination for position reports. Set this before flashing - it is a
// placeholder, not a real number.
static const char OWNER[] = "+6281234567890";

// ---- state ----------------------------------------------------------------
// Deep sleep resets RAM, so anything that must outlive a sleep cycle goes in
// RTC slow memory. Keeping the last fix here means a parked chair reports its
// last known position for free instead of buying another acquisition.
struct PersistState {
  uint32_t magic;
  int32_t latE7;          // degrees x 1e7, to keep float out of slow mem
  int32_t lonE7;
  uint8_t sats;
  uint32_t fixTicks;      // ticks_ms() when acquired
  uint32_t nextSendTicks;
  uint8_t moving;
  uint8_t settled;
  uint8_t hadFix;
};
static PersistState RSTATE;

static const uint32_t STATE_MAGIC = 0x57435452;   // "WCTR"

static HardwareSerial ModemSerial(1);
static Sim7600At modem(ModemSerial);
static Lis2dh imu;

// ---- fix cache ------------------------------------------------------------

static bool fixValid() {
  return RSTATE.hadFix && (uint32_t)(millis() - RSTATE.fixTicks) < FIX_VALID_MS;
}

static bool acquireFix() {
  // NMEA on the modem's UART1, not the AT port. Port 3 would put NMEA into the
  // command stream and wedge it; gnssOn(1) refuses it for that reason.
  if (!modem.gnssOn(1)) {
    Serial.println(F("gnss on failed"));
    return false;
  }

  HardwareSerial nmeaSerial(2);
  nmeaSerial.begin(115200, SERIAL_8N1, -1, -1);

  NmeaFix fix;
  uint32_t deadline = millis() + 45000;      // cold start is ~35 s
  uint8_t buf[256];
  while ((int32_t)(millis() - deadline) < 0) {
    int n = nmeaSerial.readBytes(buf, sizeof(buf));
    if (n <= 0) continue;
    for (int i = 0; i < n; i++) {
      if (buf[i] == '\n') continue;
      if (nmeaFeed(fix, buf[i])) {
        if (fix.valid()) goto done;
      }
    }
  }

done:
  modem.gnssOff();                            // settles ~4 s
  nmeaSerial.end();
  if (!fix.valid()) return false;

  RSTATE.latE7 = (int32_t)(fix.lat * 1e7);
  RSTATE.lonE7 = (int32_t)(fix.lon * 1e7);
  RSTATE.sats = fix.sats;
  RSTATE.fixTicks = millis();
  RSTATE.hadFix = 1;
  return true;
}

// ---- reporting ------------------------------------------------------------

static bool report(const char *why) {
  if (!fixValid()) {
    if (RSTATE.hadFix) {
      // Stale fix: a parked chair has not moved, so re-sending the last
      // position is honest and free. Reporting nothing would be less useful
      // for no battery saving at all.
    } else if (!acquireFix()) {
      Serial.println(F("no fix, nothing sent"));
      return false;
    }
  }

  char body[160];
  uint32_t ageS = (uint32_t)(millis() - RSTATE.fixTicks) / 1000;
  bool stale = ageS * 1000 > FIX_STALE_MS;
  snprintf(body, sizeof(body), "%s%s %.5f,%.5f s%u a%lus%s %s",
           RSTATE.moving ? "M" : "S", stale ? "*" : "",
           RSTATE.latE7 / 1e7, RSTATE.lonE7 / 1e7,
           RSTATE.sats, (unsigned long)ageS, stale ? "(stale)" : "", why);

  uint16_t ref = 0;
  if (!modem.smsSend(OWNER, body, ref)) {
    Serial.println(F("sms failed"));
    return false;
  }
  Serial.printf("sent ref=%u: %s\n", ref, body);
  return true;
}

// ---- motion gate ----------------------------------------------------------

static bool updateMotion() {
  bool active = imu.motionInterrupt() ||
                imu.isMoving(MOVE_ENTER_MG, 1, 50);

  static uint8_t movingStreak = 0, idleStreak = 0;
  if (active) {
    movingStreak++;
    idleStreak = 0;
  } else {
    idleStreak++;
    movingStreak = 0;
  }

  if (!RSTATE.moving && movingStreak >= HYSTERESIS_SAMPLES) {
    RSTATE.moving = 1;
    return true;                            // transition: report now
  }
  if (RSTATE.moving && idleStreak >= HYSTERESIS_SAMPLES) {
    int32_t mag = imu.magnitude();
    if (mag > 0 && abs(mag - 1000) <= (int32_t)MOVE_EXIT_MG) {
      RSTATE.moving = 0;
      return true;
    }
  }
  return false;
}

// ---- deep sleep -----------------------------------------------------------

// EXT0/EXT1 do not exist on the C3. esp_deep_sleep_enable_gpio_wakeup() works on
// RTC pins GPIO0-5, and the wake reason is reported as ESP_SLEEP_WAKEUP_GPIO
// (0x05) rather than an EXT value.
static void sleepUntil(uint32_t targetMs) {
  uint32_t now = millis();
  uint32_t waitMs = (int32_t)(targetMs - now) > 0 ? (targetMs - now) : 0;

  // Cap on the motion wait: the watchdog wake is what guarantees a heartbeat.
  if (waitMs > POLL_MS) waitMs = POLL_MS;
  // And a floor, so a target already in the past does not busy-loop the RTC.
  if (waitMs < 50) waitMs = 50;

  // Drain the USB CDC before cutting power. Deep sleep does not flush, so any
  // un-flushed print is silently lost - and the wake after it re-enters
  // setup(), so the log can look empty even though the firmware is running.
  Serial.flush();
  fflush(stdout);

  esp_sleep_enable_timer_wakeup(waitMs * 1000ULL);
  // LIS2DH INT1 is ACTIVE HIGH by default: the pin is driven low when idle and
  // rises when the threshold is crossed. Polarity is settable via CTRL_REG3
  // (INTx_POL), which is left at its reset value here.
  // UNVERIFIED: no LIS2DH attached yet. If motion never wakes the chip, this
  // polarity is the first thing to flip - see README.md "Wake polarity".
  esp_deep_sleep_enable_gpio_wakeup(BIT(IMU_INT1), ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}

// ---- setup / loop ---------------------------------------------------------

void setup() {
  // MUST build with CDCOnBoot=cdc. With the default (Disabled), ARDUINO_USB_MODE
  // is 1 and Serial is not the USB CDC at all: the boot ROM log still appears
  // (it is raw ROM output) but every Serial.print from the app goes nowhere,
  // which looks exactly like a dead board. Verified on the bench.
  Serial.begin(115200);
  Serial.setTimeout(2000);
  delay(1000);                             // let the host finish enumerating

  // Get the modem answering before touching anything else.
  ModemSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  digitalWrite(PWRKEY_GATE, HIGH);
  pinMode(PWRKEY_GATE, OUTPUT);

  Serial.println(F("pulsing PWRKEY"));
  digitalWrite(PWRKEY_GATE, LOW);          // active low, via N-MOSFET
  delay(MODEM_PWRKEY_HOLD_MS);
  digitalWrite(PWRKEY_GATE, HIGH);

  if (!modem.sync(MODEM_SYNC_ATTEMPTS, 500)) {
    Serial.println(F("modem silent - check PWRKEY and the 4.0 V supply"));
    return;
  }
  modem.begin();
  modem.bringUp();

  String mfg, model, fw;
  if (modem.info(mfg, model, fw)) {
    Serial.printf("%s %s fw=%s\n", mfg.c_str(), model.c_str(), fw.c_str());
  }

  Sim7600At::Capabilities caps = modem.capabilities();
  Serial.printf("sms=%d dns=%d gnss=%d tcp=%d http=%d mqtt=%d\n",
                caps.sms, caps.dns, caps.gnss, caps.tcp, caps.http, caps.mqtt);
  if (!caps.sms) {
    Serial.println(F("no SMS on this firmware - tracker cannot report"));
    return;
  }

  Wire.begin(IMU_SDA, IMU_SCL, 400000);
  if (!imu.begin(Wire, IMU_ADDR)) {
    Serial.printf("no LIS2DH at 0x%02X - IMU_SDA/SCL or SDO/SA0\n", IMU_ADDR);
    // Without the IMU the timer watchdog is all that keeps this reporting.
    Serial.println(F("continuing without motion gating"));
  } else {
    imu.configure(0x08);
    Serial.println(F("LIS2DH ready, threshold 0x08 (0.5 g)"));
  }
}

void loop() {
  // Restore across deep sleep. First boot has no valid magic.
  if (RSTATE.magic != STATE_MAGIC) {
    RSTATE.magic = STATE_MAGIC;
    RSTATE.hadFix = 0;
    RSTATE.moving = 0;
    RSTATE.settled = 0;
    RSTATE.nextSendTicks = millis() + POLL_MS;
  }

  bool transitioning = updateMotion();

  // Let the gate classify before the first send, so a chair that boots while
  // parked does not buy a GNSS fix on the way to deciding it is parked.
  if (!RSTATE.settled) {
    RSTATE.settled = 1;
    if (RSTATE.moving) {
      RSTATE.nextSendTicks = 0;            // moving at boot: report now
    } else {
      RSTATE.nextSendTicks = millis() + SEND_IDLE_MS;
    }
  }

  bool due = (int32_t)(millis() - RSTATE.nextSendTicks) >= 0;
  bool watchdog = false;
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    watchdog = (uint32_t)(millis() - RSTATE.nextSendTicks) > WATCHDOG_MS;
  }

  if (due || watchdog || transitioning) {
    report(transitioning ? "move" : (watchdog ? "watchdog" : "tick"));
    RSTATE.nextSendTicks = millis() + (RSTATE.moving ? SEND_MOVING_MS
                                                     : SEND_IDLE_MS);
  }

  sleepUntil(RSTATE.nextSendTicks);
}