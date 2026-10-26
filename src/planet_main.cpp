// planet_main.cpp — Solar System PLANET cart firmware (ESP32-C3 Super Mini).
// env: solar_planet_esp32c3
//
// One identical binary runs on all 8 carts. Each cart is a "dumb executor": it does not
// know its own orbit index — the Sun addresses it by MAC and sends fully-computed motor
// speeds / light / current / microstep parameters. The cart reports a compact status back.
//
// Comms: ESP-NOW. The cart has no Wi-Fi credentials; it discovers the Sun's Wi-Fi channel
// by scanning for the Sun's AP SSID, locks onto that channel, then broadcasts status until
// it hears a command (learning the Sun's MAC from the packet) and switches to unicast.
//
// Motors: 2x TMC2209 on one single-wire UART bus (addresses 0/1). The planets do NOT spin
// about their own axis (that motor was dropped) — they only roll around the rail and light up.
//
// STEP GENERATION: the ESP32-C3 LEDC peripheral (proven on the bench rig, stepper_test_main.cpp).
// LEDC emits a hardware square wave on a GPIO at a set frequency — that square wave IS the STEP
// signal. Two reasons this beats the alternatives on THIS cart:
//   * vs FastAccelStepper (RMT): the RMT step ISR is starved by the always-on Wi-Fi/ESP-NOW
//     radio on the single-core C3 and intermittently wedges on restart (motor freezes mid-ramp
//     until reboot). LEDC is pure hardware — a busy CPU never interrupts the step train.
//   * vs TMC VACTUAL: each driver's internal ~12 MHz oscillator runs free, so two VACTUAL motors
//     drift apart. The two LEDC channels derive from the ONE 80 MHz APB clock, so the L:R
//     frequency ratio is exact and drift-free — the cart rolls the circular rail without the
//     wheels slipping. Direction = TMC shaft bit over UART (DIR pins tied to GND).
//
// Accel/decel is RATIO-PRESERVING: a single ramp fraction (0->1 over WHEEL_RAMP_S) scales BOTH
// wheel targets together, so the commanded L:R speed ratio holds at every instant of the ramp
// (e.g. R=300 L=356 stays 356:300 from standstill to full speed and back). Changing microsteps
// while moving would jump the speed, so we ramp to a stop, apply at standstill, then ramp back up.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Update.h>
#include <TMCStepper.h>
#include "solar_protocol.h"

// Build identity: the Sun reads this marker out of an uploaded image (see SOLAR_FW_MARK).
#ifndef SOLAR_FW_BUILD_UTC
#error "SOLAR_FW_BUILD_UTC is set by scripts/fw_build_stamp.py (extra_scripts in platformio.ini)"
#endif
#define FW_STR2(x) #x
#define FW_STR(x)  FW_STR2(x)
static const char FW_ID[] = SOLAR_FW_MARK FW_STR(SOLAR_FW_BUILD_UTC);
static const uint32_t g_fwStamp = SOLAR_FW_BUILD_UTC;   // UTC Unix seconds of this build

// ----------------------------- Pin map (from spec) --------------------------
static const uint8_t PIN_TMC_EN     = 2;   // TMC enable, active-low (LOW = drivers on)
static const uint8_t PIN_TMC_RX     = 4;   // Serial1 RX  <- driver PDN/UART bus
static const uint8_t PIN_TMC_TX     = 3;   // Serial1 TX  -> driver PDN/UART bus (via 1k)
static const uint8_t PIN_STEP_L     = 0;   // left  wheel STEP (LEDC ch 0 / timer0)
static const uint8_t PIN_STEP_R     = 1;   // right wheel STEP (LEDC ch 2 / timer1)
static const uint8_t PIN_MOTOR_PWR  = 5;   // (freed by dropping the spin motor) drives a high-side
                                           // MOSFET / load switch feeding VM to the TMC drivers.
                                           // HIGH = motor supply on; LOW = drivers fully asleep.
static const uint8_t PIN_HOME       = 6;   // homing sensor, active-low, FALLING pulse/rev
static const uint8_t PIN_LIGHT_EN   = 7;   // N-MOS gate switching the high-power planet LED (PWM)
static const uint8_t PIN_STATUS_LED = 8;   // onboard LED, active-low
static const uint8_t PIN_BUTTON     = 9;   // onboard boot button, active-low

// Planet light: one high-power LED behind an N-MOS on PIN_LIGHT_EN, dimmed by steady LEDC PWM.
// The Sun sends a per-planet brightness (0..100 %, perceptual); the cart gamma-corrects it to a
// duty, shows it when its light is on (per the show's lighting logic) and goes dark when off.
#define LIGHT_LEDC_CH      4               // ch4 -> timer2: clear of the wheel step timers 0 and 1
#define LIGHT_PWM_HZ       20000           // above hearing, no visible flicker
#define LIGHT_PWM_BITS     10              // 20 kHz x 1024 = 20.5 MHz: fits the C3's 40 MHz LEDC
                                           // clock (11 bits would need 41 MHz -> ledcSetup fails)
#define LIGHT_GAMMA        2.2f

// ----------------------------- Motor / TMC config ---------------------------
#define TMC_R_SENSE        0.11f
#define TMC_UART_BAUD      115200
// UART addresses (MS1/MS2 straps) differ between cart boards: the first cart has L=3 R=2, the new
// cart PCB 0 and 3. These are only the defaults; tmcFindDrivers() finds the real ones.
#define TMC_ADDR_L         0b11            // MS1=VIO MS2=VIO
#define TMC_ADDR_R         0b10            // MS1=GND MS2=VIO
// Wheel direction differs between carts (the first cart's wheels sit back-to-back and need the
// right one reversed, the new cart PCB needs neither), so it is a per-planet Sun setting
// (planets.wheels.reverse_per) sent in CmdMsg.driver_cfg, not a build option.
#define TMC_READY_TIMEOUT_MS 300           // max wait for the drivers to answer after VM switches on
// LEDC step generation. Channels must sit on DIFFERENT timers to run at independent speeds
// (timer = channel/2, one frequency per timer): ch0->timer0, ch2->timer1. Both timers still
// derive from the single 80 MHz APB clock, so the L:R frequency ratio stays exact (synchronized).
#define LEDC_CH_L          0
#define LEDC_CH_R          2
#define LEDC_CLK_HZ        80000000.0      // APB clock feeding LEDC
#define WHEEL_RAMP_S       1.0f            // seconds to accelerate 0->commanded speed (and back)
#define WHEEL_UPDATE_MS    20              // how often the software frequency ramp advances
// NOTE: motor stuck / disconnected sensing was removed — the TMC2209 StallGuard and open-load
// signals are not reliable in this installation's quiet (stealthChop) low-RPM operating regime.

// ----------------------------- Radio / power --------------------------------
// The C3 runs warm because its Wi-Fi radio is always on for ESP-NOW. Two levers reduce it:
//  - Lower the max TX power: 44 = 11 dBm (vs the 84/21 dBm default). Ample for a room-scale
//    installation and noticeably cooler / lower current. Raise if range is a problem.
//  - Rescan for the Sun's channel less often while OFFLINE (active scans are the biggest
//    periodic radio burst on an otherwise idle cart).
#define WIFI_TX_POWER_QDBM 44
#define OFFLINE_RESCAN_MS  10000

// Boot defaults (overwritten by the first command from the Sun).
static uint16_t g_currentMa   = 200;       // current_amp=0.2 (RMS)
static uint16_t g_microsteps  = 16;        // 1..256 (uint16: 256 does not fit in a byte)
static float    g_stepDegree  = 1.8f;      // motor step angle -> full steps/rev = 360/step_degree
static bool     g_spread      = false;     // chopper: false = stealthChop (quiet), true = spreadCycle
static bool     g_interpolate = true;      // MicroPlyer: interpolate microsteps up to 256 internally
static bool     g_revL = false, g_revR = false;   // reverse a wheel (per cart, set on the Sun)
static bool     g_cfgOk      = false;     // last applyDriverParams() verified over UART?
static bool     g_cfgStale    = false;     // params changed while asleep -> apply on next wake

HardwareSerial   TMC_SERIAL(1);            // ESP32-C3 UART1
// One driver object per UART address; drvL/drvR point at the two that answer.
static TMC2209Stepper g_drv[4] = {
  TMC2209Stepper(&TMC_SERIAL, TMC_R_SENSE, 0), TMC2209Stepper(&TMC_SERIAL, TMC_R_SENSE, 1),
  TMC2209Stepper(&TMC_SERIAL, TMC_R_SENSE, 2), TMC2209Stepper(&TMC_SERIAL, TMC_R_SENSE, 3)};
static TMC2209Stepper *drvL = &g_drv[TMC_ADDR_L], *drvR = &g_drv[TMC_ADDR_R];
static bool g_drvFound = false;            // drvL/drvR confirmed by tmcFindDrivers()

static uint8_t  g_lightBrightness = 100;   // Sun-chosen brightness, 0..100 %; shown when on

// ----------------------------- Runtime state --------------------------------
enum PlanetState : uint8_t { ST_OFFLINE, ST_IDLE, ST_ACTIVE };
static PlanetState g_state = ST_OFFLINE;

static uint8_t  g_myIndex     = PLANET_INDEX_UNKNOWN;
static bool     g_motorsOn    = false;     // drivers enabled (VM on, EN low)
static bool     g_lightOn     = false;
static bool     g_homingStop  = false;     // true while executing CMD_HOME_AND_STOP

// Wheel velocity ramp (µsteps/s, signed = direction). A SINGLE ramp fraction scales both wheels
// from their start velocity to their final target, so the L:R speed ratio is preserved throughout
// accel/decel. serviceRamp() advances the fraction and emits the matching LEDC frequency.
static float    g_cmdWheelL   = 0;         // last commanded wheel rpm (for status/debug)
static float    g_cmdWheelR   = 0;
static float    g_velCurL = 0, g_velCurR = 0;     // current wheel velocity, µsteps/s (signed)
static float    g_velStartL = 0, g_velStartR = 0; // velocity at ramp start, µsteps/s (signed)
static float    g_velFinalL = 0, g_velFinalR = 0; // target velocity, µsteps/s (signed)
static float    g_rampFrac    = 1.0f;      // 0..1 shared ramp progress (1 = settled at target)
static float    g_actHzL = 0, g_actHzR = 0;// actual LEDC frequency achieved (magnitude, Hz)
static uint32_t g_rampMs      = 0;         // last ramp tick

// Deferred actions triggered when the wheels have ramped to a stop.
static bool     g_stopThenOff   = false;   // ramp to 0, then disable the drivers (CMD_STOP/homing)
static bool     g_resumePending = false;   // ramp to 0, change params, then ramp back up
static float    g_pendingRpmL = 0, g_pendingRpmR = 0;   // rpm to resume with after the change
static uint16_t g_pendingMicro   = 16;
static uint16_t g_pendingCurrent = 200;
static bool     g_pendingSpread  = false;
static bool     g_pendingInterp  = true;
static bool     g_pendingRevL = false, g_pendingRevR = false;

// Diagnostics reported back to the Sun and printed in the heartbeat.
static int8_t   g_sunRssi     = 0;         // Sun signal (dBm): live from its frames, else last scan
static volatile int16_t g_rssiQ16 = 0;     // live Sun RSSI, smoothed, in 1/16 dBm (0 = none yet)
static float    g_dieTempC    = 0;         // ESP32-C3 internal temperature (°C)

static volatile uint32_t g_lastCmdMs = 0;  // last time a valid command arrived (ESP-NOW callback)

// Milliseconds since t. t is written by the ESP-NOW callback, which can run between our millis()
// and our read of t, making t a hair newer than "now": count that as 0 rather than letting the
// unsigned difference wrap to ~4e9 ("forever ago"). That wrap made a cart declare itself OFFLINE
// right after a command and run a blocking 3-7 s rescan: the link drops every 1-3 minutes.
static inline uint32_t msSince(uint32_t t) {
  int32_t d = (int32_t)(millis() - t);
  return d > 0 ? (uint32_t)d : 0;
}
static uint32_t g_nextStatusMs = 0;        // next scheduled status transmit
static uint32_t g_channelScanMs = 0;       // last channel (re)scan while offline
static uint32_t g_lastDbgMs   = 0;         // periodic debug heartbeat print

// HOME sensor (ISR-driven, one pulse per revolution).
static volatile bool     g_homePulse = false;   // consumed by status TX
static volatile bool     g_homeSeen  = false;   // consumed by homing logic

// Sun peer bookkeeping.
static uint8_t  g_sunMac[6] = {0};
static bool     g_sunKnown  = false;
static const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Self-test (button in OFFLINE): spin + light for a fixed window.
static bool     g_selfTest    = false;
static uint32_t g_selfTestEndMs = 0;

// Firmware download from the Sun (see solar_protocol.h). The offer arrives in every CmdMsg; image
// chunks are queued by the recv callback and written from loop() (no flash writes in the Wi-Fi
// task). The download's state rides along in every StatusMsg.
#define FW_BURST          4         // chunks asked for per request
#define FW_REQ_TIMEOUT_MS 200       // re-ask from the current offset if a burst stalls
#define FW_NO_DATA_MS     8000      // give up when the Sun stops answering for this long
#define FW_RETRY_MS       60000     // after a failure, wait this long before trying that image again
#define FW_RXQ            8         // chunk queue depth

static uint32_t  g_offerStamp = 0, g_offerSize = 0;    // image on offer, from the latest CmdMsg
static uint8_t   g_offerMd5[16];
static bool      g_offerForce = false;                  // install it even if it isn't newer
static FwDataMsg g_fwRxq[FW_RXQ];
static volatile uint8_t g_fwRxHead = 0, g_fwRxTail = 0;
static bool      g_dlActive = false;
static uint32_t  g_dlStamp = 0, g_dlSize = 0, g_dlOffset = 0;
static uint32_t  g_dlReqMs = 0, g_dlDataMs = 0;
static uint8_t   g_dlGot = 0;                          // chunks handled since the last request
static uint32_t  g_dlFailStamp = 0, g_dlFailMs = 0;
static uint8_t   g_otaState = OTA_IDLE, g_otaProgress = 0;
static int16_t   g_otaErr = 0;

// ----------------------------- HOME ISR -------------------------------------
void IRAM_ATTR homeIsr() {
  g_homePulse = true;
  g_homeSeen  = true;
}

// ----------------------------- Helpers --------------------------------------
// Wi-Fi signal as a friendly 0-100%: -100 dBm -> 0%, -50 dBm or better -> 100%.
static inline uint8_t rssiPct(int8_t rssi) {
  if (rssi == 0) return 0;                       // unknown
  int p = 2 * (rssi + 100);
  return (uint8_t)constrain(p, 0, 100);
}

// ESP32-C3 built-in temperature sensor (die temp; reads ~10-20 °C above ambient from self-heat).
static float readDieTempC() {
  return temperatureRead();
}

// Signed wheel speed in µsteps/s for a given rpm. Speed is fully defined by rpm and the step
// angle; microsteps scales both the µstep rate and the driver divisor, so shaft RPM is
// independent of the microstep setting.
static float rpmToUstepsHz(float rpm) {
  float fullStepsPerRev = 360.0f / (g_stepDegree > 0.01f ? g_stepDegree : 1.8f);
  return (rpm / 60.0f) * fullStepsPerRev * (float)g_microsteps;
}

// Write microsteps and VERIFY over UART. The TMCStepper library encodes full-step (1) as register
// value 0 and has no setter case for 1, so map 1->0 on write; read the register back (0 means 1)
// and retry up to 3x if a UART write was dropped. Returns the read-back value (1..256).
static uint16_t setMicrostepsVerified(TMC2209Stepper &d, uint16_t want, uint8_t &tries) {
  uint16_t reg  = (want <= 1) ? 0 : want;              // 1 microstep -> MRES value 0
  uint16_t goal = (want <= 1) ? 1 : want;
  for (tries = 1; tries <= 3; tries++) {
    d.microsteps(reg);
    uint16_t rb = d.microsteps(); if (rb == 0) rb = 1; // read-back: 0 means full-step (1)
    if (rb == goal) return rb;
  }
  uint16_t rb = d.microsteps(); if (rb == 0) rb = 1;
  return rb;
}

// Emit a hardware square wave at f Hz on a STEP channel (f = step rate). Pick the LEDC resolution
// so the clock divider stays <=1023 (well-conditioned -> accurate frequency, unlike ledcWriteTone's
// fixed 10 bits which mis-divides fast tones). ledcSetup returns the frequency actually achieved.
static void stepEmit(int ch, float f, float &actOut) {
  if (f < 1.0f) { ledcWrite(ch, 0); actOut = 0; return; }   // duty 0 -> STEP held low
  int res = 1;
  while (res < 14 && (LEDC_CLK_HZ / (f * (double)(1u << res))) > 1023.0) res++;
  uint32_t act = ledcSetup(ch, (uint32_t)(f + 0.5f), res);
  ledcWrite(ch, (1u << res) / 2);                           // 50% duty at this resolution
  actOut = act ? (float)act : f;
}

// Apply ALL driver params to one wheel and VERIFY they landed over UART, retrying the whole apply
// up to `maxTries` times. Verification combines:
//   * read-back of the RW registers (microsteps, MicroPlyer/intpol, chopper/en_spreadCycle) — these
//     are read straight from the chip, so a mismatch means a dropped/garbled write; and
//   * IFCNT, the driver's own received-frame counter, which must advance during the apply. IHOLD_IRUN
//     (current), TPOWERDOWN and freewheel are write-only (no read-back), so IFCNT advancing is the
//     proof those frames were received too.
// Low-heat standstill: hold current 0 + freewheel, so a stopped-but-enabled motor barely dissipates
// (planets never need to hold position precisely) — the requested hold = 0.
static bool applyOneDriverVerified(TMC2209Stepper &d, char id, uint8_t maxTries) {
  uint16_t goalUs = (g_microsteps <= 1) ? 1 : g_microsteps;
  for (uint8_t attempt = 1; attempt <= maxTries; attempt++) {
    uint8_t before = d.IFCNT();
    d.rms_current(g_currentMa, 0.0f);       // IRUN from mA; IHOLD = 0 (see freewheel below)
    uint8_t msTries;
    setMicrostepsVerified(d, g_microsteps, msTries);   // maps 1->0, own inner read-back+retry
    d.intpol(g_interpolate);                // MicroPlyer
    d.en_spreadCycle(g_spread);             // false = stealthChop (quiet), true = spreadCycle
    d.TPOWERDOWN(20);                        // ~0.3 s after stop, drop to standstill state
    d.freewheel(1);                          // IHOLD=0 -> coils freewheel at standstill (no heat)
    uint8_t ifDelta = (uint8_t)(d.IFCNT() - before);

    uint16_t usRb = d.microsteps(); if (usRb == 0) usRb = 1;   // RW read-backs from the chip
    bool intRb = d.intpol();
    bool spRb  = d.en_spreadCycle();
    bool ok = (usRb == goalUs) && (intRb == g_interpolate) && (spRb == g_spread) && (ifDelta > 0);
    Serial.printf("apply %c try%u: usteps=%u(want %u) mplyer=%d(%d) spread=%d(%d) I=%umA IFCNT+%u => %s\n",
                  id, attempt, usRb, goalUs, intRb, g_interpolate, spRb, g_spread,
                  g_currentMa, ifDelta, ok ? "OK" : "RESEND");
    if (ok) return true;
  }
  return false;
}

static void applyDriverParams() {
  bool okL = applyOneDriverVerified(*drvL, 'L', 3);
  bool okR = applyOneDriverVerified(*drvR, 'R', 3);
  g_cfgOk = okL && okR;
  Serial.printf("driver cfg %s: I=%umA usteps=%u mode=%s mplyer=%d\n",
                g_cfgOk ? "VERIFIED" : "UNVERIFIED(!)", g_currentMa, g_microsteps,
                g_spread ? "spread" : "stealth", g_interpolate);
}

// Find the two wheel drivers: the two addresses that answer, told apart by which one sees
// PIN_STEP_L held high in its IOIN register. Runs with EN high (outputs off), so the one STEP edge
// only moves the driver's internal microstep counter. Silent on failure (tmcProbe() reports it).
static bool tmcFindDrivers() {
  uint8_t a[4], n = 0;
  for (uint8_t i = 0; i < 4; i++) if (g_drv[i].version() == 0x21) a[n++] = i;
  if (n != 2) return false;
  ledcDetachPin(PIN_STEP_L);
  pinMode(PIN_STEP_L, OUTPUT); digitalWrite(PIN_STEP_L, HIGH);
  bool s0 = g_drv[a[0]].step(), s1 = g_drv[a[1]].step();
  digitalWrite(PIN_STEP_L, LOW);
  ledcAttachPin(PIN_STEP_L, LEDC_CH_L);
  if (s0 == s1) return false;
  drvL = &g_drv[s0 ? a[0] : a[1]];
  drvR = &g_drv[s0 ? a[1] : a[0]];
  g_drvFound = true;
  Serial.printf("TMC drivers found: L at address %u, R at address %u\n",
                (unsigned)(drvL - g_drv), (unsigned)(drvR - g_drv));
  return true;
}

// The TMC2209 logic runs off its internal 5V regulator, i.e. off VM: with VM switched off the
// drivers are unpowered and silent. Wait (bounded) until both answer (IOIN VERSION = 0x21),
// finding them first if that hasn't happened yet.
static bool driversReady(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  do {
    if (!g_drvFound) { if (tmcFindDrivers()) return true; }
    else if (drvL->version() == 0x21 && drvR->version() == 0x21) return true;
  } while (millis() - t0 < timeoutMs);
  return false;
}

// Which UART addresses answer? Printed when the drivers can't be found, to tell a missing motor
// supply or a broken bus (none answer) from odd address straps.
static void tmcProbe() {
  Serial.print("TMC probe, addresses answering:");
  bool any = false;
  for (uint8_t a = 0; a < 4; a++)
    if (g_drv[a].version() == 0x21) { Serial.printf(" %u", a); any = true; }
  Serial.println(any ? "" : " none (motor supply off, or the UART line doesn't reach the drivers)");
}

// Full driver config: GCONF (which also carries the shaft/direction bit) + all params, verified.
// Then clear GSTAT.reset, so a later wake can tell whether a VM power cycle wiped the registers.
static void driversConfigure() {
  TMC2209Stepper *all[2] = {drvL, drvR};
  for (auto *d : all) {
    d->begin();
    d->toff(0);                 // chopper off until motors are enabled (driver fully idle/cool)
    d->pdn_disable(true);       // required so UART (not the PDN pin) controls the driver
    d->I_scale_analog(false);   // internal Vref (ignore VREF pin)
    d->mstep_reg_select(true);  // microsteps set over UART, not MS1/MS2 pins
  }
  applyDriverParams();
  drvL->GSTAT(0b111); drvR->GSTAT(0b111);       // write-1-to-clear reset/drv_err/uv_cp
  g_cfgStale = false;
}

// Take new driver params from a command. With the motor supply asleep the drivers can't be written,
// so just mark the config stale; motorsEnable(true) applies it on wake.
static void driverParamsSet(uint16_t ma, uint16_t us, bool spread, bool interp) {
  g_currentMa = ma; g_microsteps = us; g_spread = spread; g_interpolate = interp;
  if (g_motorsOn) applyDriverParams();
  else            g_cfgStale = true;
}

// Boot: power VM just long enough to configure + verify the drivers (EN stays high, so the outputs
// stay off and no motor current flows), then put the motor supply back to sleep.
static void tmcBringup() {
  TMC_SERIAL.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  digitalWrite(PIN_MOTOR_PWR, HIGH);
  if (driversReady(TMC_READY_TIMEOUT_MS)) driversConfigure();
  else { g_cfgOk = false; Serial.println("TMC drivers not answering on UART (VM missing?)"); tmcProbe(); }
  digitalWrite(PIN_MOTOR_PWR, LOW);
}

// Enable/disable the drivers. Levers, strongest first:
//   1. PIN_MOTOR_PWR cuts the motor supply (VM) entirely when idle — true sleep, ~zero draw.
//      Where that really removes VM the drivers reset to OTP defaults (wrong microsteps, current,
//      direction), so every wake checks GSTAT.reset and re-configures them before driving.
//   2. EN pin high disables the output stage.  3. toff=0 gates the chopper off.
static void motorsEnable(bool on) {
  if (on) {
    digitalWrite(PIN_MOTOR_PWR, HIGH);          // wake: motor supply on
    if (!driversReady(TMC_READY_TIMEOUT_MS)) {
      g_cfgOk = false;
      Serial.println("wake: TMC drivers not answering on UART (VM missing?)");
      tmcProbe();
    } else if (g_cfgStale || !g_cfgOk || drvL->reset() || drvR->reset()) {
      Serial.println("wake: drivers reset by VM power-up (or params changed) -> configuring");
      driversConfigure();
    }
    drvL->toff(4); drvR->toff(4);               // re-assert chopper
    digitalWrite(PIN_TMC_EN, LOW);              // enable outputs (active-low)
  } else {
    ledcWrite(LEDC_CH_L, 0); ledcWrite(LEDC_CH_R, 0);   // STEP lines low: stop the step trains
    g_velCurL = g_velCurR = 0; g_velStartL = g_velStartR = 0;
    g_velFinalL = g_velFinalR = 0; g_actHzL = g_actHzR = 0; g_rampFrac = 1.0f;
    digitalWrite(PIN_TMC_EN, HIGH);             // disable outputs
    drvL->toff(0); drvR->toff(0);               // chopper off
    digitalWrite(PIN_MOTOR_PWR, LOW);           // sleep: cut motor supply
  }
  g_motorsOn = on;
}

// Drive the light: g_lightBrightness when on, else dark. Brightness is perceptual, so gamma maps it
// to a PWM duty (50 % looks half as bright, not nearly full); any non-zero setting keeps >= 1 count.
// (The core's ledcWrite treats the max count as fully on.)
static void lightRender() {
  const uint32_t maxDuty = (1u << LIGHT_PWM_BITS) - 1;
  uint32_t duty = 0;
  if (g_lightOn && g_lightBrightness > 0) {
    duty = (uint32_t)lroundf(powf(g_lightBrightness / 100.0f, LIGHT_GAMMA) * maxDuty);
    if (duty < 1) duty = 1;
  }
  ledcWrite(LIGHT_LEDC_CH, duty);
}
static void lightSet(bool on) {
  g_lightOn = on;
  lightRender();
}
static void lightSetBrightness(uint8_t pct) {
  g_lightBrightness = pct > 100 ? 100 : pct;
  if (g_lightOn) lightRender();
}

static inline bool wheelsMoving() {
  return (fabsf(g_velCurL) > 1.0f) || (fabsf(g_velCurR) > 1.0f);
}

// Set new wheel velocity targets from signed rpm and restart the shared ramp fraction at 0.
// Because both wheels interpolate from their
// start velocity to their final target on the SAME fraction, the L:R speed ratio is preserved for
// the whole ramp (when starting from / ending at a stop, ratio == commanded ratio exactly).
// Direction is the TMC shaft bit, set once here from the target sign (a stop keeps the last dir);
// g_revL/g_revR reverse a wheel for this cart's mounting.
static void wheelSetTargets(float lRpm, float rRpm) {
  float lHz = rpmToUstepsHz(lRpm) * (g_revL ? -1.0f : 1.0f);  // signed µsteps/s
  float rHz = rpmToUstepsHz(rRpm) * (g_revR ? -1.0f : 1.0f);
  if (fabsf(lHz) > 0.5f) drvL->shaft(lHz < 0);              // shaft bit selects direction
  if (fabsf(rHz) > 0.5f) drvR->shaft(rHz < 0);
  g_velStartL = g_velCurL; g_velStartR = g_velCurR;
  g_velFinalL = lHz;       g_velFinalR = rHz;
  g_rampFrac  = 0.0f;
}

// Software frequency ramp: advance the shared fraction, compute each wheel's velocity, and emit
// the matching LEDC frequency. The LEDC hardware keeps producing the square wave between updates,
// so a busy CPU / Wi-Fi never interrupts the step train. When the wheels reach a full stop, run
// any deferred action (power-off, or the microsteps/params-change resume).
static void serviceRamp() {
  uint32_t now = millis();
  if (now - g_rampMs < WHEEL_UPDATE_MS) return;
  float dt = (now - g_rampMs) / 1000.0f;
  g_rampMs = now;

  if (g_rampFrac < 1.0f) {
    g_rampFrac += dt / WHEEL_RAMP_S;               // fixed-time ramp -> preserves the L:R ratio
    if (g_rampFrac > 1.0f) g_rampFrac = 1.0f;
  }
  g_velCurL = g_velStartL + g_rampFrac * (g_velFinalL - g_velStartL);
  g_velCurR = g_velStartR + g_rampFrac * (g_velFinalR - g_velStartR);

  if (g_motorsOn) {
    stepEmit(LEDC_CH_L, fabsf(g_velCurL), g_actHzL);
    stepEmit(LEDC_CH_R, fabsf(g_velCurR), g_actHzR);
  } else {
    g_actHzL = g_actHzR = 0;
  }

  bool stopped  = (fabsf(g_velCurL) < 1.0f) && (fabsf(g_velCurR) < 1.0f);
  if (!(stopped && g_rampFrac >= 1.0f)) return;    // only act once fully ramped to a stop

  if (g_resumePending) {                  // params change: apply at standstill, then re-accelerate
    g_resumePending = false;
    g_currentMa  = g_pendingCurrent;
    g_microsteps = g_pendingMicro;
    g_spread     = g_pendingSpread;
    g_interpolate = g_pendingInterp;
    g_revL = g_pendingRevL; g_revR = g_pendingRevR;
    applyDriverParams();
    Serial.printf("params applied at standstill (usteps=%u); re-accelerating to L=%.0f R=%.0f rpm\n",
                  g_microsteps, g_pendingRpmL, g_pendingRpmR);
    g_cmdWheelL = g_pendingRpmL; g_cmdWheelR = g_pendingRpmR;
    wheelSetTargets(g_pendingRpmL, g_pendingRpmR);
  } else if (g_stopThenOff && g_motorsOn) {   // ramped to a stop for CMD_STOP / homing: cut power
    g_stopThenOff = false;
    motorsEnable(false);
  }
}

// ----------------------------- ESP-NOW --------------------------------------
static void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;              // 0 = use current Wi-Fi channel
  p.ifidx   = WIFI_IF_STA;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

// Command handoff. The ESP-NOW recv callback runs in the Wi-Fi task; it must NOT touch the TMC
// UART or motor state (that all runs from loop()). It only copies the packet and raises a flag;
// serviceCommand() below does the work from loop() so all driver access is serialized.
static volatile bool g_cmdPending = false;
static CmdMsg        g_rxCmd;

static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 2 || data[0] != SOLAR_MAGIC) return;
  if (data[1] == SOLAR_MSG_FW_DATA) {   // image chunk from our Sun: queue it for loop()
    const FwDataMsg *c = (const FwDataMsg *)data;
    if (len < (int)SOLAR_FW_DATA_HDR || !g_sunKnown || memcmp(mac, g_sunMac, 6) != 0) return;
    if (c->len > SOLAR_FW_CHUNK || SOLAR_FW_DATA_HDR + c->len > (size_t)len) return;
    uint8_t next = (g_fwRxHead + 1) % FW_RXQ;
    if (next == g_fwRxTail) return;                    // full: the cart will ask again
    memcpy(&g_fwRxq[g_fwRxHead], data, len);
    g_fwRxHead = next;
    return;
  }
  if (data[1] != SOLAR_MSG_CMD || len < (int)sizeof(CmdMsg)) return;

  g_lastCmdMs = millis();
  if (!g_sunKnown) {              // learn the Sun and switch to unicast replies (ESP-NOW only)
    memcpy(g_sunMac, mac, 6);
    g_sunKnown = true;
    addPeer(g_sunMac);
  }
  memcpy(&g_rxCmd, data, sizeof(CmdMsg));
  g_cmdPending = true;
}

// Live Sun signal. The ESP-NOW receive callback on this core (IDF 4.4) carries no RSSI, so a
// management-frame sniffer reads it off the Sun's own ESP-NOW frames (802.11 action frames, one
// every ~2 s) and keeps a smoothed value. Runs in the Wi-Fi task, like onRecv.
static void onSniff(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT || !g_sunKnown) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  if (p->rx_ctrl.sig_len < 24 || p->payload[0] != 0xD0) return;   // 0xD0 = action frame
  if (memcmp(p->payload + 10, g_sunMac, 6) != 0) return;           // addr2 = sender
  int16_t r = (int16_t)(p->rx_ctrl.rssi * 16), q = g_rssiQ16;
  g_rssiQ16 = q ? (int16_t)(q + (r - q) / 4) : r;
}

static void sniffEnable(bool on) {
  if (on) {
    wifi_promiscuous_filter_t f = {WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&f);
    esp_wifi_set_promiscuous_rx_cb(onSniff);
  }
  esp_wifi_set_promiscuous(on);
}

static void rssiRefresh() {
  int16_t q = g_rssiQ16;
  if (q) g_sunRssi = (int8_t)lroundf(q / 16.0f);
}

// Apply the most recent command. Runs in loop() (main task). The Sun resends the active command
// every ~2 s as a heartbeat, so this acts only on genuine changes.
static void serviceCommand() {
  if (!g_cmdPending) return;
  g_cmdPending = false;
  CmdMsg m = g_rxCmd;

  g_myIndex = m.planet_index;
  if (m.step_degree > 0.01f) g_stepDegree = m.step_degree;
  g_offerStamp = m.fw_stamp; g_offerSize = m.fw_size;   // cart image on offer (0 = none / paused)
  g_offerForce = (m.fw_flags & SOLAR_FW_FORCE) != 0;
  memcpy(g_offerMd5, m.fw_md5, sizeof(g_offerMd5));
  lightSetBrightness(m.light_brightness);           // per-planet brightness chosen on the Sun

  // Unpack the driver-config byte (microsteps enum + chopper + MicroPlyer bits).
  uint16_t effUs = solarMicrostepsFromEnum(m.driver_cfg & 0x0F);
  bool spread = (m.driver_cfg & SOLAR_DRVCFG_CHOP)   != 0;
  bool interp = (m.driver_cfg & SOLAR_DRVCFG_INTERP) != 0;
  bool revL   = (m.driver_cfg & SOLAR_DRVCFG_REV_L) != 0;
  bool revR   = (m.driver_cfg & SOLAR_DRVCFG_REV_R) != 0;
  bool microChanged   = (effUs != g_microsteps);
  bool currentChanged = (m.current_ma != g_currentMa);
  bool chopChanged    = (spread != g_spread) || (interp != g_interpolate);
  bool paramsChanged  = microChanged || currentChanged || chopChanged;
  bool revChanged     = (revL != g_revL) || (revR != g_revR);

  switch (m.command) {
    case CMD_STOP:
      g_revL = revL; g_revR = revR;             // takes effect on the next start
      g_homingStop = false; g_resumePending = false;
      g_cmdWheelL = 0; g_cmdWheelR = 0;
      if (g_motorsOn) { g_stopThenOff = true; wheelSetTargets(0, 0); }  // decelerate, then power off
      lightSet(false);
      break;

    case CMD_RUN:
      g_homingStop = false; g_stopThenOff = false;
      // Changing microsteps while turning would jump the speed; decelerate to a stop, apply the new
      // params at standstill, then re-accelerate (serviceRamp completes the sequence). Reliable with
      // LEDC — no step-generator restart involved. (Current/chopper/MicroPlyer alone don't jump the
      // speed, so those are applied live below without stopping.) A wheel reversal goes the same
      // way: flipping the shaft bit at speed would stall the motor.
      if ((microChanged || revChanged) && wheelsMoving()) {
        Serial.printf("microsteps %u->%u / reverse L%d R%d while running: STOP, apply, then re-accelerate\n",
                      g_microsteps, effUs, revL, revR);
        g_resumePending = true;
        g_pendingMicro = effUs; g_pendingCurrent = m.current_ma;
        g_pendingSpread = spread; g_pendingInterp = interp;
        g_pendingRevL = revL; g_pendingRevR = revR;
        g_pendingRpmL = m.wheel_l_rpm; g_pendingRpmR = m.wheel_r_rpm;
        wheelSetTargets(0, 0);
      } else if (!g_resumePending) {
        if (paramsChanged) driverParamsSet(m.current_ma, effUs, spread, interp);
        g_revL = revL; g_revR = revR;
        bool wasOff = !g_motorsOn;
        if (wasOff) motorsEnable(true);
        // The Sun re-sends the same CMD_RUN every ~2 s as a heartbeat; only (re)start the ramp on a
        // genuine speed change (or a fresh enable), so an identical heartbeat doesn't reset it.
        bool targetChanged = wasOff || revChanged ||
                             (m.wheel_l_rpm != g_cmdWheelL) || (m.wheel_r_rpm != g_cmdWheelR);
        g_cmdWheelL = m.wheel_l_rpm; g_cmdWheelR = m.wheel_r_rpm;
        if (targetChanged) wheelSetTargets(m.wheel_l_rpm, m.wheel_r_rpm);
      }
      lightSet(m.light_on);
      break;

    case CMD_HOME_AND_STOP:           // drive slowly toward HOME; homing logic stops on the pulse
      g_resumePending = false; g_stopThenOff = false;
      if (paramsChanged) driverParamsSet(m.current_ma, effUs, spread, interp);
      g_revL = revL; g_revR = revR;             // homing is slow: a reversal can apply at once
      if (!g_homingStop) g_homeSeen = false;    // clear on entry only, not every heartbeat
      g_homingStop = true;
      if (!g_motorsOn) motorsEnable(true);
      g_cmdWheelL = m.wheel_l_rpm; g_cmdWheelR = m.wheel_r_rpm;
      wheelSetTargets(m.wheel_l_rpm, m.wheel_r_rpm);
      lightSet(m.light_on);
      break;

    case CMD_LIGHT:
      lightSet(m.light_on);
      break;
  }
}

static void sendStatus() {
  StatusMsg s = {};
  s.magic        = SOLAR_MAGIC;
  s.type         = SOLAR_MSG_STATUS;
  s.planet_index = g_myIndex;
  s.home_pulsed  = g_homePulse ? 1 : 0;
  s.error        = ERR_NONE;                 // motor stuck/disconnected sensing was removed
  s.light_on     = g_lightOn ? 1 : 0;
  s.moving       = wheelsMoving() ? 1 : 0;
  g_dieTempC     = readDieTempC();
  s.temp_c       = (int8_t)lroundf(constrain(g_dieTempC, -128.0f, 127.0f));
  rssiRefresh();
  s.rssi         = g_sunRssi;
  s.cfg_ok       = g_cfgOk ? 1 : 0;
  s.fw_stamp     = g_fwStamp;
  s.ota_state    = g_otaState;
  s.ota_progress = g_otaProgress;
  s.ota_err      = g_otaErr;
  g_homePulse    = false;

  const uint8_t *dst = g_sunKnown ? g_sunMac : BCAST_MAC;
  esp_now_send(dst, (uint8_t *)&s, sizeof(s));
}

// Scan for the Sun's AP to discover its Wi-Fi channel, then lock onto it. Blocking (~2s);
// only called at boot and while OFFLINE, so it never stalls active operation. The live signal
// reading restarts from the scan's value.
static bool discoverChannel() {
  uint32_t t0 = millis();
  sniffEnable(false);                     // scan with the sniffer off
  int n = WiFi.scanNetworks(false, true);
  int ch = -1;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == SOLAR_AP_SSID_DEFAULT) {
      ch = WiFi.channel(i);
      g_sunRssi = (int8_t)WiFi.RSSI(i);   // record link quality to the Sun's AP
      break;
    }
  }
  WiFi.scanDelete();
  g_rssiQ16 = 0;
  Serial.printf("[%lu] LINK rescan: Sun on ch %d (%d dBm), took %lu ms\n", (unsigned long)millis(),
                ch, g_sunRssi, (unsigned long)(millis() - t0));
  if (ch >= 0) esp_wifi_set_channel((uint8_t)ch, WIFI_SECOND_CHAN_NONE);
  sniffEnable(true);
  return ch >= 0;
}

// ----------------------------- Firmware update ------------------------------
// Pull the Sun's cart image when its build is newer than ours: ask for FW_BURST chunks from the
// current offset, write the ones that arrive in order (Update buffers them into flash sectors),
// ask again. Runs only while the motors are off and the Sun offers the image (it stops offering
// while a show runs -> the download pauses where it is). Update.end() checks the MD5 and switches
// the boot slot; then the cart reboots into the new build.
static void fwRequest() {
  FwReqMsg r = {SOLAR_MAGIC, SOLAR_MSG_FW_REQ, g_dlStamp, g_dlOffset, FW_BURST};
  esp_now_send(g_sunMac, (const uint8_t *)&r, sizeof(r));
  g_dlReqMs = millis(); g_dlGot = 0;
}

static void fwStop(uint8_t state, int16_t err) {      // OTA_IDLE = abandoned, OTA_FAILED = error
  if (Update.isRunning()) Update.abort();
  g_dlActive = false;
  g_otaState = state; g_otaErr = err;
  if (state == OTA_FAILED) {
    g_dlFailStamp = g_dlStamp; g_dlFailMs = millis();
    Serial.printf("FW download of build %lu failed (err %d); retry in %us\n",
                  (unsigned long)g_dlStamp, err, FW_RETRY_MS / 1000);
  }
}

static void serviceFwDownload() {
  uint32_t now = millis();
  // Only ever move to a NEWER build: a cart flashed with something newer (USB, testing) stays put,
  // unless the Sun's page pushed its image to the carts (forced).
  bool wanted  = g_offerStamp > g_fwStamp || (g_offerForce && g_offerStamp && g_offerStamp != g_fwStamp);
  bool offered = wanted && g_sunKnown && !g_motorsOn && !g_selfTest;
  if (!g_dlActive) {
    g_fwRxTail = g_fwRxHead;                            // drop stray chunks
    if (!offered) return;
    if (g_offerStamp == g_dlFailStamp && now - g_dlFailMs < FW_RETRY_MS) return;
    g_dlStamp = g_offerStamp; g_dlSize = g_offerSize;
    if (!Update.begin(g_dlSize, U_FLASH)) { fwStop(OTA_FAILED, OTA_ERR_BEGIN); return; }
    char md5[33];
    for (int k = 0; k < 16; k++) sprintf(md5 + 2 * k, "%02x", g_offerMd5[k]);
    Update.setMD5(md5);
    g_dlActive = true; g_dlOffset = 0; g_dlDataMs = now;
    g_otaState = OTA_DOWNLOADING; g_otaProgress = 0; g_otaErr = 0;
    Serial.printf("FW download: build %lu -> %lu (%lu bytes)\n", (unsigned long)g_fwStamp,
                  (unsigned long)g_dlStamp, (unsigned long)g_dlSize);
    fwRequest();
    return;
  }
  if (g_offerStamp && g_offerStamp != g_dlStamp) {     // a new image was uploaded meanwhile
    Serial.println("FW download: the Sun's image changed, starting over");
    fwStop(OTA_IDLE, 0);
    return;
  }
  // Write queued chunks that continue the image; duplicates / out-of-order ones are dropped
  // (the next request restarts from the first missing byte).
  while (g_fwRxTail != g_fwRxHead) {
    const FwDataMsg &c = g_fwRxq[g_fwRxTail];
    bool next = c.fw_stamp == g_dlStamp && c.offset == g_dlOffset && c.len && g_dlOffset + c.len <= g_dlSize;
    if (next && Update.write((uint8_t *)c.data, c.len) != c.len) {
      g_fwRxTail = (g_fwRxTail + 1) % FW_RXQ;
      fwStop(OTA_FAILED, Update.getError());
      return;
    }
    if (next) {
      g_dlOffset += c.len; g_dlDataMs = millis();
      g_otaProgress = (uint8_t)((uint64_t)g_dlOffset * 100 / g_dlSize);
    }
    g_fwRxTail = (g_fwRxTail + 1) % FW_RXQ;
    g_dlGot++;
  }
  if (g_dlOffset >= g_dlSize) {
    if (!Update.end()) { fwStop(OTA_FAILED, Update.getError()); return; }   // MD5 + boot slot
    g_dlActive = false; g_otaState = OTA_DONE; g_otaProgress = 100;
    Serial.println("FW download complete and MD5-verified: rebooting into it");
    for (int k = 0; k < 3; k++) { sendStatus(); delay(150); }
    ESP.restart();
  }
  now = millis();                                      // the writes above take time: re-read the
                                                       // clock, or now - g_dlDataMs wraps negative
  if (!offered) { g_dlDataMs = now; return; }          // paused (show running): keep our place
  if (now - g_dlDataMs > FW_NO_DATA_MS) { fwStop(OTA_FAILED, OTA_ERR_NO_DATA); return; }
  if (g_dlGot >= FW_BURST || now - g_dlReqMs > FW_REQ_TIMEOUT_MS) fwRequest();
}

static bool espnowStart() {
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  addPeer(BCAST_MAC);
  return true;
}

// ----------------------------- Status LED -----------------------------------
static void serviceStatusLed() {
  uint16_t onMs, offMs;
  switch (g_state) {
    case ST_OFFLINE: onMs = 200; offMs = 1000; break;
    case ST_IDLE:    onMs = 100; offMs = 400;  break;
    default:         onMs = 100; offMs = 200;  break;   // ST_ACTIVE
  }
  uint32_t period = onMs + offMs;
  bool on = (millis() % period) < onMs;
  digitalWrite(PIN_STATUS_LED, on ? LOW : HIGH);        // active-low
}

// ----------------------------- Homing ---------------------------------------
static void serviceHoming() {
  if (g_homingStop && g_homeSeen) {
    g_homingStop = false;
    g_stopThenOff = true;      // decelerate to a stop on the ramp, then cut power (serviceRamp)
    g_cmdWheelL = 0; g_cmdWheelR = 0;
    wheelSetTargets(0, 0);
  }
}

// ----------------------------- Button self-test ------------------------------
static void serviceButton() {
  static uint32_t lastEdge = 0;
  bool pressed = (digitalRead(PIN_BUTTON) == LOW);
  if (pressed && g_state == ST_OFFLINE && !g_selfTest && millis() - lastEdge > 300) {
    lastEdge = millis();
    g_selfTest = true;
    g_selfTestEndMs = millis() + 5000;
    motorsEnable(true);                 // jog both wheels slowly + light: a lone-cart self-test
    g_cmdWheelL = 60; g_cmdWheelR = 60;
    wheelSetTargets(60, 60);
    lightSet(true);
  }
  if (g_selfTest && millis() > g_selfTestEndMs) {
    g_selfTest = false;
    g_stopThenOff = true;
    g_cmdWheelL = 0; g_cmdWheelR = 0;
    wheelSetTargets(0, 0);
    lightSet(false);
  }
}

// ----------------------------- Link watch -----------------------------------
// Log when commands from the Sun stop for longer than LINK_GAP_MS and when they come back
// (the Sun sends each cart a command every ~2 s), to diagnose link drops.
#define LINK_GAP_MS 3000
static void serviceLinkWatch() {
  static bool     gap = false;
  static uint32_t gapFrom = 0;
  uint32_t last = g_lastCmdMs, now = millis();
  if (!g_sunKnown) return;
  if (!gap && now - last > LINK_GAP_MS) {
    gap = true; gapFrom = last;
    Serial.printf("[%lu] LINK gap: no command from the Sun for %lu ms\n", (unsigned long)now,
                  (unsigned long)(now - last));
  } else if (gap && last != gapFrom) {
    gap = false;
    Serial.printf("[%lu] LINK back: commands again after %lu ms\n", (unsigned long)now,
                  (unsigned long)(last - gapFrom));
  }
}

// ----------------------------- State update ---------------------------------
static void updateState() {
  if (msSince(g_lastCmdMs) > 5000) {
    g_state = ST_OFFLINE;
  } else if (g_motorsOn || g_lightOn) {
    g_state = ST_ACTIVE;
  } else {
    g_state = ST_IDLE;
  }
}

// ----------------------------- Setup / loop ----------------------------------
void setup() {
  pinMode(PIN_LIGHT_EN, OUTPUT); digitalWrite(PIN_LIGHT_EN, LOW);   // light off at once: no floating
                                                                    // MOSFET gate during the USB wait
  Serial.begin(115200);
  // Native USB serial (HWCDC) waits while its TX buffer is full - i.e. whenever USB is plugged
  // into a computer that isn't reading the port - until the TX timeout, then treats the port as
  // disconnected and drops output until the host reads again. Keep that wait to 1 ms. NOT 0: on
  // core 2.0.17 write() counts its no-progress tries in a uint32 that starts at the timeout, so
  // 0 wraps to ~4e9 and the cart's loop() freezes until someone opens the port.
  Serial.setTxTimeoutMs(1);
  uint32_t tUsb = millis();                // wait for the USB-CDC host to attach so the
  while (!Serial && millis() - tUsb < 3000) delay(10);   // boot banner isn't lost
  delay(300);
  Serial.println("\n==== Solar System - PLANET boot ====");
  time_t bt = g_fwStamp; struct tm b; gmtime_r(&bt, &b);
  Serial.printf("Firmware build %s = %04d-%02d-%02d %02d:%02d:%02d UTC\n", FW_ID + sizeof(SOLAR_FW_MARK) - 1,
                b.tm_year + 1900, b.tm_mon + 1, b.tm_mday, b.tm_hour, b.tm_min, b.tm_sec);

  pinMode(PIN_MOTOR_PWR, OUTPUT);  digitalWrite(PIN_MOTOR_PWR, LOW);  // motor supply asleep
  pinMode(PIN_TMC_EN, OUTPUT);     digitalWrite(PIN_TMC_EN, HIGH);   // drivers off
  // LEDC step generation: each STEP pin on a channel of its own timer (ch0/timer0, ch2/timer1),
  // both off the one 80 MHz clock so the two wheels stay frequency-synchronized. stepEmit() later
  // sets the actual step rate with a per-frequency resolution for accuracy; start silent (duty 0).
  ledcSetup(LEDC_CH_L, 1000, 8); ledcAttachPin(PIN_STEP_L, LEDC_CH_L); ledcWrite(LEDC_CH_L, 0);
  ledcSetup(LEDC_CH_R, 1000, 8); ledcAttachPin(PIN_STEP_R, LEDC_CH_R); ledcWrite(LEDC_CH_R, 0);
  ledcSetup(LIGHT_LEDC_CH, LIGHT_PWM_HZ, LIGHT_PWM_BITS); ledcAttachPin(PIN_LIGHT_EN, LIGHT_LEDC_CH);
  lightSet(false);                 // light dark until the Sun turns it on
  pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, HIGH);
  pinMode(PIN_HOME, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HOME), homeIsr, FALLING);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);   // run the radio cooler (see notes above)
  Serial.print("Planet MAC: ");
  Serial.println(WiFi.macAddress());     // provision this into the Sun's mac_address[]

  tmcBringup();

  if (!discoverChannel()) {
    Serial.println("Sun AP not found yet; will retry while OFFLINE.");
  }
  if (!espnowStart()) {
    Serial.println("ESP-NOW init failed!");
  }
  g_nextStatusMs = millis() + 1000;
  g_rampMs = millis();
}

void loop() {
  uint32_t now = millis();

  updateState();
  serviceLinkWatch();
  serviceStatusLed();
  serviceButton();
  serviceCommand();
  serviceFwDownload();
  serviceHoming();
  serviceRamp();

  // Periodic status transmit, jittered 1-3s to avoid collisions among the 8 carts.
  if (now >= g_nextStatusMs) {
    sendStatus();
    g_nextStatusMs = now + 1000 + (esp_random() % 2000);
  }

  // While offline, periodically re-scan for the Sun's channel (it may move when the Sun's
  // STA connects to a router and drags the SoftAP to a new channel).
  if (g_state == ST_OFFLINE && now - g_channelScanMs > OFFLINE_RESCAN_MS) {
    g_channelScanMs = now;
    discoverChannel();
  }

  // Debug heartbeat: always emit something so a late-connecting monitor sees the cart is alive.
  if (now - g_lastDbgMs > 2000) {
    g_lastDbgMs = now;
    g_dieTempC = readDieTempC();
    rssiRefresh();
    const char *st = g_state == ST_OFFLINE ? "OFFLINE" : g_state == ST_IDLE ? "IDLE" : "ACTIVE";
    bool home = (digitalRead(PIN_HOME) == LOW);          // active-low sensor
    // Current wheel velocities and the equivalent rpm derived from the ACTUAL emitted LEDC step
    // frequency, so a bench check (scope on STEP) can confirm the motor turns at the commanded
    // speed and that the L:R ratio holds throughout the ramp.
    float fullSteps = 360.0f / (g_stepDegree > 0.01f ? g_stepDegree : 1.8f);
    float rpmL = g_actHzL / (fullSteps * g_microsteps) * 60.0f * (g_velCurL < 0 ? -1 : 1);
    float rpmR = g_actHzR / (fullSteps * g_microsteps) * 60.0f * (g_velCurR < 0 ? -1 : 1);
    Serial.printf("[%lus] %s idx=%u sun=%d | signal=%d%%(%ddBm) temp=%.1fC home=%d | "
                  "TMC:I=%umA usteps=%u %s%s cfg=%d rev=%s | "
                  "cmd L=%.0f R=%.0f rpm -> now L=%.0f R=%.0f rpm (%.0f/%.0f Hz) | "
                  "light=%d(%u%%) motors=%d\n",
                  (unsigned long)(now / 1000), st, g_myIndex, g_sunKnown ? 1 : 0,
                  rssiPct(g_sunRssi), g_sunRssi, g_dieTempC, home ? 1 : 0,
                  g_currentMa, g_microsteps,
                  g_spread ? "spread" : "stealth", g_interpolate ? " mplyer" : "", g_cfgOk ? 1 : 0,
                  g_revL ? (g_revR ? "LR" : "L") : (g_revR ? "R" : "-"),
                  g_cmdWheelL, g_cmdWheelR, rpmL, rpmR, g_actHzL, g_actHzR,
                  g_lightOn ? 1 : 0, g_lightBrightness,
                  g_motorsOn ? 1 : 0);
  }
}
