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
#include <TMCStepper.h>
#include "solar_protocol.h"

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
static const uint8_t PIN_LIGHT_EN   = 7;   // N-MOS gate for planet light
static const uint8_t PIN_STATUS_LED = 8;   // onboard LED, active-low
static const uint8_t PIN_BUTTON     = 9;   // onboard boot button, active-low

// ----------------------------- Motor / TMC config ---------------------------
#define TMC_R_SENSE        0.11f
#define TMC_UART_BAUD      115200
#define TMC_ADDR_L         0b11            // MS1=VIO MS2=VIO
#define TMC_ADDR_R         0b10            // MS1=GND MS2=VIO
#define WHEEL_R_INVERT     true            // wheels are mounted back-to-back: flip the right one
                                           // so a "forward" command rolls both the same way.
// LEDC step generation. Channels must sit on DIFFERENT timers to run at independent speeds
// (timer = channel/2, one frequency per timer): ch0->timer0, ch2->timer1. Both timers still
// derive from the single 80 MHz APB clock, so the L:R frequency ratio stays exact (synchronized).
#define LEDC_CH_L          0
#define LEDC_CH_R          2
#define LEDC_CLK_HZ        80000000.0      // APB clock feeding LEDC
#define WHEEL_RAMP_S       1.0f            // seconds to accelerate 0->commanded speed (and back)
#define WHEEL_UPDATE_MS    20              // how often the software frequency ramp advances
// StallGuard "stuck" detection uses a threshold sent from the Sun (settings: stall_threshold,
// runtime g_stallThreshold): if the StallGuard load drops below it while a wheel is commanded to
// move, the rotor is mechanically held. TMC2209 StallGuard needs per-rig tuning — watch sg=x/y in
// the serial heartbeat / the Sun UI and set the threshold a little above the "held" reading but
// below the free-running reading. 0 disables the check.
#define FAULT_DEBOUNCE     3               // consecutive bad polls before a fault is latched
// Stall detection uses a RUNNING AVERAGE of the StallGuard load per wheel, because a single
// SG_RESULT sample is noisy and reads low at low RPM (unreliable for a bare threshold). The EMA
// is (re)armed at SG_AVG_INIT_MULT * threshold, so only several consecutive low readings pull it
// under the threshold -> a stall is declared (this also spaces out the auto-retry attempts).
#define SG_AVG_ALPHA       0.30f           // EMA weight per fault poll (higher = reacts faster)
#define SG_AVG_INIT_MULT   5               // arm the average at this * stall_threshold
#define STALL_RESTART_MS   800             // pause after the hard stop before spinning up again

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
static uint16_t g_stallThreshold = 0;      // StallGuard load below this while driving = stuck (0=off)
static bool     g_cfgOk       = false;     // last applyDriverParams() verified over UART?

HardwareSerial   TMC_SERIAL(1);            // ESP32-C3 UART1
TMC2209Stepper   drvL(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_L);
TMC2209Stepper   drvR(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_R);

// ----------------------------- Runtime state --------------------------------
enum PlanetState : uint8_t { ST_OFFLINE, ST_IDLE, ST_ACTIVE };
static PlanetState g_state = ST_OFFLINE;

static uint8_t  g_myIndex     = PLANET_INDEX_UNKNOWN;
static bool     g_motorsOn    = false;     // drivers enabled (VM on, EN low)
static bool     g_lightOn     = false;
static bool     g_homingStop  = false;     // true while executing CMD_HOME_AND_STOP
static uint8_t  g_error       = ERR_NONE;

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

// Diagnostics reported back to the Sun and printed in the heartbeat.
static int8_t   g_sunRssi     = 0;         // Sun-AP RSSI (dBm) from the last channel scan
static float    g_dieTempC    = 0;         // ESP32-C3 internal temperature (°C)
static uint16_t g_sgL = 0, g_sgR = 0;      // last StallGuard load readings (for tuning)
static float    g_sgAvgL = 0, g_sgAvgR = 0;// running (EMA) StallGuard load per wheel
static uint32_t g_stallRecoverMs = 0;      // when !=0: millis() at which to spin up again after a stall
static uint8_t  g_drvStat = 0;             // last open-load/overtemp flag snapshot (bitfield)

static uint32_t g_lastCmdMs   = 0;         // last time a valid command arrived
static uint32_t g_nextStatusMs = 0;        // next scheduled status transmit
static uint32_t g_lastFaultMs = 0;         // last TMC fault poll
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

static const char *errText(uint8_t e) {
  switch (e) {
    case ERR_MOTOR_DISCONNECTED: return "MOTOR_DISCONNECTED";
    case ERR_MOTOR_STUCK:        return "MOTOR_STUCK";
    default:                     return "OK";
  }
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
  bool okL = applyOneDriverVerified(drvL, 'L', 3);
  bool okR = applyOneDriverVerified(drvR, 'R', 3);
  g_cfgOk = okL && okR;
  Serial.printf("driver cfg %s: I=%umA usteps=%u mode=%s mplyer=%d stall_thr=%u\n",
                g_cfgOk ? "VERIFIED" : "UNVERIFIED(!)", g_currentMa, g_microsteps,
                g_spread ? "spread" : "stealth", g_interpolate, g_stallThreshold);
}

static void tmcBringup() {
  TMC_SERIAL.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  TMC2209Stepper *all[2] = {&drvL, &drvR};
  for (auto *d : all) {
    d->begin();
    d->toff(0);                 // chopper off until motors are enabled (driver fully idle/cool)
    d->pdn_disable(true);       // required so UART (not the PDN pin) controls the driver
    d->I_scale_analog(false);   // internal Vref (ignore VREF pin)
    d->mstep_reg_select(true);  // microsteps set over UART, not MS1/MS2 pins
  }
  applyDriverParams();
}

// (Re)arm the running StallGuard average to its high init value (SG_AVG_INIT_MULT * threshold), so
// a fresh spin-up must gather several low readings before the average trips a stall.
static inline void resetSgAvg() {
  float init = (float)g_stallThreshold * SG_AVG_INIT_MULT;
  g_sgAvgL = g_sgAvgR = init;
}

// Enable/disable the drivers. Levers, strongest first:
//   1. PIN_MOTOR_PWR cuts the motor supply (VM) entirely when idle — true sleep, ~zero draw.
//      VCC_IO stays on 3.3V so the TMC register config survives and needs no re-init on wake.
//   2. EN pin high disables the output stage.  3. toff=0 gates the chopper off.
static void motorsEnable(bool on) {
  if (on) {
    digitalWrite(PIN_MOTOR_PWR, HIGH);          // wake: motor supply on
    delayMicroseconds(1500);                    // let VM settle before driving
    drvL.toff(4); drvR.toff(4);                 // re-assert chopper
    digitalWrite(PIN_TMC_EN, LOW);              // enable outputs (active-low)
    resetSgAvg();                               // start the stall average high (needs low run to trip)
  } else {
    ledcWrite(LEDC_CH_L, 0); ledcWrite(LEDC_CH_R, 0);   // STEP lines low: stop the step trains
    g_velCurL = g_velCurR = 0; g_velStartL = g_velStartR = 0;
    g_velFinalL = g_velFinalR = 0; g_actHzL = g_actHzR = 0; g_rampFrac = 1.0f;
    digitalWrite(PIN_TMC_EN, HIGH);             // disable outputs
    drvL.toff(0); drvR.toff(0);                 // chopper off
    digitalWrite(PIN_MOTOR_PWR, LOW);           // sleep: cut motor supply
  }
  g_motorsOn = on;
}

static void lightSet(bool on) {
  digitalWrite(PIN_LIGHT_EN, on ? HIGH : LOW);
  g_lightOn = on;
}

static inline bool wheelsMoving() {
  return (fabsf(g_velCurL) > 1.0f) || (fabsf(g_velCurR) > 1.0f);
}

// Set new wheel velocity targets from signed rpm and restart the shared ramp fraction at 0. The
// right wheel is inverted for the back-to-back mount. Because both wheels interpolate from their
// start velocity to their final target on the SAME fraction, the L:R speed ratio is preserved for
// the whole ramp (when starting from / ending at a stop, ratio == commanded ratio exactly).
// Direction is the TMC shaft bit, set once here from the target sign (a stop keeps the last dir).
static void wheelSetTargets(float lRpm, float rRpm) {
  float lHz = rpmToUstepsHz(lRpm);                          // signed µsteps/s
  float rHz = rpmToUstepsHz(rRpm) * (WHEEL_R_INVERT ? -1.0f : 1.0f);
  if (fabsf(lHz) > 0.5f) drvL.shaft(lHz < 0);               // shaft bit selects direction
  if (fabsf(rHz) > 0.5f) drvR.shaft(rHz < 0);
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
  if (len < 2 || data[0] != SOLAR_MAGIC || data[1] != SOLAR_MSG_CMD) return;
  if (len < (int)sizeof(CmdMsg)) return;

  g_lastCmdMs = millis();
  if (!g_sunKnown) {              // learn the Sun and switch to unicast replies (ESP-NOW only)
    memcpy(g_sunMac, mac, 6);
    g_sunKnown = true;
    addPeer(g_sunMac);
  }
  memcpy(&g_rxCmd, data, sizeof(CmdMsg));
  g_cmdPending = true;
}

// Apply the most recent command. Runs in loop() (main task). The Sun resends the active command
// every ~2 s as a heartbeat, so this acts only on genuine changes.
static void serviceCommand() {
  if (!g_cmdPending) return;
  g_cmdPending = false;
  CmdMsg m = g_rxCmd;

  g_myIndex = m.planet_index;
  if (m.step_degree > 0.01f) g_stepDegree = m.step_degree;
  g_stallThreshold = m.stall_threshold;     // stuck-detection threshold (0 = disabled)
  g_stallRecoverMs = 0;                      // a fresh command supersedes any pending auto-retry

  // Unpack the driver-config byte (microsteps enum + chopper + MicroPlyer bits).
  uint16_t effUs = solarMicrostepsFromEnum(m.driver_cfg & 0x0F);
  bool spread = (m.driver_cfg & SOLAR_DRVCFG_CHOP)   != 0;
  bool interp = (m.driver_cfg & SOLAR_DRVCFG_INTERP) != 0;
  bool microChanged   = (effUs != g_microsteps);
  bool currentChanged = (m.current_ma != g_currentMa);
  bool chopChanged    = (spread != g_spread) || (interp != g_interpolate);
  bool paramsChanged  = microChanged || currentChanged || chopChanged;

  switch (m.command) {
    case CMD_STOP:
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
      // speed, so those are applied live below without stopping.)
      if (microChanged && wheelsMoving()) {
        Serial.printf("microsteps %u->%u while running: STOP, apply, then re-accelerate\n",
                      g_microsteps, effUs);
        g_resumePending = true;
        g_pendingMicro = effUs; g_pendingCurrent = m.current_ma;
        g_pendingSpread = spread; g_pendingInterp = interp;
        g_pendingRpmL = m.wheel_l_rpm; g_pendingRpmR = m.wheel_r_rpm;
        wheelSetTargets(0, 0);
      } else if (!g_resumePending) {
        if (paramsChanged) {
          g_currentMa = m.current_ma; g_microsteps = effUs;
          g_spread = spread; g_interpolate = interp;
          applyDriverParams();
        }
        bool wasOff = !g_motorsOn;
        if (wasOff) motorsEnable(true);
        // The Sun re-sends the same CMD_RUN every ~2 s as a heartbeat; only (re)start the ramp on a
        // genuine speed change (or a fresh enable), so an identical heartbeat doesn't reset it.
        bool targetChanged = wasOff || (m.wheel_l_rpm != g_cmdWheelL) || (m.wheel_r_rpm != g_cmdWheelR);
        g_cmdWheelL = m.wheel_l_rpm; g_cmdWheelR = m.wheel_r_rpm;
        if (targetChanged) wheelSetTargets(m.wheel_l_rpm, m.wheel_r_rpm);
      }
      lightSet(m.light_on);
      break;

    case CMD_HOME_AND_STOP:           // drive slowly toward HOME; homing logic stops on the pulse
      g_resumePending = false; g_stopThenOff = false;
      if (paramsChanged) {
        g_currentMa = m.current_ma; g_microsteps = effUs;
        g_spread = spread; g_interpolate = interp;
        applyDriverParams();
      }
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
  s.error        = g_error;
  s.light_on     = g_lightOn ? 1 : 0;
  s.moving       = wheelsMoving() ? 1 : 0;
  g_dieTempC     = readDieTempC();
  s.temp_c       = (int8_t)lroundf(constrain(g_dieTempC, -128.0f, 127.0f));
  s.rssi         = g_sunRssi;
  s.cfg_ok       = g_cfgOk ? 1 : 0;
  s.sg_load      = (g_sgL < g_sgR) ? g_sgL : g_sgR;   // worst-case (lowest) wheel load
  g_homePulse    = false;

  const uint8_t *dst = g_sunKnown ? g_sunMac : BCAST_MAC;
  esp_now_send(dst, (uint8_t *)&s, sizeof(s));
}

// Scan for the Sun's AP to discover its Wi-Fi channel, then lock onto it. Blocking (~2s);
// only called at boot and while OFFLINE, so it never stalls active operation.
static bool discoverChannel() {
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
  if (ch < 0) return false;
  esp_wifi_set_channel((uint8_t)ch, WIFI_SECOND_CHAN_NONE);
  return true;
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

// A wheel is mechanically held. Stop the cart INSTANTLY (no decel ramp): kill both step trains and
// disable the drivers, report the fault right away, then schedule a fresh spin-up (as if a new RUN
// arrived) as long as the Sun still wants us moving. The status is pushed immediately so the Sun
// registers the stall even though the fault clears again the moment the drivers go idle.
static void handleStall() {
  Serial.printf("STALL detected (sgAvg L=%.0f R=%.0f thr=%u): hard stop + retry\n",
                g_sgAvgL, g_sgAvgR, g_stallThreshold);
  g_error = ERR_MOTOR_STUCK;
  ledcWrite(LEDC_CH_L, 0); ledcWrite(LEDC_CH_R, 0);    // instant: STEP lines low, no deceleration
  g_velCurL = g_velCurR = 0; g_velStartL = g_velStartR = 0;
  g_velFinalL = g_velFinalR = 0; g_actHzL = g_actHzR = 0; g_rampFrac = 1.0f;
  g_resumePending = false; g_stopThenOff = false;
  sendStatus();                                        // make sure the Sun sees ERR_MOTOR_STUCK
  motorsEnable(false);                                 // disable outputs + cut VM
  bool wantMotion = (fabsf(g_cmdWheelL) > 0.01f) || (fabsf(g_cmdWheelR) > 0.01f);
  g_stallRecoverMs = wantMotion ? (millis() + STALL_RESTART_MS) : 0;
}

// ----------------------------- Fault polling --------------------------------
// TMC2209 fault sensing is inherently limited: test_connection() only proves the *driver chip*
// answers over UART (it stays "OK" if you unplug just the motor coils), and open-load /
// StallGuard flags are only meaningful while the coils are actually being driven. So we sample
// them only when the motor is enabled and moving, and require several consecutive bad polls
// (FAULT_DEBOUNCE) before latching a disconnect. Stall uses a running SG average (see below).
static void serviceFaults() {
  if (millis() - g_lastFaultMs < 500) return;
  g_lastFaultMs = millis();

  static uint8_t discCnt = 0;
  if (!g_motorsOn) {                     // drivers asleep (VM may be cut): nothing valid to read
    if (g_error != ERR_MOTOR_STUCK) g_error = ERR_NONE;  // keep a just-set stall until recovery
    g_sgL = g_sgR = 0; g_drvStat = 0; discCnt = 0;
    return;
  }
  bool driving = g_motorsOn && wheelsMoving();

  // Snapshot StallGuard load + open-load/overtemp flags for the heartbeat and tuning.
  g_sgL = drvL.SG_RESULT();
  g_sgR = drvR.SG_RESULT();
  g_drvStat = (driving && drvL.ola() ? 0x01 : 0) | (driving && drvL.olb() ? 0x02 : 0) |
              (driving && drvR.ola() ? 0x04 : 0) | (driving && drvR.olb() ? 0x08 : 0) |
              (drvL.ot() ? 0x10 : 0) | (drvR.ot() ? 0x20 : 0);

  bool commsBad = (drvL.test_connection() != 0) || (drvR.test_connection() != 0);
  bool openLoad = driving && (g_drvStat & 0x0F);       // a coil open while being driven
  bool disc     = commsBad || openLoad;

  // Running StallGuard average per wheel. SG_RESULT is noisy and reads low at low RPM, so we only
  // fold samples into the EMA once the wheel is AT its commanded speed (ramp settled); during the
  // accel ramp the average stays parked at its high init value (no false trips while speeding up).
  bool atSpeed = driving && (g_rampFrac >= 1.0f);
  bool lMoving = fabsf(g_velCurL) > 1.0f;
  bool rMoving = fabsf(g_velCurR) > 1.0f;
  if (atSpeed && lMoving) g_sgAvgL += SG_AVG_ALPHA * ((float)g_sgL - g_sgAvgL);
  if (atSpeed && rMoving) g_sgAvgR += SG_AVG_ALPHA * ((float)g_sgR - g_sgAvgR);

  bool stuck = (g_stallThreshold > 0) && atSpeed && !disc &&
               ((lMoving && g_sgAvgL < (float)g_stallThreshold) ||
                (rMoving && g_sgAvgR < (float)g_stallThreshold));

  discCnt = disc ? (uint8_t)(discCnt + 1) : 0;
  if (discCnt >= FAULT_DEBOUNCE) { g_error = ERR_MOTOR_DISCONNECTED; return; }
  if (stuck)                     { handleStall(); return; }
  g_error = ERR_NONE;                                  // healthy (or still accelerating)
}

// Finish a stall recovery: after the brief post-stop pause, re-enable and ramp back up to the last
// commanded speed — but only while the Sun still wants motion and the cart is still online. (A
// fresh command that arrives during the pause clears g_stallRecoverMs and restarts us itself.)
static void serviceStallRecover() {
  if (!g_stallRecoverMs || (int32_t)(millis() - g_stallRecoverMs) < 0) return;
  g_stallRecoverMs = 0;
  bool wantMotion = (fabsf(g_cmdWheelL) > 0.01f) || (fabsf(g_cmdWheelR) > 0.01f);
  if (!wantMotion || g_state == ST_OFFLINE) return;
  Serial.printf("stall recovery: re-accelerating to L=%.2f R=%.2f rpm\n", g_cmdWheelL, g_cmdWheelR);
  motorsEnable(true);
  wheelSetTargets(g_cmdWheelL, g_cmdWheelR);
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

// ----------------------------- State update ---------------------------------
static void updateState() {
  if (millis() - g_lastCmdMs > 5000) {
    g_state = ST_OFFLINE;
  } else if (g_motorsOn || g_lightOn) {
    g_state = ST_ACTIVE;
  } else {
    g_state = ST_IDLE;
  }
}

// ----------------------------- Setup / loop ----------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t tUsb = millis();                // wait for the USB-CDC host to attach so the
  while (!Serial && millis() - tUsb < 3000) delay(10);   // boot banner isn't lost
  delay(300);
  Serial.println("\n==== Solar System - PLANET boot ====");

  pinMode(PIN_MOTOR_PWR, OUTPUT);  digitalWrite(PIN_MOTOR_PWR, LOW);  // motor supply asleep
  pinMode(PIN_TMC_EN, OUTPUT);     digitalWrite(PIN_TMC_EN, HIGH);   // drivers off
  // LEDC step generation: each STEP pin on a channel of its own timer (ch0/timer0, ch2/timer1),
  // both off the one 80 MHz clock so the two wheels stay frequency-synchronized. stepEmit() later
  // sets the actual step rate with a per-frequency resolution for accuracy; start silent (duty 0).
  ledcSetup(LEDC_CH_L, 1000, 8); ledcAttachPin(PIN_STEP_L, LEDC_CH_L); ledcWrite(LEDC_CH_L, 0);
  ledcSetup(LEDC_CH_R, 1000, 8); ledcAttachPin(PIN_STEP_R, LEDC_CH_R); ledcWrite(LEDC_CH_R, 0);
  pinMode(PIN_LIGHT_EN, OUTPUT);   digitalWrite(PIN_LIGHT_EN, LOW);
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
  serviceStatusLed();
  serviceButton();
  serviceCommand();
  serviceHoming();
  serviceRamp();
  serviceFaults();
  serviceStallRecover();

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
    const char *st = g_state == ST_OFFLINE ? "OFFLINE" : g_state == ST_IDLE ? "IDLE" : "ACTIVE";
    bool home = (digitalRead(PIN_HOME) == LOW);          // active-low sensor
    // Current wheel velocities and the equivalent rpm derived from the ACTUAL emitted LEDC step
    // frequency, so a bench check (scope on STEP) can confirm the motor turns at the commanded
    // speed and that the L:R ratio holds throughout the ramp.
    float fullSteps = 360.0f / (g_stepDegree > 0.01f ? g_stepDegree : 1.8f);
    float rpmL = g_actHzL / (fullSteps * g_microsteps) * 60.0f * (g_velCurL < 0 ? -1 : 1);
    float rpmR = g_actHzR / (fullSteps * g_microsteps) * 60.0f * (g_velCurR < 0 ? -1 : 1);
    Serial.printf("[%lus] %s idx=%u sun=%d | signal=%d%%(%ddBm) temp=%.1fC home=%d | "
                  "TMC:%s I=%umA usteps=%u %s%s sg=%u/%u ol=0x%02X | "
                  "cmd L=%.2f R=%.2f rpm -> now L=%.2f R=%.2f rpm (%.0f/%.0f Hz) | "
                  "light=%d motors=%d\n",
                  (unsigned long)(now / 1000), st, g_myIndex, g_sunKnown ? 1 : 0,
                  rssiPct(g_sunRssi), g_sunRssi, g_dieTempC, home ? 1 : 0,
                  errText(g_error), g_currentMa, g_microsteps,
                  g_spread ? "spread" : "stealth", g_interpolate ? " mplyer" : "",
                  g_sgL, g_sgR, g_drvStat, g_cmdWheelL, g_cmdWheelR,
                  rpmL, rpmR, g_actHzL, g_actHzR,
                  g_lightOn ? 1 : 0, g_motorsOn ? 1 : 0);
  }
}
