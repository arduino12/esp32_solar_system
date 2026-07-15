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
// Wheel drive uses the TMC2209's built-in constant-velocity generator (VACTUAL register) over
// UART, NOT MCU-generated STEP pulses. On the single-core ESP32-C3, FastAccelStepper's RMT step
// ISR is starved by the always-on Wi-Fi/ESP-NOW radio, which intermittently wedges the step
// generator on restart (motor freezes mid-ramp until reboot). VACTUAL sidesteps that entirely:
// the driver spins the motor from its own internal clock, so start/stop/restart is just a
// register write and there is no 200 kHz step-rate ceiling (any microstep setting works). We
// ramp VACTUAL up/down in software (serviceRamp) to get smooth ~1 s accel/decel.

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
static const uint8_t PIN_STEP_L     = 0;   // (unused with VACTUAL; STEP is generated inside TMC)
static const uint8_t PIN_STEP_R     = 1;   // (unused with VACTUAL)
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
// TMC2209 internal oscillator (datasheet typical 12 MHz). VACTUAL speed:
//   v[µsteps/s] = VACTUAL * fCLK / 2^24  ->  VACTUAL = v * 2^24 / fCLK.
// If measured RPM is a few % off, trim TMC_FCLK (the internal clock varies part to part).
#define TMC_FCLK           12000000.0f
#define VACTUAL_PER_HZ     (16777216.0f / TMC_FCLK)   // µsteps/s -> VACTUAL register units
#define WHEEL_RAMP_S       1.0f            // seconds to accelerate 0->commanded speed (and back)
#define VACTUAL_UPDATE_MS  20              // how often the software ramp pushes VACTUAL over UART
// StallGuard load below this while a wheel is commanded to move = mechanically stuck.
// TMC2209 StallGuard needs per-rig tuning; watch sgL/sgR in the serial heartbeat and set this
// a little above the "held" reading but below the free-running reading. 0 disables the check.
#define STALL_SG_MIN       0
#define FAULT_DEBOUNCE     3               // consecutive bad polls before a fault is latched

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
static uint8_t  g_microsteps  = 16;
static float    g_stepDegree  = 1.8f;      // motor step angle -> full steps/rev = 360/step_degree

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

// Wheel velocity ramp (µsteps/s, signed = direction). serviceRamp() moves cur toward tgt at
// g_rampRate and pushes the matching VACTUAL to each driver.
static float    g_cmdWheelL   = 0;         // last commanded wheel rpm (for status/debug)
static float    g_cmdWheelR   = 0;
static float    g_velCurL = 0, g_velCurR = 0;   // current wheel velocity, µsteps/s
static float    g_velTgtL = 0, g_velTgtR = 0;   // target  wheel velocity, µsteps/s
static float    g_rampRate    = 1000.0f;   // µsteps/s^2, recomputed whenever a target is set
static uint32_t g_rampMs      = 0;         // last ramp tick

// Deferred actions triggered when the wheels have ramped to a stop.
static bool     g_stopThenOff   = false;   // ramp to 0, then disable the drivers (CMD_STOP/homing)
static bool     g_resumePending = false;   // ramp to 0, change microsteps, then ramp back up
static float    g_pendingRpmL = 0, g_pendingRpmR = 0;   // rpm to resume with after a microstep change
static uint8_t  g_pendingMicro  = 16;
static uint16_t g_pendingCurrent = 200;

// Diagnostics reported back to the Sun and printed in the heartbeat.
static int8_t   g_sunRssi     = 0;         // Sun-AP RSSI (dBm) from the last channel scan
static float    g_dieTempC    = 0;         // ESP32-C3 internal temperature (°C)
static uint16_t g_sgL = 0, g_sgR = 0;      // last StallGuard load readings (for tuning)
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

// Round an arbitrary microstep request to the nearest valid TMC2209 value (power of two, 1..256).
static uint8_t sanitizeMicrosteps(uint8_t m) {
  const uint8_t valid[] = {1, 2, 4, 8, 16, 32, 64, 128, 255};  // 255 => 256 per TMCStepper
  uint8_t best = 16; uint16_t bestErr = 0xFFFF;
  for (uint8_t v : valid) {
    uint16_t e = (v > m) ? (v - m) : (m - v);
    if (e < bestErr) { bestErr = e; best = v; }
  }
  return best;
}

static void applyDriverParams() {
  // Re-apply current/microsteps to both wheel drivers (called at boot and whenever they change).
  // Low-heat standstill: hold current 0 + freewheel so a stopped-but-enabled motor dissipates
  // almost nothing (planets never need to hold position precisely).
  TMC2209Stepper *all[2] = {&drvL, &drvR};
  for (auto *d : all) {
    d->rms_current(g_currentMa, 0.0f);     // IRUN from mA; IHOLD = 0 (see freewheel below)
    d->microsteps(g_microsteps);
    d->intpol(g_microsteps >= 8);
    d->en_spreadCycle(g_microsteps < 8);   // stealthChop at >=8, spreadCycle below
    d->TPOWERDOWN(20);                      // ~0.4 s after stop, drop to standstill state
    d->freewheel(1);                        // IHOLD=0 -> coils freewheel at standstill (no heat)
  }
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
    d->VACTUAL(0);              // ensure the internal velocity generator starts stopped
  }
  applyDriverParams();
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
  } else {
    drvL.VACTUAL(0); drvR.VACTUAL(0);           // stop the internal velocity generators
    g_velCurL = g_velCurR = 0; g_velTgtL = g_velTgtR = 0;
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

// Set new wheel velocity targets (µsteps/s, signed) and pick a ramp rate that reaches them in
// ~WHEEL_RAMP_S seconds. The right wheel is inverted for the back-to-back mount.
static void wheelSetTargets(float lHz, float rHz) {
  g_velTgtL = lHz;
  g_velTgtR = rHz * (WHEEL_R_INVERT ? -1.0f : 1.0f);
  float span = fmaxf(fmaxf(fabsf(g_velTgtL), fabsf(g_velTgtR)),
                     fmaxf(fabsf(g_velCurL), fabsf(g_velCurR)));
  g_rampRate = fmaxf(span / WHEEL_RAMP_S, 100.0f);
}

static inline void approach(float &cur, float tgt, float step) {
  if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
  else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
}

// Software velocity ramp: nudge current velocity toward target and push VACTUAL to the drivers.
// When the wheels reach a stop, run any deferred action (power-off, or microsteps-change resume).
static void serviceRamp() {
  uint32_t now = millis();
  if (now - g_rampMs < VACTUAL_UPDATE_MS) return;
  float dt = (now - g_rampMs) / 1000.0f;
  g_rampMs = now;

  float step = g_rampRate * dt;
  approach(g_velCurL, g_velTgtL, step);
  approach(g_velCurR, g_velTgtR, step);

  if (g_motorsOn) {
    drvL.VACTUAL((uint32_t)(int32_t)lroundf(g_velCurL * VACTUAL_PER_HZ));
    drvR.VACTUAL((uint32_t)(int32_t)lroundf(g_velCurR * VACTUAL_PER_HZ));
  }

  bool stopped = (fabsf(g_velCurL) < 1.0f) && (fabsf(g_velCurR) < 1.0f);
  if (!stopped) return;

  if (g_resumePending) {                  // microsteps change: apply at standstill, then resume
    g_resumePending = false;
    g_currentMa  = g_pendingCurrent;
    g_microsteps = g_pendingMicro;
    applyDriverParams();
    Serial.printf("microsteps applied=%u at standstill; re-accelerating to L=%.0f R=%.0f rpm\n",
                  g_microsteps, g_pendingRpmL, g_pendingRpmR);
    g_cmdWheelL = g_pendingRpmL; g_cmdWheelR = g_pendingRpmR;
    wheelSetTargets(rpmToUstepsHz(g_pendingRpmL), rpmToUstepsHz(g_pendingRpmR));
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

  uint8_t effUs = sanitizeMicrosteps(m.microsteps);
  bool microChanged  = (effUs != g_microsteps);
  bool currentChanged = (m.current_ma != g_currentMa);

  switch (m.command) {
    case CMD_STOP:
      g_homingStop = false; g_resumePending = false;
      g_cmdWheelL = 0; g_cmdWheelR = 0;
      if (g_motorsOn) { g_stopThenOff = true; wheelSetTargets(0, 0); }  // decelerate, then power off
      lightSet(false);
      break;

    case CMD_RUN:
      g_homingStop = false; g_stopThenOff = false;
      // Changing microsteps while turning would jump the speed; decelerate to a stop, change it at
      // standstill, then re-accelerate (serviceRamp completes the sequence). Safe & reliable with
      // VACTUAL — no step-generator restart involved.
      if (microChanged && wheelsMoving()) {
        Serial.printf("microsteps %u->%u while running: STOP, apply, then re-accelerate\n",
                      g_microsteps, effUs);
        g_resumePending = true;
        g_pendingMicro = effUs; g_pendingCurrent = m.current_ma;
        g_pendingRpmL = m.wheel_l_rpm; g_pendingRpmR = m.wheel_r_rpm;
        wheelSetTargets(0, 0);
      } else if (!g_resumePending) {
        if (currentChanged || microChanged) {
          g_currentMa = m.current_ma; g_microsteps = effUs; applyDriverParams();
        }
        if (!g_motorsOn) motorsEnable(true);
        g_cmdWheelL = m.wheel_l_rpm; g_cmdWheelR = m.wheel_r_rpm;
        wheelSetTargets(rpmToUstepsHz(m.wheel_l_rpm), rpmToUstepsHz(m.wheel_r_rpm));
      }
      lightSet(m.light_on);
      break;

    case CMD_HOME_AND_STOP:           // drive slowly toward HOME; homing logic stops on the pulse
      g_resumePending = false; g_stopThenOff = false;
      if (currentChanged || microChanged) {
        g_currentMa = m.current_ma; g_microsteps = effUs; applyDriverParams();
      }
      if (!g_homingStop) g_homeSeen = false;    // clear on entry only, not every heartbeat
      g_homingStop = true;
      if (!g_motorsOn) motorsEnable(true);
      g_cmdWheelL = m.wheel_l_rpm; g_cmdWheelR = m.wheel_r_rpm;
      wheelSetTargets(rpmToUstepsHz(m.wheel_l_rpm), rpmToUstepsHz(m.wheel_r_rpm));
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

// ----------------------------- Fault polling --------------------------------
// TMC2209 fault sensing is inherently limited: test_connection() only proves the *driver chip*
// answers over UART (it stays "OK" if you unplug just the motor coils), and open-load /
// StallGuard flags are only meaningful while the coils are actually being driven. So we sample
// them only when the motor is enabled and moving, and require several consecutive bad polls
// (FAULT_DEBOUNCE) before latching a fault, to avoid the flags' well-known false positives.
static void serviceFaults() {
  if (millis() - g_lastFaultMs < 500) return;
  g_lastFaultMs = millis();

  static uint8_t discCnt = 0, stuckCnt = 0;
  if (!g_motorsOn) {                     // drivers asleep (VM may be cut): nothing valid to read
    g_error = ERR_NONE; g_sgL = g_sgR = 0; g_drvStat = 0; discCnt = stuckCnt = 0;
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

  // Stuck: commanded to move but StallGuard load collapses (rotor physically held).
  // Off by default (STALL_SG_MIN==0) until tuned against the real rig — see the #define.
  bool stuck = (STALL_SG_MIN > 0) && driving && !disc &&
               ((g_sgL < STALL_SG_MIN) || (g_sgR < STALL_SG_MIN));

  discCnt  = disc  ? (uint8_t)(discCnt  + 1) : 0;
  stuckCnt = stuck ? (uint8_t)(stuckCnt + 1) : 0;

  if      (discCnt  >= FAULT_DEBOUNCE) g_error = ERR_MOTOR_DISCONNECTED;
  else if (stuckCnt >= FAULT_DEBOUNCE) g_error = ERR_MOTOR_STUCK;
  else if (!disc && !stuck)            g_error = ERR_NONE;
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
    wheelSetTargets(rpmToUstepsHz(60), rpmToUstepsHz(60));
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
  pinMode(PIN_STEP_L, OUTPUT);     digitalWrite(PIN_STEP_L, LOW);    // unused with VACTUAL
  pinMode(PIN_STEP_R, OUTPUT);     digitalWrite(PIN_STEP_R, LOW);
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
    // Current wheel velocities in µsteps/s and the equivalent rpm, so a bench check can confirm
    // the motor is actually turning at the commanded speed (VACTUAL, no external step pin).
    float fullSteps = 360.0f / (g_stepDegree > 0.01f ? g_stepDegree : 1.8f);
    float rpmL = g_velCurL / (fullSteps * g_microsteps) * 60.0f;
    float rpmR = g_velCurR / (fullSteps * g_microsteps) * 60.0f;
    Serial.printf("[%lus] %s idx=%u sun=%d | signal=%d%%(%ddBm) temp=%.1fC home=%s | "
                  "TMC:%s I=%umA usteps=%u %.1fdeg sg=%u/%u ol=0x%02X | "
                  "cmd L=%.0f R=%.0f rpm -> now L=%.0f R=%.0f rpm (%.0f/%.0f ustep/s) | "
                  "light=%d motors=%d\n",
                  (unsigned long)(now / 1000), st, g_myIndex, g_sunKnown ? 1 : 0,
                  rssiPct(g_sunRssi), g_sunRssi, g_dieTempC, home ? "ON" : "off",
                  errText(g_error), g_currentMa, g_microsteps, g_stepDegree,
                  g_sgL, g_sgR, g_drvStat, g_cmdWheelL, g_cmdWheelR,
                  rpmL, rpmR, g_velCurL, g_velCurR,
                  g_lightOn ? 1 : 0, g_motorsOn ? 1 : 0);
  }
}
