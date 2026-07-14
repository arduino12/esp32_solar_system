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
// Motors: 3x TMC2209 on one single-wire UART bus (addresses 0/1/2).
//   - Left wheel  (outer rail track) -> FastAccelStepper step pulses (RMT), dir via UART shaft bit
//   - Right wheel (inner rail track) -> FastAccelStepper step pulses (RMT), dir via UART shaft bit
//   - Planet ball spin               -> TMC internal VACTUAL velocity mode (no MCU pulses)
// DIR pins are tied to GND in hardware; direction is set over UART, so only 2 RMT channels
// (both wheels) are needed — which is exactly what the C3 provides.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include "solar_protocol.h"

// ----------------------------- Pin map (from spec) --------------------------
static const uint8_t PIN_TMC_EN     = 2;   // TMC enable, active-low (LOW = drivers on)
static const uint8_t PIN_TMC_RX     = 4;   // Serial1 RX  <- driver PDN/UART bus
static const uint8_t PIN_TMC_TX     = 3;   // Serial1 TX  -> driver PDN/UART bus (via 1k)
static const uint8_t PIN_STEP_L     = 0;   // left  wheel step
static const uint8_t PIN_STEP_R     = 1;   // right wheel step
static const uint8_t PIN_STEP_P     = 5;   // planet-spin step (only used in LEDC fallback)
static const uint8_t PIN_HOME       = 6;   // homing sensor, active-low, FALLING pulse/rev
static const uint8_t PIN_LIGHT_EN   = 7;   // N-MOS gate for planet light
static const uint8_t PIN_STATUS_LED = 8;   // onboard LED, active-low
static const uint8_t PIN_BUTTON     = 9;   // onboard boot button, active-low

// ----------------------------- Motor / TMC config ---------------------------
#define USE_VACTUAL_SPIN   1               // 1: spin ball via TMC VACTUAL; 0: LEDC PWM on PIN_STEP_P
#define TMC_R_SENSE        0.11f
#define TMC_UART_BAUD      115200
#define TMC_ADDR_L         0b11            // MS1=VIO MS2=VIO
#define TMC_ADDR_R         0b10            // MS1=GND MS2=VIO
#define TMC_ADDR_P         0b00            // MS1=GND MS2=GND
#define MOTOR_FULL_STEPS   200             // 1.8 deg/step
#define WHEEL_ACCEL_RPM_S  300.0f          // wheel acceleration ramp (rpm per second)
#define LEDC_SPIN_CH       0               // LEDC channel for spin fallback
#define LEDC_SPIN_RES      8               // LEDC resolution bits

// ----------------------------- Radio / power --------------------------------
// The C3 runs warm because its Wi-Fi radio is always on for ESP-NOW. Two levers reduce it:
//  - Lower the max TX power: 44 = 11 dBm (vs the 84/21 dBm default). Ample for a room-scale
//    installation and noticeably cooler / lower current. Raise if range is a problem.
//  - Rescan for the Sun's channel less often while OFFLINE (active scans are the biggest
//    periodic radio burst on an otherwise idle cart).
#define WIFI_TX_POWER_QDBM 44
#define OFFLINE_RESCAN_MS  10000

// Boot defaults (overwritten by the first command from the Sun).
static uint16_t g_currentMa  = 1000;       // current_amp=1.0
static uint8_t  g_microsteps = 16;

HardwareSerial   TMC_SERIAL(1);            // ESP32-C3 UART1
TMC2209Stepper   drvL(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_L);
TMC2209Stepper   drvR(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_R);
TMC2209Stepper   drvP(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_P);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepL = nullptr;
FastAccelStepper *stepR = nullptr;

// ----------------------------- Runtime state --------------------------------
enum PlanetState : uint8_t { ST_OFFLINE, ST_IDLE, ST_ACTIVE };
static PlanetState g_state = ST_OFFLINE;

static uint8_t  g_myIndex     = PLANET_INDEX_UNKNOWN;
static bool     g_motorsOn    = false;     // wheels/spin enabled
static bool     g_lightOn     = false;
static bool     g_homingStop  = false;     // true while executing CMD_HOME_AND_STOP
static uint8_t  g_error       = ERR_NONE;

// Last commanded motion (for status/debug; the cart is a dumb executor of the Sun's numbers).
static float    g_cmdWheelL   = 0;
static float    g_cmdWheelR   = 0;
static float    g_cmdSpin     = 0;

// Diagnostics reported back to the Sun and printed in the heartbeat.
static int8_t   g_sunRssi     = 0;         // Sun-AP RSSI (dBm) from the last channel scan
static float    g_dieTempC    = 0;         // ESP32-C3 internal temperature (°C)

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

static inline uint32_t rpmToStepsHz(float rpm) {
  // |rpm| -> microsteps per second
  return (uint32_t)((fabsf(rpm) / 60.0f) * (float)MOTOR_FULL_STEPS * (float)g_microsteps);
}

static void applyDriverParams() {
  // Re-apply current/microsteps to all three drivers (called at boot and on every command
  // that changes them). Mirrors the bring-up sequence used in rotating_rods_skr_pico.
  TMC2209Stepper *all[3] = {&drvL, &drvR, &drvP};
  for (auto *d : all) {
    d->rms_current(g_currentMa);
    d->microsteps(g_microsteps);
    d->intpol(g_microsteps >= 8);
    d->en_spreadCycle(g_microsteps < 8);   // stealthChop at >=8, spreadCycle below
  }
}

static void tmcBringup() {
  TMC_SERIAL.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  TMC2209Stepper *all[3] = {&drvL, &drvR, &drvP};
  for (auto *d : all) {
    d->begin();
    d->toff(4);
    d->pdn_disable(true);       // required so UART (not the PDN pin) controls the driver
    d->I_scale_analog(false);   // internal Vref (ignore VREF pin)
    d->mstep_reg_select(true);  // microsteps set over UART, not MS1/MS2 pins
  }
  applyDriverParams();
}

static void motorsEnable(bool on) {
  digitalWrite(PIN_TMC_EN, on ? LOW : HIGH);  // active-low
  g_motorsOn = on;
}

static void lightSet(bool on) {
  digitalWrite(PIN_LIGHT_EN, on ? HIGH : LOW);
  g_lightOn = on;
}

// Continuous wheel rotation at a signed rpm. Direction handled via the TMC shaft bit
// because the DIR pins are tied to GND; FastAccelStepper only produces step pulses.
static void wheelRun(FastAccelStepper *st, TMC2209Stepper &drv, float rpm) {
  if (!st) return;
  if (fabsf(rpm) < 0.01f) { st->stopMove(); return; }
  drv.shaft(rpm < 0);
  uint32_t hz    = rpmToStepsHz(rpm);
  uint32_t accel = rpmToStepsHz(WHEEL_ACCEL_RPM_S);
  st->setSpeedInHz(hz > 0 ? hz : 1);
  st->setAcceleration(accel > 0 ? accel : 1);
  st->runForward();
}

static void wheelsStop() {
  if (stepL) stepL->stopMove();
  if (stepR) stepR->stopMove();
}

// Planet-ball spin at a signed rpm using the TMC internal velocity generator (VACTUAL),
// or LEDC PWM step clock as a fallback. VACTUAL[Hz] = VACTUAL_reg * fclk / 2^24, fclk~12MHz.
static void spinSet(float rpm) {
#if USE_VACTUAL_SPIN
  int32_t microPerSec = (int32_t)((rpm / 60.0f) * (float)MOTOR_FULL_STEPS * (float)g_microsteps);
  int32_t vactual = (int32_t)((float)microPerSec / 0.715f);   // 12e6 / 2^24
  drvP.VACTUAL(vactual);
#else
  uint32_t hz = rpmToStepsHz(rpm);
  if (hz == 0) { ledcWrite(LEDC_SPIN_CH, 0); return; }
  ledcSetup(LEDC_SPIN_CH, hz, LEDC_SPIN_RES);
  ledcAttachPin(PIN_STEP_P, LEDC_SPIN_CH);
  ledcWrite(LEDC_SPIN_CH, 1 << (LEDC_SPIN_RES - 1));          // 50% duty
#endif
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

static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 2 || data[0] != SOLAR_MAGIC || data[1] != SOLAR_MSG_CMD) return;
  if (len < (int)sizeof(CmdMsg)) return;
  CmdMsg m;
  memcpy(&m, data, sizeof(m));

  g_lastCmdMs = millis();
  g_myIndex   = m.planet_index;

  if (!g_sunKnown) {              // learn the Sun and switch to unicast replies
    memcpy(g_sunMac, mac, 6);
    g_sunKnown = true;
    addPeer(g_sunMac);
  }

  // Apply driver parameters if they changed.
  if (m.current_ma != g_currentMa || m.microsteps != g_microsteps) {
    g_currentMa  = m.current_ma;
    g_microsteps = m.microsteps;
    applyDriverParams();
  }

  switch (m.command) {
    case CMD_STOP:
      g_homingStop = false;
      g_cmdWheelL = 0; g_cmdWheelR = 0; g_cmdSpin = 0;
      wheelsStop(); spinSet(0);
      motorsEnable(false);
      lightSet(false);
      break;

    case CMD_RUN:
      g_homingStop = false;
      g_cmdWheelL = m.wheel_l_rpm; g_cmdWheelR = m.wheel_r_rpm; g_cmdSpin = m.spin_rpm;
      motorsEnable(true);
      wheelRun(stepL, drvL, m.wheel_l_rpm);
      wheelRun(stepR, drvR, m.wheel_r_rpm);
      spinSet(m.spin_rpm);
      lightSet(m.light_on);
      break;

    case CMD_HOME_AND_STOP:
      // Drive both wheels slowly toward HOME; homing logic stops on the pulse.
      g_homingStop = true;
      g_homeSeen   = false;
      g_cmdWheelL = m.wheel_l_rpm; g_cmdWheelR = m.wheel_r_rpm; g_cmdSpin = 0;
      motorsEnable(true);
      wheelRun(stepL, drvL, m.wheel_l_rpm);
      wheelRun(stepR, drvR, m.wheel_r_rpm);
      spinSet(0);
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
  s.moving       = (stepL && stepL->isRunning()) || (stepR && stepR->isRunning());
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
static void serviceFaults() {
  if (millis() - g_lastFaultMs < 500) return;
  g_lastFaultMs = millis();

  uint8_t err = ERR_NONE;
  // A failed UART round-trip => driver/harness disconnected.
  if (drvL.test_connection() != 0 || drvR.test_connection() != 0) {
    err = ERR_MOTOR_DISCONNECTED;
  } else if (g_motorsOn) {
    // Open-load flags in DRV_STATUS indicate an unplugged/broken coil.
    if (drvL.ola() || drvL.olb() || drvR.ola() || drvR.olb()) err = ERR_MOTOR_DISCONNECTED;
    // Stall while commanded to move (StallGuard result near zero) => stuck. Tunable.
    else if (((stepL && stepL->isRunning()) || (stepR && stepR->isRunning())) &&
             (drvL.SG_RESULT() == 0 && drvR.SG_RESULT() == 0)) {
      err = ERR_MOTOR_STUCK;
    }
  }
  g_error = err;
}

// ----------------------------- Homing ---------------------------------------
static void serviceHoming() {
  if (g_homingStop && g_homeSeen) {
    g_homingStop = false;
    wheelsStop();
    // Let the ramp-down finish, then cut power to hold position passively.
    motorsEnable(false);
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
    motorsEnable(true);
    spinSet(60);
    lightSet(true);
  }
  if (g_selfTest && millis() > g_selfTestEndMs) {
    g_selfTest = false;
    spinSet(0);
    motorsEnable(false);
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

  pinMode(PIN_TMC_EN, OUTPUT);     digitalWrite(PIN_TMC_EN, HIGH);   // drivers off
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

  engine.init();
  stepL = engine.stepperConnectToPin(PIN_STEP_L);
  stepR = engine.stepperConnectToPin(PIN_STEP_R);
  if (stepL) stepL->setAutoEnable(false);
  if (stepR) stepR->setAutoEnable(false);

  if (!discoverChannel()) {
    Serial.println("Sun AP not found yet; will retry while OFFLINE.");
  }
  if (!espnowStart()) {
    Serial.println("ESP-NOW init failed!");
  }
  g_nextStatusMs = millis() + 1000;
}

void loop() {
  uint32_t now = millis();

  updateState();
  serviceStatusLed();
  serviceButton();
  serviceHoming();
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
    Serial.printf("[%lus] %s idx=%u sun=%d | signal=%d%%(%ddBm) temp=%.1fC home=%s | "
                  "TMC:%s I=%umA usteps=%u | cmd L=%.0f R=%.0f spin=%.0f rpm | light=%d motors=%d\n",
                  (unsigned long)(now / 1000), st, g_myIndex, g_sunKnown ? 1 : 0,
                  rssiPct(g_sunRssi), g_sunRssi, g_dieTempC, home ? "ON" : "off",
                  errText(g_error), g_currentMa, g_microsteps,
                  g_cmdWheelL, g_cmdWheelR, g_cmdSpin,
                  g_lightOn ? 1 : 0, g_motorsOn ? 1 : 0);
  }
}
