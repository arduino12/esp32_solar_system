// sun_main.cpp — Solar System SUN controller firmware (ESP32 WROOM-32).
// env: solar_sun_esp32
//
// The Sun is the master: it hosts a Wi-Fi web page (AP + optional home-STA), drives a
// WS2812 strip with a fire effect, runs the show state machine, and bridges the web/button
// controls to the 8 planet carts over ESP-NOW. Settings and statistics live in LittleFS.
//
// Web/Wi-Fi/JSON structure follows the proven pattern from esp32s3_pov_cylinder
// (ESPAsyncWebServer + ArduinoJson v7 + a PROGMEM single-page UI + /api/* JSON).

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ElegantOTA.h>
#include <MD5Builder.h>

#define FASTLED_INTERNAL
#include <FastLED.h>

#include "solar_protocol.h"
#include "sun_web_ui.h"

// ----------------------------- Pin map (from spec) --------------------------
static const uint8_t PIN_LED         = 14;  // WS2812 data
static const uint8_t PIN_BTN_START   = 25;  // Start button, active-low
static const uint8_t PIN_BTN_BOOT    = 0;   // onboard boot button, active-low (5s hold = factory reset)

// ----------------------------- LED config -----------------------------------
#define NUM_LEDS        45
#define LED_TYPE        WS2812B
#define LED_ORDER       GRB
static CRGB      g_leds[NUM_LEDS];
static uint8_t   g_heat[NUM_LEDS];
static uint32_t  g_lastFireMs = 0;

// ----------------------------- Timing constants ------------------------------
#define HEARTBEAT_MS    2000        // resend current command set to planets
#define HOMING_BASE_RPM 40.0f       // wheel reference speed while homing for menorah
#define ORBIT_SIGN      1.0f        // wheel-speed sign that orbits counter-clockwise (all planets);
                                    // -1.0f if the carts turn out to go clockwise
#define STA_RETRY_MS    60000
#define FACTORY_HOLD_MS 5000
#define LOG_N           12
#define MDNS_HOST       "solar-system"     // reachable as solar-system.local

// ----------------------------- Persistent state ------------------------------
static JsonDocument g_settings;            // full settings.json in RAM
static uint32_t g_pressesTriggered = 0;
static uint32_t g_pressesIgnored   = 0;

// Default settings written on first boot / factory reset (matches the plan).
static const char DEFAULT_SETTINGS_JSON[] PROGMEM = R"JSON({
  "wifi": { "ap_ssid": "SolarSystem", "ap_pass": "didivardi", "sta_ssid": "", "sta_pass": "" },
  "modes": {
    "solar":   { "runtime_minutes": 5, "cooldown_minutes": 1 },
    "test":    { "runtime_minutes": 30 },
    "menorah": { "active_candles": 0 }
  },
  "planets": {
    "mac_address": ["","","","","","","",""],
    "light_brightness_per": [100,100,100,100,100,100,100,100],
    "auto_update_firmware": true,
    "wheels": {
      "diameter_mm": 36,
      "reverse_per": ["none","none","none","none","none","none","none","none"],
      "stepper": { "current_amp": 0.2, "microsteps": 16, "step_degree": 1.8,
                   "chopper_mode": "stealth", "microplyer": true,
                   "max_speed_rpm": [500,500,500,500,500,500,500,500] }
    },
    "rail": {
      "width_mm": 60,
      "outer_diameter_mm": [640,740,840,940,1040,1140,1240,1340],
      "log_scale_speed_rpm": [300,270,200,120,104,62,38,22]
    }
  },
  "sun_led": {
    "solar":   { "brightness_per": 100, "effect": "fire01", "effect_speed": 5 },
    "menorah": { "brightness_per": 70,  "effect": "fire02", "effect_speed": 3 }
  }
})JSON";

// ----------------------------- Runtime state ---------------------------------
static SolarMode g_mode = MODE_IDLE;
static uint32_t  g_phaseStartMs = 0;       // for count-up (IDLE/MENORAH)
static uint32_t  g_phaseEndMs   = 0;       // for count-down (SOLAR/COOLDOWN/TEST); 0 if none
static uint32_t  g_lastHeartbeatMs = 0;

// Per-planet MACs (parsed) and last-known status.
static uint8_t g_planetMac[SOLAR_PLANET_COUNT][6];
static bool    g_macValid[SOLAR_PLANET_COUNT];

struct PlanetState {
  volatile uint32_t lastSeenMs;   // set by the ESP-NOW callback: compare with msSince()
  uint8_t  error;
  bool     lightOn;
  bool     moving;
  uint32_t homeCount;     // increments each reported HOME pulse
  float    orbitalRpm;    // signed; Sun-computed for the web animation
  int8_t   tempC;         // planet die temperature, °C (reported by the cart)
  int8_t   rssi;          // planet<->Sun signal, dBm (reported by the cart)
  bool     cfgOk;         // planet verified its last driver-param apply over UART
  uint32_t fwStamp;       // running cart build, YYMMDDhhmm
  uint8_t  otaState;      // OtaState reported by the cart
  uint8_t  otaProgress;   // 0..100 % while downloading
  int16_t  otaErr;        // reason for OTA_FAILED
  uint32_t txOk, txFail;  // ESP-NOW unicasts to this cart that were / weren't ACKed by its radio
  uint8_t  txFailRun;     // consecutive unACKed unicasts
};
static PlanetState g_planet[SOLAR_PLANET_COUNT];

// Cart firmware image, LittleFS CART_FW_PATH (see "Cart firmware update" below), and the chunk
// requests the ESP-NOW callback queues for loop() to answer.
#define CART_FW_PATH "/cart.bin"
struct CartFw { bool present; uint32_t size; uint32_t stamp; uint8_t md5[16]; };
static CartFw g_fw = {};
static SemaphoreHandle_t g_fwLock = nullptr;   // an upload replacing the file vs loop() reading it
#define FW_REQQ 16
struct FwReq { uint8_t planet; uint32_t stamp; uint32_t offset; uint8_t count; };
static FwReq  g_fwReqq[FW_REQQ];
static volatile uint8_t g_fwReqHead = 0, g_fwReqTail = 0;

// Deferred work flags (set from async/ISR context, handled in loop()).
static volatile bool g_reloadSettings = false;
static volatile bool g_doFactoryReset = false;
static String        g_pendingSettings;
static String        g_pendingAction;

// Event/error log ring buffer for the web UI.
static String g_log[LOG_N];
static int    g_logHead = 0;

AsyncWebServer server(80);
static bool g_staOk = false;
static uint32_t g_lastStaTryMs = 0;
// STA link events, counted in the Wi-Fi event task and logged from loop(). A STA reconnect scans
// every channel, which takes the AP and ESP-NOW off the planets' channel for seconds.
static volatile uint32_t g_staDrops = 0, g_staUps = 0;
static volatile uint8_t  g_staDropReason = 0;
static uint32_t g_staDropsLogged = 0, g_staUpsLogged = 0;

// ----------------------------- Small helpers ---------------------------------
static String fmtUptime() {
  uint32_t s = millis() / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u",
           (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
  return String(buf);
}

// Milliseconds since t, for timestamps the ESP-NOW callback writes (PlanetState.lastSeenMs): it can
// land between our millis() and our read of t, so a "future" t counts as 0 instead of the unsigned
// difference wrapping to ~4e9 (which would flag an online cart as gone).
static inline uint32_t msSince(uint32_t t) {
  int32_t d = (int32_t)(millis() - t);
  return d > 0 ? (uint32_t)d : 0;
}

static void addLog(const String &s) {
  g_log[g_logHead] = fmtUptime() + "  " + s;
  g_logHead = (g_logHead + 1) % LOG_N;
}

static uint8_t hex2u8(const char *p) {
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  int hi = nib(p[0]), lo = nib(p[1]);
  if (hi < 0 || lo < 0) return 0;
  return (uint8_t)((hi << 4) | lo);
}

// Parse "AA:BB:CC:DD:EE:FF" -> 6 bytes. Returns false if malformed/empty.
static bool parseMac(const char *s, uint8_t out[6]) {
  if (!s || strlen(s) < 17) return false;
  for (int i = 0; i < 6; i++) {
    out[i] = hex2u8(s + i * 3);
    if (i < 5 && s[i * 3 + 2] != ':') return false;
  }
  return true;
}

// ----------------------------- LittleFS config -------------------------------
static void writeDefaults() {
  File f = LittleFS.open("/settings.json", "w");
  if (f) { f.print(FPSTR(DEFAULT_SETTINGS_JSON)); f.close(); }
}

static void loadSettings() {
  File f = LittleFS.open("/settings.json", "r");
  if (!f) { writeDefaults(); f = LittleFS.open("/settings.json", "r"); }
  if (!f) return;
  DeserializationError e = deserializeJson(g_settings, f);
  f.close();
  if (e) {                                  // corrupt file -> restore defaults
    writeDefaults();
    f = LittleFS.open("/settings.json", "r");
    if (f) { deserializeJson(g_settings, f); f.close(); }
  }
}

static void saveSettings() {
  File f = LittleFS.open("/settings.json", "w");
  if (!f) return;
  serializeJson(g_settings, f);
  f.close();
}

// Drop keys belonging to removed features so an old saved file doesn't keep resurfacing them in
// the web form. Runs once at boot and rewrites the file only if something changed.
static void migrateSettings() {
  bool changed = false;
  JsonObject planets = g_settings["planets"].as<JsonObject>();
  if (!planets.isNull() && !planets["axial_rotation_speed_rpm"].isNull()) {
    planets.remove("axial_rotation_speed_rpm");
    changed = true;
    Serial.println("Migrated settings: removed obsolete axial_rotation_speed_rpm");
  }
  // inner_diameter_mm -> outer_diameter_mm: the model now takes the OUTER rail diameter, which is
  // larger than the old inner value by 2*width_mm. Convert in place so an old file keeps its rig.
  JsonObject rail = g_settings["planets"]["rail"].as<JsonObject>();
  if (!rail.isNull() && !rail["inner_diameter_mm"].isNull() && rail["outer_diameter_mm"].isNull()) {
    float w = rail["width_mm"] | 70.0f;
    float vals[SOLAR_PLANET_COUNT]; int n = 0;    // copy out first (adding a key may move the pool)
    for (JsonVariant v : rail["inner_diameter_mm"].as<JsonArray>())
      if (n < SOLAR_PLANET_COUNT) vals[n++] = v.as<float>() + 2.0f * w;
    rail.remove("inner_diameter_mm");
    JsonArray out = rail["outer_diameter_mm"].to<JsonArray>();
    for (int k = 0; k < n; k++) out.add(vals[k]);
    changed = true;
    Serial.println("Migrated settings: inner_diameter_mm -> outer_diameter_mm (+2*width)");
  }
  // stall_threshold: the motor stuck/disconnected sensing was dropped, so remove the dead setting.
  JsonObject st = g_settings["planets"]["wheels"]["stepper"].as<JsonObject>();
  if (!st.isNull() && !st["stall_threshold"].isNull()) {
    st.remove("stall_threshold");
    changed = true;
    Serial.println("Migrated settings: removed obsolete stall_threshold");
  }
  // colors -> light_brightness_per: the cart light is now one PWM-dimmed LED (N-MOS), not a WS2812.
  if (!planets.isNull() && !planets["colors"].isNull()) {
    planets.remove("colors");
    changed = true;
    Serial.println("Migrated settings: removed obsolete planets.colors");
  }
  if (!planets.isNull() && planets["light_brightness_per"].isNull()) {
    JsonArray br = planets["light_brightness_per"].to<JsonArray>();
    for (int k = 0; k < SOLAR_PLANET_COUNT; k++) br.add(100);
    changed = true;
    Serial.println("Migrated settings: added planets.light_brightness_per");
  }
  if (!planets.isNull() && planets["auto_update_firmware"].isNull()) {
    planets["auto_update_firmware"] = true;     // carts fetch the Sun's cart image by themselves
    changed = true;
    Serial.println("Migrated settings: added planets.auto_update_firmware");
  }
  JsonObject wheels = planets["wheels"].as<JsonObject>();
  if (!wheels.isNull() && wheels["reverse_per"].isNull()) {
    JsonArray rv = wheels["reverse_per"].to<JsonArray>();   // per-cart wheel reversal (was a build
    for (int k = 0; k < SOLAR_PLANET_COUNT; k++) rv.add("none");   // option on the cart)
    changed = true;
    Serial.println("Migrated settings: added planets.wheels.reverse_per");
  }
  if (changed) saveSettings();
}

static void saveStats();   // fwd

static void loadStats() {
  File f = LittleFS.open("/stats.json", "r");
  if (!f) {                                 // first boot: create it so it always exists
    g_pressesTriggered = 0; g_pressesIgnored = 0;
    saveStats();
    return;
  }
  JsonDocument d;
  if (!deserializeJson(d, f)) {
    g_pressesTriggered = d["presses_triggered"] | 0;
    g_pressesIgnored   = d["presses_ignored"]   | 0;
  }
  f.close();
}

static void saveStats() {
  File f = LittleFS.open("/stats.json", "w");
  if (!f) return;
  JsonDocument d;
  d["presses_triggered"] = g_pressesTriggered;
  d["presses_ignored"]   = g_pressesIgnored;
  serializeJson(d, f);
  f.close();
}

static void dumpFileToSerial(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f) { Serial.printf("(%s missing)\n", path); return; }
  Serial.printf("---- %s ----\n", path);
  while (f.available()) Serial.write(f.read());
  Serial.println();
  f.close();
}

static void parseMacs() {
  JsonArray macs = g_settings["planets"]["mac_address"].as<JsonArray>();
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++) {
    const char *s = (i < (int)macs.size()) ? macs[i].as<const char *>() : nullptr;
    g_macValid[i] = parseMac(s, g_planetMac[i]);
  }
}

// ----------------------------- ESP-NOW ---------------------------------------
static int planetByMac(const uint8_t *mac) {
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++)
    if (g_macValid[i] && memcmp(mac, g_planetMac[i], 6) == 0) return i;
  return -1;
}

static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 2 || data[0] != SOLAR_MAGIC) return;
  if (data[1] == SOLAR_MSG_FW_REQ) {                   // a cart wants image bytes: loop() answers
    int i = planetByMac(mac);
    if (i < 0 || len < (int)sizeof(FwReqMsg)) return;
    const FwReqMsg *r = (const FwReqMsg *)data;
    uint8_t next = (g_fwReqHead + 1) % FW_REQQ;
    if (next == g_fwReqTail) return;                   // full: the cart asks again
    g_fwReqq[g_fwReqHead] = {(uint8_t)i, r->fw_stamp, r->offset, r->count};
    g_fwReqHead = next;
    return;
  }
  if (data[1] != SOLAR_MSG_STATUS || len < (int)sizeof(StatusMsg)) return;
  StatusMsg s; memcpy(&s, data, sizeof(s));

  int idx = planetByMac(mac);
  if (idx < 0 && s.planet_index < SOLAR_PLANET_COUNT)  // fall back to echoed index
    idx = s.planet_index;
  if (idx < 0) return;

  PlanetState &p = g_planet[idx];
  p.lastSeenMs = millis();
  p.lightOn    = s.light_on;
  p.moving     = s.moving;
  p.tempC      = s.temp_c;
  p.rssi       = s.rssi;
  if (!s.cfg_ok && p.cfgOk)             // config verification just failed on the cart
    addLog("Planet " + String(idx) + " driver cfg UNVERIFIED (UART)");
  p.cfgOk      = s.cfg_ok;
  if (s.home_pulsed) p.homeCount++;
  p.error = s.error;                    // always ERR_NONE now (fault sensing removed)
  p.fwStamp     = s.fw_stamp;
  p.otaState    = s.ota_state;
  p.otaProgress = s.ota_progress;
  p.otaErr      = s.ota_err;
}

static void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t pi = {};
  memcpy(pi.peer_addr, mac, 6);
  pi.channel = 0;             // follow current channel (AP == STA channel on one radio)
  pi.ifidx   = WIFI_IF_AP;    // planets discover us by our AP SSID/channel
  pi.encrypt = false;
  esp_now_add_peer(&pi);
}

static void refreshPeers() {
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++)
    if (g_macValid[i]) addPeer(g_planetMac[i]);
}

static void sendCmd(int i, const CmdMsg &m) {
  if (i < 0 || i >= SOLAR_PLANET_COUNT || !g_macValid[i]) return;
  esp_now_send(g_planetMac[i], (const uint8_t *)&m, sizeof(m));
}

// ----------------------------- Speed model -----------------------------------
// Pack the three per-move TMC params (microsteps / chopper / MicroPlyer) and planet i's wheel
// reversal ("none" / "left" / "right" / "both") into the one byte the planet decodes with
// solarMicrostepsFromEnum()/SOLAR_DRVCFG_*.
static uint8_t packDriverCfg(JsonObject pl, int i) {
  uint16_t ms   = pl["wheels"]["stepper"]["microsteps"] | 16;
  bool spread   = (String((const char *)(pl["wheels"]["stepper"]["chopper_mode"] | "stealth")) == "spread");
  bool interp   = (pl["wheels"]["stepper"]["microplyer"] | true);
  String rev    = (const char *)(pl["wheels"]["reverse_per"][i] | "none");
  return solarPackDriverCfg(ms, spread, interp) |
         ((rev == "left"  || rev == "both") ? SOLAR_DRVCFG_REV_L : 0) |
         ((rev == "right" || rev == "both") ? SOLAR_DRVCFG_REV_R : 0);
}

// Fill the CmdMsg light brightness for planet i from the settings "light_brightness_per" array.
static void setCmdLight(CmdMsg &m, int i) {
  float pct = g_settings["planets"]["light_brightness_per"][i] | 100.0f;   // float: accepts 50 or 50.5
  m.light_brightness = (uint8_t)lroundf(constrain(pct, 0.0f, 100.0f));
}

// Offer the cart image in the command: only while IDLE with auto-update on. Otherwise fw_stamp
// stays 0 and a cart that is downloading pauses until it is offered again.
static void setCmdFw(CmdMsg &m) {
  bool autoUpdate = g_settings["planets"]["auto_update_firmware"] | true;
  if (!g_fw.present || !autoUpdate || g_mode != MODE_IDLE) return;
  m.fw_stamp = g_fw.stamp;
  m.fw_size  = g_fw.size;
  memcpy(m.fw_md5, g_fw.md5, sizeof(m.fw_md5));
}

// Given the per-rail OUTER wheel speed, compute the two wheel speeds so the cart rolls around the
// ring, and the resulting orbital speed for the web animation. log_scale_speed_rpm is the OUTER
// (faster) wheel; the inner rail is smaller by 2*width_mm, so the inner wheel turns proportionally
// slower (ratio = Rin/Rout <= 1). L = outer, R = inner (matches the CmdMsg field comments).
static CmdMsg buildRunCmd(int i, bool light) {
  JsonObject pl   = g_settings["planets"];
  float dWheel    = pl["wheels"]["diameter_mm"] | 36.0f;
  float railW     = pl["rail"]["width_mm"]      | 70.0f;
  float outerD    = pl["rail"]["outer_diameter_mm"][i] | 640.0f;
  float base      = pl["rail"]["log_scale_speed_rpm"][i] | 100.0f;
  float maxRpm    = pl["wheels"]["stepper"]["max_speed_rpm"][i] | 500.0f;

  float Rout = outerD * 0.5f;
  float Rin  = Rout - railW;
  if (Rin < 1.0f) Rin = 1.0f;             // guard degenerate geometry (outerD <= 2*width)
  float wheelL = base;                    // outer track = reference (the higher speed)
  float wheelR = base * (Rin / Rout);     // inner track proportionally slower
  if (wheelL > maxRpm) { float k = maxRpm / wheelL; wheelL *= k; wheelR *= k; }

  float sign = ORBIT_SIGN;                // every planet orbits the same way: counter-clockwise
  float orbital = (Rout > 0.1f) ? (wheelL * dWheel / (2.0f * Rout)) : 0.0f;
  g_planet[i].orbitalRpm = orbital;       // orbital speed, positive = counter-clockwise (web UI)

  CmdMsg m = {};
  m.magic = SOLAR_MAGIC; m.type = SOLAR_MSG_CMD; m.planet_index = i;
  m.command     = CMD_RUN;
  m.wheel_l_rpm = wheelL * sign;
  m.wheel_r_rpm = wheelR * sign;
  m.light_on    = light ? 1 : 0;
  setCmdLight(m, i);
  setCmdFw(m);
  m.current_ma  = (uint16_t)((pl["wheels"]["stepper"]["current_amp"] | 0.2f) * 1000.0f);
  m.step_degree = pl["wheels"]["stepper"]["step_degree"] | 1.8f;
  m.driver_cfg  = packDriverCfg(pl, i);
  return m;
}

static CmdMsg buildSimpleCmd(int i, PlanetCmd cmd, bool light) {
  JsonObject pl = g_settings["planets"];
  CmdMsg m = {};
  m.magic = SOLAR_MAGIC; m.type = SOLAR_MSG_CMD; m.planet_index = i;
  m.command    = cmd;
  m.light_on   = light ? 1 : 0;
  setCmdLight(m, i);
  setCmdFw(m);
  m.current_ma = (uint16_t)((pl["wheels"]["stepper"]["current_amp"] | 0.2f) * 1000.0f);
  m.step_degree = pl["wheels"]["stepper"]["step_degree"] | 1.8f;
  m.driver_cfg  = packDriverCfg(pl, i);
  if (cmd == CMD_HOME_AND_STOP) {         // slow homing move, with differential
    float railW  = pl["rail"]["width_mm"]      | 70.0f;
    float outerD = pl["rail"]["outer_diameter_mm"][i] | 640.0f;
    float Rout = outerD * 0.5f, Rin = Rout - railW;
    if (Rin < 1.0f) Rin = 1.0f;
    m.wheel_l_rpm = ORBIT_SIGN * HOMING_BASE_RPM;                 // outer wheel (faster)
    m.wheel_r_rpm = ORBIT_SIGN * HOMING_BASE_RPM * (Rin / Rout);  // inner wheel (slower)
    g_planet[i].orbitalRpm = 0;
  } else {
    g_planet[i].orbitalRpm = 0;
  }
  return m;
}

static int activeCandles() {
  int c = g_settings["modes"]["menorah"]["active_candles"] | 0;
  if (c < 0) c = 0;
  if (c > SOLAR_PLANET_COUNT) c = SOLAR_PLANET_COUNT;
  return c;
}

// Send the command that matches the current mode to every planet (mode entry + heartbeat).
static void broadcastModeCmd(bool homingEntry) {
  int candles = activeCandles();
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++) {
    if (!g_macValid[i]) continue;
    switch (g_mode) {
      case MODE_SOLAR:    sendCmd(i, buildRunCmd(i, true)); break;
      case MODE_TEST:     sendCmd(i, buildRunCmd(i, true)); break;
      case MODE_MENORAH:
        if (homingEntry) sendCmd(i, buildSimpleCmd(i, CMD_HOME_AND_STOP, i < candles));
        else             sendCmd(i, buildSimpleCmd(i, CMD_LIGHT, i < candles));
        break;
      case MODE_IDLE:
      case MODE_COOLDOWN:
      default:            sendCmd(i, buildSimpleCmd(i, CMD_STOP, false)); break;
    }
  }
  g_lastHeartbeatMs = millis();
}

// ----------------------------- LED fire effect --------------------------------
// Two Fire2012 variants selectable by settings; cooling/sparking scaled by effect_speed.
static void fireStep(uint8_t cooling, uint8_t sparking, const CRGBPalette16 &pal) {
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t c = random8(0, ((cooling * 10) / NUM_LEDS) + 2);
    g_heat[i] = qsub8(g_heat[i], c);
  }
  for (int k = NUM_LEDS - 1; k >= 2; k--)
    g_heat[k] = (g_heat[k - 1] + g_heat[k - 2] + g_heat[k - 2]) / 3;
  if (random8() < sparking) {
    int y = random8(7);
    g_heat[y] = qadd8(g_heat[y], random8(160, 255));
  }
  for (int j = 0; j < NUM_LEDS; j++) {
    uint8_t idx = scale8(g_heat[j], 240);
    g_leds[j] = ColorFromPalette(pal, idx);
  }
}

static void serviceLeds() {
  const char *sect = nullptr;
  if (g_mode == MODE_SOLAR || g_mode == MODE_TEST) sect = "solar";
  else if (g_mode == MODE_MENORAH)                 sect = "menorah";

  if (!sect) {                                     // IDLE / COOLDOWN: dark
    fill_solid(g_leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    return;
  }

  JsonObject led = g_settings["sun_led"][sect];
  uint8_t bright  = (uint8_t)((led["brightness_per"] | 100) * 255 / 100);
  int     speed   = led["effect_speed"] | 5;
  String  effect  = led["effect"] | "fire01";
  uint32_t frameMs = constrain(70 - speed * 6, 12, 70);

  if (millis() - g_lastFireMs < frameMs) return;
  g_lastFireMs = millis();

  FastLED.setBrightness(bright);
  if (effect == "fire02")
    fireStep(70, 90, CRGBPalette16(CRGB::Black, CRGB::DarkRed, CRGB::Orange, CRGB::White));
  else
    fireStep(55, 120, HeatColors_p);
  FastLED.show();
}

// ----------------------------- State machine ---------------------------------
static uint32_t minToMs(int m) { return (uint32_t)m * 60000UL; }

static void enterMode(SolarMode m) {
  g_mode = m;
  g_phaseStartMs = millis();
  g_phaseEndMs = 0;
  switch (m) {
    case MODE_SOLAR:
      g_phaseEndMs = millis() + minToMs(g_settings["modes"]["solar"]["runtime_minutes"] | 5);
      addLog("SOLAR start");
      broadcastModeCmd(false);
      break;
    case MODE_TEST:
      g_phaseEndMs = millis() + minToMs(g_settings["modes"]["test"]["runtime_minutes"] | 30);
      addLog("TEST start");
      broadcastModeCmd(false);
      break;
    case MODE_COOLDOWN:
      g_phaseEndMs = millis() + minToMs(g_settings["modes"]["solar"]["cooldown_minutes"] | 1);
      addLog("COOLDOWN");
      broadcastModeCmd(false);
      break;
    case MODE_MENORAH:
      addLog("MENORAH (" + String(activeCandles()) + " candles)");
      broadcastModeCmd(true);              // home-and-stop entry
      break;
    case MODE_IDLE:
    default:
      g_mode = MODE_IDLE;
      addLog("IDLE");
      broadcastModeCmd(false);
      break;
  }
}

static void onShowEnd() {                  // called when SOLAR runtime elapses
  if (activeCandles() > 0) enterMode(MODE_MENORAH);
  else                     enterMode(MODE_COOLDOWN);
}

static void serviceStateMachine() {
  if (g_phaseEndMs && (int32_t)(millis() - g_phaseEndMs) >= 0) {
    switch (g_mode) {
      case MODE_SOLAR:    onShowEnd(); break;
      case MODE_COOLDOWN: enterMode(MODE_IDLE); break;
      case MODE_TEST:     enterMode(MODE_IDLE); break;
      default: g_phaseEndMs = 0; break;
    }
  }
  if (millis() - g_lastHeartbeatMs > HEARTBEAT_MS)
    broadcastModeCmd(false);
}

// ----------------------------- Cart firmware update ---------------------------
// The last uploaded cart image is the LittleFS file CART_FW_PATH. While IDLE (and auto-update is
// on) every CmdMsg offers it; a cart running an older build pulls it over ESP-NOW with FwReqMsg and
// the Sun answers each request with FwDataMsg chunks read from the file. Nothing per cart is kept
// here, so any number of carts download at once. The ESP-NOW callback only queues the requests;
// loop() reads the file and sends the chunks.
static String   g_upErr;                    // why the last upload was rejected ("" = accepted)
static File     g_upFile;
static volatile bool g_fwUploaded = false;  // an upload finished: log it from loop()

static String fwStampText(uint32_t s) {     // build stamp (UTC Unix seconds) -> readable
  time_t t = s; struct tm b; gmtime_r(&t, &b);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d UTC", b.tm_year + 1900, b.tm_mon + 1,
           b.tm_mday, b.tm_hour, b.tm_min);
  return String(buf);
}

// Inspect CART_FW_PATH: it must be one complete ESP32-C3 app image (header + segments + checksum
// padding + optional SHA-256 add up to the file size) carrying the SOLAR_FW_MARK build stamp.
// Sets g_fw (size, MD5, stamp); g_fw.present stays false for anything else.
static void loadCartFw() {
  CartFw fw = {};
  File f = LittleFS.open(CART_FW_PATH, "r");
  if (!f) { g_fw = fw; return; }
  uint32_t size = f.size();
  uint8_t h[24];
  bool ok = f.read(h, sizeof(h)) == sizeof(h) && h[0] == 0xE9 && (h[12] | (h[13] << 8)) == 5;
  uint32_t pos = sizeof(h);
  for (uint8_t s = 0; ok && s < h[1]; s++) {          // segment: load address, length, data
    uint32_t seg[2];
    ok = f.seek(pos) && f.read((uint8_t *)seg, sizeof(seg)) == sizeof(seg) && seg[1] < size;
    pos += sizeof(seg) + seg[1];
    ok = ok && pos < size;
  }
  pos = (pos | 15) + 1;                                // padded to 16, checksum as the last byte
  if (h[23]) pos += 32;                                // SHA-256 appended
  ok = ok && pos == size;

  static const char MK[] = SOLAR_FW_MARK;
  static uint8_t buf[2048];
  char cap[SOLAR_FW_STAMP_LEN + 1];
  uint8_t mk = 0, capLen = 0;
  bool capturing = false;
  MD5Builder md5;
  md5.begin();
  f.seek(0);
  int n;
  while (ok && (n = f.read(buf, sizeof(buf))) > 0) {
    md5.add(buf, n);
    for (int k = 0; k < n && !fw.stamp; k++) {         // find the marker, then read the stamp
      char c = (char)buf[k];
      if (capturing) {
        cap[capLen++] = c;
        if (capLen == SOLAR_FW_STAMP_LEN) {            // always stop here: cap[] is full
          cap[capLen] = 0;
          fw.stamp = solarFwStamp(cap);                // 0 if not digits -> look for another marker
          capturing = false; mk = 0;
        }
      } else if (c == MK[mk]) {
        if (++mk == sizeof(MK) - 1) { capturing = true; capLen = 0; }
      } else {
        mk = (c == MK[0]) ? 1 : 0;                     // the marker has no repeated prefix
      }
    }
  }
  f.close();
  md5.calculate();
  md5.getBytes(fw.md5);
  fw.size = size;
  fw.present = ok && fw.stamp;
  g_fw = fw;
}

// Upload body handler (async_tcp task): the image streams straight into CART_FW_PATH. The flash
// holds one image, so the old one is deleted first (under g_fwLock: loop() may be reading it).
static void fwUploadChunk(AsyncWebServerRequest *req, const String &name, size_t index,
                          uint8_t *data, size_t len, bool final) {
  if (index == 0) {
    g_upErr = "";
    if (len < 24 || data[0] != 0xE9 || (data[12] | (data[13] << 8)) != 5) {
      g_upErr = "not an ESP32-C3 firmware image";
      return;
    }
    xSemaphoreTake(g_fwLock, portMAX_DELAY);
    g_fw.present = false;                              // stop offering / serving the old image
    LittleFS.remove(CART_FW_PATH);
    g_upFile = LittleFS.open(CART_FW_PATH, "w");
    xSemaphoreGive(g_fwLock);
    if (!g_upFile) { g_upErr = "cannot write to the Sun's flash"; return; }
  }
  if (!g_upErr.isEmpty() || !g_upFile) return;
  if (g_upFile.write(data, len) != len) {
    g_upErr = "image does not fit in the Sun's flash";
    g_upFile.close();
    LittleFS.remove(CART_FW_PATH);
    return;
  }
  if (!final) return;
  g_upFile.close();
  loadCartFw();
  if (!g_fw.present) g_upErr = "not a complete Solar System cart build";
  g_fwUploaded = true;
}

// Answer queued chunk requests from the image file (at most 8 chunks per request).
static void serviceFwRequests() {
  if (g_fwReqTail == g_fwReqHead) return;
  if (xSemaphoreTake(g_fwLock, 0) != pdTRUE) return;   // an upload is replacing the image
  File f = g_fw.present ? LittleFS.open(CART_FW_PATH, "r") : File();
  static FwDataMsg d;
  while (g_fwReqTail != g_fwReqHead) {
    FwReq r = g_fwReqq[g_fwReqTail];
    g_fwReqTail = (g_fwReqTail + 1) % FW_REQQ;
    if (!f || r.stamp != g_fw.stamp) continue;         // stale: the cart follows the offer instead
    uint32_t off = r.offset;
    for (uint8_t k = 0; k < r.count && k < 8 && off < g_fw.size; k++) {
      uint8_t n = (uint8_t)min((uint32_t)SOLAR_FW_CHUNK, g_fw.size - off);
      d.magic = SOLAR_MAGIC; d.type = SOLAR_MSG_FW_DATA;
      d.fw_stamp = g_fw.stamp; d.offset = off; d.len = n;
      if (!f.seek(off) || f.read(d.data, n) != n) break;
      if (esp_now_send(g_planetMac[r.planet], (const uint8_t *)&d, SOLAR_FW_DATA_HDR + n) != ESP_OK) break;
      off += n;
    }
  }
  if (f) f.close();
  xSemaphoreGive(g_fwLock);
}

// Uploads and each cart's download progress, for the web log.
// ----------------------------- Link watch ------------------------------------
// ESP-NOW unicasts are ACKed by the receiving radio, so onSent() counts per cart which commands
// arrived. serviceLinkWatch() logs (serial) when a cart stops ACKing or stops sending status for
// > LINK_GAP_MS, and when it recovers, to tell which side of a link drop failed.
#define LINK_GAP_MS 3000
static void onSent(const uint8_t *mac, esp_now_send_status_t st) {
  int i = planetByMac(mac);
  if (i < 0) return;
  PlanetState &p = g_planet[i];
  if (st == ESP_NOW_SEND_SUCCESS) { p.txOk++; p.txFailRun = 0; }
  else                            { p.txFail++; if (p.txFailRun < 255) p.txFailRun++; }
}

static void serviceLinkWatch() {
  static bool     statusGap[SOLAR_PLANET_COUNT], failing[SOLAR_PLANET_COUNT];
  static uint32_t gapFrom[SOLAR_PLANET_COUNT];
  uint32_t now = millis();
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++) {
    PlanetState &p = g_planet[i];
    if (!g_macValid[i] || !p.lastSeenMs) continue;
    uint32_t last = p.lastSeenMs;
    if (!statusGap[i] && msSince(last) > LINK_GAP_MS) {
      statusGap[i] = true; gapFrom[i] = last;
      Serial.printf("[%lu] LINK p%d: no status for %lu ms (commands ACKed %lu, not %lu)\n",
                    (unsigned long)now, i, (unsigned long)msSince(last),
                    (unsigned long)p.txOk, (unsigned long)p.txFail);
    } else if (statusGap[i] && last != gapFrom[i]) {
      statusGap[i] = false;
      Serial.printf("[%lu] LINK p%d: status back after %lu ms (commands ACKed %lu, not %lu)\n",
                    (unsigned long)now, i, (unsigned long)(last - gapFrom[i]),
                    (unsigned long)p.txOk, (unsigned long)p.txFail);
    }
    uint8_t run = p.txFailRun;
    if (!failing[i] && run >= 2) {
      failing[i] = true;
      Serial.printf("[%lu] LINK p%d: commands not ACKed (%u in a row)\n", (unsigned long)now, i, run);
    } else if (failing[i] && run == 0) {
      failing[i] = false;
      Serial.printf("[%lu] LINK p%d: commands ACKed again\n", (unsigned long)now, i);
    }
  }
}

static void serviceCartFw() {
  if (g_fwUploaded) {
    g_fwUploaded = false;
    addLog(g_fw.present ? "Cart firmware uploaded: build " + fwStampText(g_fw.stamp) + " (" +
                          String(g_fw.size / 1024) + " KB)"
                        : "Cart firmware upload rejected: " + g_upErr);
  }
  serviceFwRequests();
  static uint8_t  lastState[SOLAR_PLANET_COUNT];
  static uint32_t lastStamp[SOLAR_PLANET_COUNT];
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++) {
    PlanetState &p = g_planet[i];
    if (!g_macValid[i] || !p.lastSeenMs || msSince(p.lastSeenMs) > 5000) continue;
    if (p.otaState != lastState[i]) {
      if (p.otaState == OTA_DOWNLOADING) addLog("Planet " + String(i + 1) + ": downloading the new cart build");
      if (p.otaState == OTA_FAILED)      addLog("Planet " + String(i + 1) + ": update failed (error " + String(p.otaErr) + "), retrying in 1 min");
      lastState[i] = p.otaState;
    }
    if (p.fwStamp != lastStamp[i]) {
      if (lastStamp[i]) addLog("Planet " + String(i + 1) + ": now runs build " + fwStampText(p.fwStamp));
      lastStamp[i] = p.fwStamp;
    }
  }
}

// ----------------------------- Buttons ---------------------------------------
static void onStartButton() {
  if (g_mode == MODE_IDLE || g_mode == MODE_MENORAH) {
    g_pressesTriggered++; saveStats();
    enterMode(MODE_SOLAR);
  } else {                                 // SOLAR / COOLDOWN / TEST
    g_pressesIgnored++; saveStats();
    addLog("press ignored");
  }
}

static void serviceButtons() {
  // Start button: debounced falling edge.
  static bool lastStart = HIGH;
  static uint32_t startEdgeMs = 0;
  bool st = digitalRead(PIN_BTN_START);
  if (st == LOW && lastStart == HIGH && millis() - startEdgeMs > 250) {
    startEdgeMs = millis();
    onStartButton();
  }
  lastStart = st;

  // Boot button: 5s hold -> factory reset.
  static uint32_t bootDownMs = 0;
  static bool bootLatched = false;
  if (digitalRead(PIN_BTN_BOOT) == LOW) {
    if (bootDownMs == 0) bootDownMs = millis();
    if (!bootLatched && millis() - bootDownMs >= FACTORY_HOLD_MS) {
      bootLatched = true;
      g_doFactoryReset = true;
    }
  } else {
    bootDownMs = 0; bootLatched = false;
  }
}

// ----------------------------- Wi-Fi ------------------------------------------
static void wifiStart() {
  WiFi.persistent(false);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    g_staDropReason = info.wifi_sta_disconnected.reason; g_staDrops++;
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { g_staUps++; }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.mode(WIFI_AP_STA);
  String apSsid = g_settings["wifi"]["ap_ssid"] | SOLAR_AP_SSID_DEFAULT;
  String apPass = g_settings["wifi"]["ap_pass"] | "didivardi";
  const char *pw = apPass.length() >= 8 ? apPass.c_str() : nullptr;
  WiFi.softAP(apSsid.c_str(), pw);

  String staSsid = g_settings["wifi"]["sta_ssid"] | "";
  if (staSsid.length()) {
    String staPass = g_settings["wifi"]["sta_pass"] | "";
    WiFi.begin(staSsid.c_str(), staPass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(100);
    g_staOk = (WiFi.status() == WL_CONNECTED);
  }
  Serial.printf("AP '%s' IP %s\n", apSsid.c_str(), WiFi.softAPIP().toString().c_str());
  if (g_staOk) Serial.printf("STA IP %s\n", WiFi.localIP().toString().c_str());
}

static void wifiLogEvents() {
  if (g_staDrops != g_staDropsLogged) {
    g_staDropsLogged = g_staDrops;
    addLog("STA link lost (reason " + String(g_staDropReason) + ")");
    Serial.printf("[%s] STA link lost, reason %u (drop #%lu)\n", fmtUptime().c_str(),
                  g_staDropReason, (unsigned long)g_staDropsLogged);
  }
  if (g_staUps != g_staUpsLogged) {
    g_staUpsLogged = g_staUps;
    Serial.printf("[%s] STA got IP %s\n", fmtUptime().c_str(), WiFi.localIP().toString().c_str());
  }
}

static void wifiRetry() {
  if (g_staOk) return;
  String staSsid = g_settings["wifi"]["sta_ssid"] | "";
  if (!staSsid.length()) return;
  if (millis() - g_lastStaTryMs < STA_RETRY_MS) return;
  g_lastStaTryMs = millis();
  if (WiFi.status() == WL_CONNECTED) { g_staOk = true; return; }
  String staPass = g_settings["wifi"]["sta_pass"] | "";
  WiFi.begin(staSsid.c_str(), staPass.c_str());
}

// ----------------------------- Web endpoints ----------------------------------
static void buildStatusJson(String &out) {
  JsonDocument d;
  const char *modeNames[] = {"IDLE", "SOLAR", "COOLDOWN", "MENORAH", "TEST"};
  d["mode"] = modeNames[g_mode];
  bool countdown = (g_mode == MODE_SOLAR || g_mode == MODE_COOLDOWN || g_mode == MODE_TEST);
  d["countdown"] = countdown;
  if (countdown && g_phaseEndMs) {
    int32_t rem = ((int32_t)(g_phaseEndMs - millis())) / 1000;
    d["phase_remaining_s"] = rem > 0 ? rem : 0;
  } else {
    d["phase_remaining_s"] = 0;
  }
  d["phase_elapsed_s"] = (int32_t)((millis() - g_phaseStartMs) / 1000);
  d["active_candles"]  = activeCandles();

  JsonArray arr = d["planets"].to<JsonArray>();
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++) {
    JsonObject o = arr.add<JsonObject>();
    bool online = g_macValid[i] && g_planet[i].lastSeenMs &&
                  (msSince(g_planet[i].lastSeenMs) < 5000);
    o["online"]      = online;
    o["configured"]  = g_macValid[i];
    o["moving"]      = g_planet[i].moving;
    o["light_on"]    = g_planet[i].lightOn;
    o["error"]       = g_planet[i].error;
    o["orbital_rpm"] = g_planet[i].orbitalRpm;
    o["home_count"]  = g_planet[i].homeCount;
    o["home_deg"]    = (i % 2 == 0) ? 180 : 0;   // home line spread horizontally (L/R)
    if (online) {
      o["temp_c"]   = g_planet[i].tempC;
      int r = g_planet[i].rssi;
      o["rssi"]     = r;
      o["rssi_pct"] = r ? constrain(2 * (r + 100), 0, 100) : 0;   // -100dBm=0%, -50dBm=100%
      o["cfg_ok"]   = g_planet[i].cfgOk;
      o["fw"]       = g_planet[i].fwStamp;
      o["ota"]      = g_planet[i].otaState;      // OtaState: download progress / result
      o["ota_pct"]  = g_planet[i].otaProgress;
      o["ota_err"]  = g_planet[i].otaErr;
    }
    o["tx_ok"]   = g_planet[i].txOk;           // commands ACKed / not ACKed by the cart's radio
    o["tx_fail"] = g_planet[i].txFail;
  }

  JsonObject fw = d["cart_fw"].to<JsonObject>();
  fw["present"] = g_fw.present;
  fw["auto"]    = g_settings["planets"]["auto_update_firmware"] | true;
  if (g_fw.present) { fw["stamp"] = g_fw.stamp; fw["size"] = g_fw.size; }

  JsonObject net = d["net"].to<JsonObject>();
  net["ap_ip"]  = WiFi.softAPIP().toString();
  net["sta_ip"] = g_staOk ? WiFi.localIP().toString() : String("");
  net["mdns"]   = String(MDNS_HOST) + ".local";

  JsonArray lg = d["log"].to<JsonArray>();
  for (int k = 0; k < LOG_N; k++) {
    int idx = (g_logHead + k) % LOG_N;
    if (g_log[idx].length()) lg.add(g_log[idx]);
  }
  serializeJson(d, out);
}

static void setupWeb() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", String(FPSTR(INDEX_HTML)));
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    String s; buildStatusJson(s);
    req->send(200, "application/json", s);
  });

  server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/settings.json", "application/json");
  });

  server.on("/api/stats", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument d;
    d["presses_triggered"] = g_pressesTriggered;
    d["presses_ignored"]   = g_pressesIgnored;
    String s; serializeJson(d, s);
    req->send(200, "application/json", s);
  });

  server.on("/api/stats/reset", HTTP_POST, [](AsyncWebServerRequest *req) {
    g_pressesTriggered = 0; g_pressesIgnored = 0; saveStats();
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // Restore settings.json to defaults (same as a 5 s BOOT-button hold); applied in loop().
  server.on("/api/settings/reset", HTTP_POST, [](AsyncWebServerRequest *req) {
    g_doFactoryReset = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // POST /api/settings — JSON body replaces settings.json; applied in loop().
  auto *settingsPost = new AsyncCallbackJsonWebHandler(
      "/api/settings", [](AsyncWebServerRequest *req, JsonVariant &json) {
        g_pendingSettings = "";
        serializeJson(json, g_pendingSettings);
        g_reloadSettings = true;
        req->send(200, "application/json", "{\"ok\":true}");
      });
  server.addHandler(settingsPost);

  // POST /api/control — {"action":"start"|"test"|"stop"}; applied in loop().
  auto *controlPost = new AsyncCallbackJsonWebHandler(
      "/api/control", [](AsyncWebServerRequest *req, JsonVariant &json) {
        g_pendingAction = String((const char *)(json["action"] | ""));
        req->send(200, "application/json", "{\"ok\":true}");
      });
  server.addHandler(controlPost);

  // Cart firmware upload (multipart .bin); the carts then fetch it over ESP-NOW by themselves.
  server.on("/api/planet_fw", HTTP_POST, [](AsyncWebServerRequest *req) {
    JsonDocument d;
    d["ok"] = g_upErr.isEmpty() && g_fw.present;
    if (!g_upErr.isEmpty()) d["error"] = g_upErr;
    else if (g_fw.present) d["stamp"] = g_fw.stamp;
    String s; serializeJson(d, s);
    req->send(200, "application/json", s);
  }, fwUploadChunk);

  ElegantOTA.begin(&server);
  server.begin();
}

// ----------------------------- Deferred work ----------------------------------
static void applyPendingSettings() {
  File f = LittleFS.open("/settings.json", "w");
  if (f) { f.print(g_pendingSettings); f.close(); }
  loadSettings();
  parseMacs();
  refreshPeers();
  addLog("settings updated");
  // Live effects: menorah candle count changes take effect immediately.
  if (g_mode == MODE_MENORAH) {
    if (activeCandles() == 0) enterMode(MODE_IDLE);
    else broadcastModeCmd(false);          // re-apply candle lights
  }
}

static void applyPendingAction() {
  String a = g_pendingAction; g_pendingAction = "";
  if (a == "start")      enterMode(MODE_SOLAR);
  else if (a == "test")  enterMode(MODE_TEST);
  else if (a == "stop")  enterMode(MODE_IDLE);
}

static void doFactoryReset() {
  writeDefaults();
  loadSettings();
  parseMacs();
  refreshPeers();
  Serial.println("*** FACTORY RESET: settings restored to defaults ***");
  dumpFileToSerial("/settings.json");
  addLog("factory reset");
  enterMode(MODE_IDLE);
}

// ----------------------------- Setup / loop ----------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
  loadSettings();
  migrateSettings();
  loadStats();
  parseMacs();
  g_fwLock = xSemaphoreCreateMutex();
  loadCartFw();

  Serial.println("\n==== Solar System — SUN boot ====");
  dumpFileToSerial("/settings.json");
  dumpFileToSerial("/stats.json");
  if (g_fw.present) Serial.printf("Cart firmware: build %s, %lu bytes\n", fwStampText(g_fw.stamp).c_str(), (unsigned long)g_fw.size);
  else              Serial.println("Cart firmware: none uploaded");

  FastLED.addLeds<LED_TYPE, PIN_LED, LED_ORDER>(g_leds, NUM_LEDS);
  FastLED.setBrightness(0);
  fill_solid(g_leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  wifiStart();

  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local\n", MDNS_HOST);
  }

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onRecv);
    esp_now_register_send_cb(onSent);
    refreshPeers();
  } else {
    Serial.println("ESP-NOW init failed!");
  }

  setupWeb();
  enterMode(MODE_IDLE);
}

void loop() {
  serviceButtons();
  serviceStateMachine();
  serviceLeds();
  wifiLogEvents();
  wifiRetry();
  serviceCartFw();
  serviceLinkWatch();
  ElegantOTA.loop();

  if (g_reloadSettings) { g_reloadSettings = false; applyPendingSettings(); }
  if (g_pendingAction.length()) applyPendingAction();
  if (g_doFactoryReset) { g_doFactoryReset = false; doFactoryReset(); }
}
