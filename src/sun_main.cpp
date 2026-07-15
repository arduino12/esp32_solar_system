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

#define FASTLED_INTERNAL
#include <FastLED.h>

#include "solar_protocol.h"
#include "sun_web_ui.h"

// ----------------------------- Pin map (from spec) --------------------------
static const uint8_t PIN_LED         = 4;   // WS2812 data
static const uint8_t PIN_BTN_START   = 5;   // Start button, active-low
static const uint8_t PIN_BTN_BOOT    = 0;   // onboard boot button, active-low (5s hold = factory reset)

// ----------------------------- LED config -----------------------------------
#define NUM_LEDS        60
#define LED_TYPE        WS2812B
#define LED_ORDER       GRB
static CRGB      g_leds[NUM_LEDS];
static uint8_t   g_heat[NUM_LEDS];
static uint32_t  g_lastFireMs = 0;

// ----------------------------- Timing constants ------------------------------
#define HEARTBEAT_MS    2000        // resend current command set to planets
#define HOMING_BASE_RPM 40.0f       // wheel reference speed while homing for menorah
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
    "wheels": {
      "diameter_mm": 36,
      "stepper": { "current_amp": 0.2, "microsteps": 16, "step_degree": 1.8,
                   "max_speed_rpm": [500,500,500,500,500,500,500,500] }
    },
    "rail": {
      "width_mm": 70,
      "inner_diameter_mm": [500,600,700,800,900,1000,1100,1200],
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
  uint32_t lastSeenMs;
  uint8_t  error;
  bool     lightOn;
  bool     moving;
  uint32_t homeCount;     // increments each reported HOME pulse
  float    orbitalRpm;    // signed; Sun-computed for the web animation
  int8_t   tempC;         // planet die temperature, °C (reported by the cart)
  int8_t   rssi;          // planet<->Sun signal, dBm (reported by the cart)
};
static PlanetState g_planet[SOLAR_PLANET_COUNT];

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

// ----------------------------- Small helpers ---------------------------------
static String fmtUptime() {
  uint32_t s = millis() / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u",
           (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
  return String(buf);
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
  JsonObject planets = g_settings["planets"].as<JsonObject>();
  if (!planets.isNull() && !planets["axial_rotation_speed_rpm"].isNull()) {
    planets.remove("axial_rotation_speed_rpm");
    saveSettings();
    Serial.println("Migrated settings: removed obsolete axial_rotation_speed_rpm");
  }
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
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 2 || data[0] != SOLAR_MAGIC || data[1] != SOLAR_MSG_STATUS) return;
  if (len < (int)sizeof(StatusMsg)) return;
  StatusMsg s; memcpy(&s, data, sizeof(s));

  int idx = -1;
  for (int i = 0; i < SOLAR_PLANET_COUNT; i++)         // match by MAC
    if (g_macValid[i] && memcmp(mac, g_planetMac[i], 6) == 0) { idx = i; break; }
  if (idx < 0 && s.planet_index < SOLAR_PLANET_COUNT)  // fall back to echoed index
    idx = s.planet_index;
  if (idx < 0) return;

  PlanetState &p = g_planet[idx];
  p.lastSeenMs = millis();
  p.lightOn    = s.light_on;
  p.moving     = s.moving;
  p.tempC      = s.temp_c;
  p.rssi       = s.rssi;
  if (s.home_pulsed) p.homeCount++;
  if (s.error != p.error && s.error != ERR_NONE)
    addLog("Planet " + String(idx) + (s.error == ERR_MOTOR_DISCONNECTED ? " motor disconnected"
                                                                        : " motor stuck"));
  p.error = s.error;
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
// Given the per-rail reference wheel speed, compute the two wheel speeds so the cart rolls
// around the ring (outer track faster than inner by the radius ratio), and the resulting
// orbital speed for the web animation. See plan "Per-planet speed computation".
static CmdMsg buildRunCmd(int i, bool light) {
  JsonObject pl   = g_settings["planets"];
  float dWheel    = pl["wheels"]["diameter_mm"] | 36.0f;
  float railW     = pl["rail"]["width_mm"]      | 70.0f;
  float innerD    = pl["rail"]["inner_diameter_mm"][i] | 500.0f;
  float base      = pl["rail"]["log_scale_speed_rpm"][i] | 100.0f;
  float maxRpm    = pl["wheels"]["stepper"]["max_speed_rpm"][i] | 500.0f;

  float Rin  = innerD * 0.5f;
  float Rout = Rin + railW;
  float wheelR = base;                    // inner track = reference
  float wheelL = base * (Rout / Rin);     // outer track proportionally faster
  if (wheelL > maxRpm) { float k = maxRpm / wheelL; wheelL *= k; wheelR *= k; }

  float sign = (i % 2 == 0) ? 1.0f : -1.0f;   // alternate direction for visual variety
  float orbital = (Rin > 0.1f) ? (wheelR * dWheel / (2.0f * Rin)) : 0.0f;
  g_planet[i].orbitalRpm = orbital * sign;

  CmdMsg m = {};
  m.magic = SOLAR_MAGIC; m.type = SOLAR_MSG_CMD; m.planet_index = i;
  m.command     = CMD_RUN;
  m.wheel_l_rpm = wheelL * sign;
  m.wheel_r_rpm = wheelR * sign;
  m.light_on    = light ? 1 : 0;
  m.current_ma  = (uint16_t)((pl["wheels"]["stepper"]["current_amp"] | 0.2f) * 1000.0f);
  m.microsteps  = pl["wheels"]["stepper"]["microsteps"] | 16;
  m.step_degree = pl["wheels"]["stepper"]["step_degree"] | 1.8f;
  return m;
}

static CmdMsg buildSimpleCmd(int i, PlanetCmd cmd, bool light) {
  JsonObject pl = g_settings["planets"];
  CmdMsg m = {};
  m.magic = SOLAR_MAGIC; m.type = SOLAR_MSG_CMD; m.planet_index = i;
  m.command    = cmd;
  m.light_on   = light ? 1 : 0;
  m.current_ma = (uint16_t)((pl["wheels"]["stepper"]["current_amp"] | 0.2f) * 1000.0f);
  m.microsteps = pl["wheels"]["stepper"]["microsteps"] | 16;
  m.step_degree = pl["wheels"]["stepper"]["step_degree"] | 1.8f;
  if (cmd == CMD_HOME_AND_STOP) {         // slow homing move, with differential
    float dWheel = pl["wheels"]["diameter_mm"] | 36.0f;
    float railW  = pl["rail"]["width_mm"]      | 70.0f;
    float innerD = pl["rail"]["inner_diameter_mm"][i] | 500.0f;
    float Rin = innerD * 0.5f, Rout = Rin + railW;
    m.wheel_r_rpm = HOMING_BASE_RPM;
    m.wheel_l_rpm = HOMING_BASE_RPM * (Rout / Rin);
    (void)dWheel;
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
                  (millis() - g_planet[i].lastSeenMs < 5000);
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
    }
  }

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

  Serial.println("\n==== Solar System — SUN boot ====");
  dumpFileToSerial("/settings.json");
  dumpFileToSerial("/stats.json");

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
  wifiRetry();
  ElegantOTA.loop();

  if (g_reloadSettings) { g_reloadSettings = false; applyPendingSettings(); }
  if (g_pendingAction.length()) applyPendingAction();
  if (g_doFactoryReset) { g_doFactoryReset = false; doFactoryReset(); }
}
