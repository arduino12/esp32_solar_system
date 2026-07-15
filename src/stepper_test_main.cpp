// stepper_test_main.cpp — standalone TMC2209 bench test rig (ESP32-C3 Super Mini).
// env: esp32c3_stepper_test   (NOT part of the show firmware)
//
// Purpose: dial in and validate the two wheel motors before trusting them in the installation.
// A browser page (Wi-Fi STA) drives both TMC2209 drivers in real time and shows live status:
//   - per motor: microsteps, RMS current, direction, target speed (rpm), acceleration,
//     deceleration, run/stop, and chopper mode (stealthChop = quiet / spreadCycle = normal);
//   - live driver status read back over UART: connected? version, actual microsteps, actual
//     current scale (cs_actual), StallGuard load, standstill, open-load A/B, short-to-gnd A/B,
//     over-temp / pre-warning, and the IFCNT write counter;
//   - UART write verification: after every "apply" we compare the driver's IFCNT delta and the
//     read-back microsteps against what we sent, so a dropped/garbled UART write is flagged
//     instead of silently ignored (the "it didn't set what we sent" problem).
//
// Stepping uses FastAccelStepper (hardware-timed STEP pulses via the C3 RMT). Both motors are
// driven from the SAME MCU clock, so they stay synchronized — which is why step/dir is the right
// choice for the coupled cart wheels (unlike each driver's independent VACTUAL internal clock).
// Direction is set over UART (TMC shaft bit) because the DIR pins are tied to GND in hardware.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>

// ----------------------------- Wi-Fi ----------------------------------------
static const char *WIFI_SSID = "YOUR_WIFI_SSID";
static const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ----------------------------- Pin map (matches the planet cart) -------------
static const uint8_t PIN_TMC_EN     = 2;   // TMC enable, active-low (LOW = drivers on)
static const uint8_t PIN_TMC_RX     = 4;   // Serial1 RX  <- driver PDN/UART bus
static const uint8_t PIN_TMC_TX     = 3;   // Serial1 TX  -> driver PDN/UART bus (via 1k)
static const uint8_t PIN_STEP_L     = 0;   // left  wheel STEP
static const uint8_t PIN_STEP_R     = 1;   // right wheel STEP
static const uint8_t PIN_MOTOR_PWR  = 5;   // high-side switch for VM (HIGH = supply on)
static const uint8_t PIN_STATUS_LED = 8;   // onboard LED, active-low

#define TMC_R_SENSE   0.11f
#define TMC_UART_BAUD 115200
#define TMC_ADDR_L    0b11
#define TMC_ADDR_R    0b10

HardwareSerial TMC_SERIAL(1);
TMC2209Stepper drvL(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_L);
TMC2209Stepper drvR(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_R);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepL = nullptr;
FastAccelStepper *stepR = nullptr;

AsyncWebServer server(80);

// ----------------------------- Motor state ----------------------------------
struct MotorCfg {
  uint8_t  microsteps = 16;
  uint16_t currentMa  = 200;
  float    stepDegree = 1.8f;
  float    accelRpmS  = 300.0f;   // acceleration in rpm per second
  float    decelRpmS  = 300.0f;   // deceleration in rpm per second
  bool     spread     = false;    // false = stealthChop (quiet), true = spreadCycle
  float    speedRpm   = 60.0f;    // target speed
  bool     dirCW      = true;     // requested direction
  bool     run        = false;    // should this motor be running
};
static MotorCfg g_cfg[2];         // [0]=L, [1]=R
static bool g_enabled = false;    // drivers powered/enabled

// Per-apply UART verification results.
struct WriteCheck {
  uint8_t writesIssued = 0;       // register writes we sent in the last apply
  uint8_t ifcntDelta   = 0;       // how many the driver actually counted (IFCNT)
  uint8_t ustepsWanted = 16;      // microsteps we asked for
  uint8_t ustepsReadback = 0;     // microsteps the driver reports now
  bool    checked      = false;
};
static WriteCheck g_wc[2];

// Deferred web actions handled in loop() (keep all stepper/UART access on the main task).
static volatile bool g_doApply[2]  = {false, false};
static volatile bool g_doEnable    = false, g_doDisable = false;

static inline TMC2209Stepper &drv(int i) { return i ? drvR : drvL; }
static inline FastAccelStepper *step(int i) { return i ? stepR : stepL; }

// ----------------------------- Motion helpers --------------------------------
static float rpmToHz(const MotorCfg &c, float rpm) {
  float fullStepsPerRev = 360.0f / (c.stepDegree > 0.01f ? c.stepDegree : 1.8f);
  return (rpm / 60.0f) * fullStepsPerRev * (float)c.microsteps;
}

// Apply current + microsteps + chopper mode to one driver, then verify over UART.
static void applyDriver(int i) {
  MotorCfg &c = g_cfg[i];
  TMC2209Stepper &d = drv(i);
  WriteCheck &w = g_wc[i];

  uint8_t before = d.IFCNT();
  d.rms_current(c.currentMa, 0.0f);     // 1
  d.microsteps(c.microsteps);           // 2
  d.intpol(c.microsteps >= 8);          // 3
  d.en_spreadCycle(c.spread);           // 4  (false = stealthChop = quiet)
  d.TPOWERDOWN(20);                     // 5
  d.freewheel(1);                       // 6
  uint8_t after = d.IFCNT();

  w.writesIssued   = 6;
  w.ifcntDelta     = (uint8_t)(after - before);   // wraps mod 256 — fine for 6 writes
  w.ustepsWanted   = c.microsteps;
  w.ustepsReadback = d.microsteps();              // read MRES back from CHOPCONF
  w.checked        = true;
  Serial.printf("apply M%d: sent %u writes, IFCNT+%u, usteps set=%u read=%u\n",
                i, w.writesIssued, w.ifcntDelta, w.ustepsWanted, w.ustepsReadback);
}

static void tmcBringup() {
  TMC_SERIAL.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  for (int i = 0; i < 2; i++) {
    TMC2209Stepper &d = drv(i);
    d.begin();
    d.toff(0);
    d.pdn_disable(true);
    d.I_scale_analog(false);
    d.mstep_reg_select(true);
    applyDriver(i);
  }
}

static void enableDrivers(bool on) {
  if (on) {
    digitalWrite(PIN_MOTOR_PWR, HIGH);
    delayMicroseconds(1500);
    drvL.toff(4); drvR.toff(4);
    digitalWrite(PIN_TMC_EN, LOW);
  } else {
    if (stepL) stepL->forceStop();
    if (stepR) stepR->forceStop();
    digitalWrite(PIN_TMC_EN, HIGH);
    drvL.toff(0); drvR.toff(0);
    digitalWrite(PIN_MOTOR_PWR, LOW);
  }
  g_enabled = on;
}

// Start or stop one motor per its cfg. Direction via TMC shaft bit (DIR pin tied to GND).
static void driveMotor(int i) {
  MotorCfg &c = g_cfg[i];
  FastAccelStepper *st = step(i);
  if (!st) return;
  if (c.run && g_enabled) {
    drv(i).shaft(!c.dirCW);                       // shaft bit selects direction
    uint32_t hz    = (uint32_t)rpmToHz(c, c.speedRpm);
    uint32_t accel = (uint32_t)rpmToHz(c, c.accelRpmS);
    st->setSpeedInHz(hz > 0 ? hz : 1);
    st->setAcceleration(accel > 0 ? accel : 1);
    st->runForward();
  } else {
    uint32_t decel = (uint32_t)rpmToHz(c, c.decelRpmS);
    st->setAcceleration(decel > 0 ? decel : 1);   // use the decel rate for the stop ramp
    st->stopMove();
  }
}

// ----------------------------- Status JSON -----------------------------------
static void buildStatus(String &out) {
  JsonDocument d;
  d["enabled"] = g_enabled;
  JsonArray ms = d["motors"].to<JsonArray>();
  JsonArray tm = d["tmc"].to<JsonArray>();
  const char *ids[2] = {"L", "R"};
  for (int i = 0; i < 2; i++) {
    MotorCfg &c = g_cfg[i];
    FastAccelStepper *st = step(i);
    TMC2209Stepper &dd = drv(i);
    WriteCheck &w = g_wc[i];

    JsonObject m = ms.add<JsonObject>();
    m["id"]       = ids[i];
    m["running"]  = st && st->isRunning();
    m["cmd_rpm"]  = c.run ? (c.dirCW ? c.speedRpm : -c.speedRpm) : 0.0f;
    float fullSteps = 360.0f / (c.stepDegree > 0.01f ? c.stepDegree : 1.8f);
    float curHz = st ? (float)st->getCurrentSpeedInMilliHz() / 1000.0f : 0.0f;
    m["cur_rpm"]  = curHz / (fullSteps * c.microsteps) * 60.0f;
    m["pos"]      = st ? st->getCurrentPosition() : 0;

    // Config echo (what we believe is set).
    JsonObject cf = m["cfg"].to<JsonObject>();
    cf["microsteps"] = c.microsteps;
    cf["current_ma"] = c.currentMa;
    cf["step_degree"]= c.stepDegree;
    cf["accel"]      = c.accelRpmS;
    cf["decel"]      = c.decelRpmS;
    cf["mode"]       = c.spread ? "spread" : "stealth";
    cf["dir"]        = c.dirCW ? "CW" : "CCW";
    cf["run"]        = c.run;

    // Live driver read-back over UART.
    JsonObject t = tm.add<JsonObject>();
    t["id"]      = ids[i];
    uint8_t ver  = dd.version();
    t["version"] = ver;
    t["conn"]    = (dd.test_connection() == 0);   // 0 = OK
    t["usteps_rb"] = dd.microsteps();
    uint16_t cs  = dd.cs_actual();
    t["cs_actual"] = cs;
    t["sg"]      = dd.SG_RESULT();
    t["stst"]    = dd.stst();
    t["stealth"] = dd.stealth();
    t["ola"]     = dd.ola();
    t["olb"]     = dd.olb();
    t["s2ga"]    = dd.s2ga();
    t["s2gb"]    = dd.s2gb();
    t["ot"]      = dd.ot();
    t["otpw"]    = dd.otpw();
    t["ifcnt"]   = dd.IFCNT();
    // Write verification from the last apply.
    t["wr_sent"] = w.writesIssued;
    t["wr_ack"]  = w.ifcntDelta;
    t["us_want"] = w.ustepsWanted;
    t["us_read"] = w.ustepsReadback;
    t["wr_ok"]   = w.checked && (w.ifcntDelta >= w.writesIssued) &&
                   (w.ustepsReadback == w.ustepsWanted);
  }
  serializeJson(d, out);
}

// ----------------------------- Web page --------------------------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>TMC2209 Bench Test</title>
<style>
 body{font-family:system-ui,sans-serif;background:#14161a;color:#e6e6e6;margin:0;padding:14px}
 h1{font-size:18px;margin:0 0 10px} h2{font-size:15px;margin:14px 0 6px;color:#8fd3ff}
 .card{background:#1d2026;border:1px solid #2c303a;border-radius:8px;padding:12px;margin-bottom:12px}
 .row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:6px 0}
 label{font-size:13px;color:#b6b6b6;min-width:96px}
 input,select{background:#0f1114;color:#e6e6e6;border:1px solid #3a3f4b;border-radius:6px;padding:6px;font-size:14px}
 input{width:88px}
 button{background:#2a6ef0;color:#fff;border:0;border-radius:6px;padding:8px 14px;font-size:14px;cursor:pointer}
 button.g{background:#1f9d55} button.r{background:#d64545} button.s{background:#555}
 .mcol{display:flex;gap:12px;flex-wrap:wrap} .m{flex:1;min-width:260px}
 table{border-collapse:collapse;width:100%;font-size:13px} td{padding:3px 8px;border-bottom:1px solid #2c303a}
 td:first-child{color:#9aa0aa} .ok{color:#57d98a} .bad{color:#ff6b6b} .warn{color:#ffcf5c}
 .big{font-size:20px;font-weight:700}
</style></head><body>
<h1>TMC2209 Bench Test <span id=ip style="font-size:12px;color:#888"></span></h1>

<div class=card>
 <div class=row>
   <button class=g onclick="en(1)">Enable drivers</button>
   <button class=s onclick="en(0)">Disable</button>
   <span id=enst></span>
 </div>
</div>

<div class=card>
 <h2>Shared parameters</h2>
 <div class=row><label>Microsteps</label>
   <select id=microsteps><option>1<option>2<option>4<option>8<option selected>16<option>32<option>64<option>128<option>256</select>
   <label>Current (mA)</label><input id=current_ma type=number value=200>
   <label>Step angle°</label><input id=step_degree type=number step=0.1 value=1.8>
 </div>
 <div class=row><label>Accel (rpm/s)</label><input id=accel type=number value=300>
   <label>Decel (rpm/s)</label><input id=decel type=number value=300>
   <label>Chopper</label><select id=mode><option value=stealth selected>stealthChop (quiet)</option><option value=spread>spreadCycle (normal)</option></select>
 </div>
</div>

<div class="card mcol">
 <div class=m>
   <h2>Motor L</h2>
   <div class=row><label>Speed (rpm)</label><input id=Lspeed type=number value=60>
     <label>Dir</label><select id=Ldir><option value=CW selected>CW</option><option value=CCW>CCW</option></select></div>
   <div class=row><button class=g onclick="run('L',1)">Run L</button><button class=r onclick="run('L',0)">Stop L</button></div>
 </div>
 <div class=m>
   <h2>Motor R</h2>
   <div class=row><label>Speed (rpm)</label><input id=Rspeed type=number value=60>
     <label>Dir</label><select id=Rdir><option value=CW selected>CW</option><option value=CCW>CCW</option></select></div>
   <div class=row><button class=g onclick="run('R',1)">Run R</button><button class=r onclick="run('R',0)">Stop R</button></div>
 </div>
</div>
<div class=row><button onclick="applyAll()">Apply parameters to both</button>
  <button class=g onclick="run('both',1)">Run both (sync)</button>
  <button class=r onclick="run('both',0)">Stop both</button></div>

<div class="card mcol">
 <div class=m><h2>Status L</h2><table id=stL></table></div>
 <div class=m><h2>Status R</h2><table id=stR></table></div>
</div>

<script>
const $=id=>document.getElementById(id);
function cfg(){return{microsteps:+$('microsteps').value,current_ma:+$('current_ma').value,
  step_degree:+$('step_degree').value,accel:+$('accel').value,decel:+$('decel').value,mode:$('mode').value};}
function motor(m){return{speed:+$(m+'speed').value,dir:$(m+'dir').value};}
function post(p,o){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});}
function en(on){post('/api/enable',{on:!!on});}
function applyAll(){post('/api/apply',{cfg:cfg(),L:motor('L'),R:motor('R')});}
function run(which,on){post('/api/run',{cfg:cfg(),L:motor('L'),R:motor('R'),which:which,run:!!on});}
function b(v){return v?'<span class=ok>yes</span>':'<span class=bad>no</span>';}
function flag(v){return v?'<span class=warn>SET</span>':'<span class=ok>-</span>';}
function row(k,v){return '<tr><td>'+k+'</td><td>'+v+'</td></tr>';}
function fill(el,m,t){
  let h='';
  h+=row('running',b(m.running));
  h+=row('cmd rpm','<span class=big>'+m.cmd_rpm.toFixed(1)+'</span>');
  h+=row('actual rpm','<span class=big>'+m.cur_rpm.toFixed(1)+'</span>');
  h+=row('position',m.pos);
  h+=row('connected',t.conn?'<span class=ok>OK</span>':'<span class=bad>DISCONNECTED</span>');
  h+=row('version','0x'+t.version.toString(16)+(t.version==0x21?' <span class=ok>(TMC2209)</span>':' <span class=bad>?</span>'));
  h+=row('microsteps (read)',t.usteps_rb);
  h+=row('cs_actual',t.cs_actual);
  h+=row('StallGuard',t.sg+(m.running&&t.sg==0?' <span class=bad>(stuck?)</span>':''));
  h+=row('standstill',b(t.stst));
  h+=row('mode',t.stealth?'stealthChop':'spreadCycle');
  h+=row('open load A/B',flag(t.ola)+' / '+flag(t.olb));
  h+=row('short GND A/B',flag(t.s2ga)+' / '+flag(t.s2gb));
  h+=row('over-temp / pre-warn',flag(t.ot)+' / '+flag(t.otpw));
  h+=row('IFCNT',t.ifcnt);
  let wok=t.wr_ok?'<span class=ok>OK</span>':'<span class=bad>CHECK</span>';
  h+=row('last write',wok+' (sent '+t.wr_sent+', ack '+t.wr_ack+'; usteps want '+t.us_want+' read '+t.us_read+')');
  el.innerHTML=h;
}
async function poll(){
  try{let r=await fetch('/api/status');let d=await r.json();
    $('enst').innerHTML=d.enabled?'<span class=ok>drivers ENABLED</span>':'<span class=bad>disabled</span>';
    fill($('stL'),d.motors[0],d.tmc[0]);fill($('stR'),d.motors[1],d.tmc[1]);
  }catch(e){}
  setTimeout(poll,400);
}
poll();
</script>
</body></html>
)HTML";

// ----------------------------- Apply web payloads ----------------------------
static void ingest(JsonVariant &j) {
  JsonObject c = j["cfg"];
  if (!c.isNull()) {
    for (int i = 0; i < 2; i++) {
      g_cfg[i].microsteps = c["microsteps"] | g_cfg[i].microsteps;
      g_cfg[i].currentMa  = c["current_ma"] | g_cfg[i].currentMa;
      g_cfg[i].stepDegree = c["step_degree"]| g_cfg[i].stepDegree;
      g_cfg[i].accelRpmS  = c["accel"]      | g_cfg[i].accelRpmS;
      g_cfg[i].decelRpmS  = c["decel"]      | g_cfg[i].decelRpmS;
      g_cfg[i].spread     = String((const char*)(c["mode"] | "stealth")) == "spread";
    }
  }
  JsonObject L = j["L"], R = j["R"];
  if (!L.isNull()) { g_cfg[0].speedRpm = L["speed"] | g_cfg[0].speedRpm;
                     g_cfg[0].dirCW = String((const char*)(L["dir"]|"CW")) == "CW"; }
  if (!R.isNull()) { g_cfg[1].speedRpm = R["speed"] | g_cfg[1].speedRpm;
                     g_cfg[1].dirCW = String((const char*)(R["dir"]|"CW")) == "CW"; }
}

static void setupWeb() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "text/html", String(FPSTR(INDEX_HTML)));
  });
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *r) {
    String s; buildStatus(s); r->send(200, "application/json", s);
  });
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/enable",
    [](AsyncWebServerRequest *r, JsonVariant &j) {
      if (j["on"] | false) g_doEnable = true; else g_doDisable = true;
      r->send(200, "application/json", "{\"ok\":true}");
    }));
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/apply",
    [](AsyncWebServerRequest *r, JsonVariant &j) {
      ingest(j); g_doApply[0] = g_doApply[1] = true;
      r->send(200, "application/json", "{\"ok\":true}");
    }));
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/run",
    [](AsyncWebServerRequest *r, JsonVariant &j) {
      ingest(j);
      String which = j["which"] | "both";
      bool on = j["run"] | false;
      if (which == "L" || which == "both") g_cfg[0].run = on;
      if (which == "R" || which == "both") g_cfg[1].run = on;
      g_doApply[0] = g_doApply[1] = true;    // (re)apply params then (re)issue motion in loop()
      r->send(200, "application/json", "{\"ok\":true}");
    }));
  server.begin();
}

// ----------------------------- Setup / loop ----------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  delay(200);
  Serial.println("\n==== TMC2209 Bench Test ====");

  pinMode(PIN_MOTOR_PWR, OUTPUT);  digitalWrite(PIN_MOTOR_PWR, LOW);
  pinMode(PIN_TMC_EN, OUTPUT);     digitalWrite(PIN_TMC_EN, HIGH);
  pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, HIGH);

  tmcBringup();

  engine.init();
  stepL = engine.stepperConnectToPin(PIN_STEP_L);
  stepR = engine.stepperConnectToPin(PIN_STEP_R);
  if (stepL) stepL->setAutoEnable(false);
  if (stepR) stepR->setAutoEnable(false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  uint32_t tw = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - tw < 15000) { delay(300); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\nConnected. Open http://%s/\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\nWi-Fi not connected; will keep retrying.");

  setupWeb();
}

void loop() {
  // Blink: solid-ish when connected, slow when not.
  uint32_t period = (WiFi.status() == WL_CONNECTED) ? 1500 : 400;
  digitalWrite(PIN_STATUS_LED, (millis() % period) < 80 ? LOW : HIGH);

  if (g_doEnable)  { g_doEnable  = false; enableDrivers(true); }
  if (g_doDisable) { g_doDisable = false; enableDrivers(false); }
  for (int i = 0; i < 2; i++) {
    if (g_doApply[i]) {
      g_doApply[i] = false;
      applyDriver(i);
      driveMotor(i);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 8000) { lastTry = millis(); WiFi.reconnect(); }
  }

  // Periodic serial heartbeat so a bench monitor can see link state + both motors at a glance.
  static uint32_t lastDbg = 0;
  if (millis() - lastDbg > 3000) {
    lastDbg = millis();
    bool up = WiFi.status() == WL_CONNECTED;
    Serial.printf("[%lus] wifi=%s%s en=%d | ", (unsigned long)(millis()/1000),
                  up ? "OK " : "..", up ? WiFi.localIP().toString().c_str() : "", g_enabled);
    for (int i = 0; i < 2; i++) {
      FastAccelStepper *st = step(i);
      TMC2209Stepper &dd = drv(i);
      Serial.printf("M%c[%s run=%d us=%u cs=%u sg=%u ifc=%u wr:%s] ",
                    i ? 'R' : 'L', dd.test_connection() == 0 ? "conn" : "DISC",
                    st && st->isRunning(), dd.microsteps(), dd.cs_actual(),
                    dd.SG_RESULT(), dd.IFCNT(),
                    g_wc[i].checked && g_wc[i].ustepsReadback == g_wc[i].ustepsWanted ? "ok" : "?");
    }
    Serial.println();
  }
}
