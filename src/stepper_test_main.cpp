// stepper_test_main.cpp — standalone TMC2209 bench test rig (ESP32-C3 Super Mini).
// env: esp32c3_stepper_test   (NOT part of the show firmware)
//
// Purpose: dial in and validate the two wheel motors. A browser page (Wi-Fi STA) drives both
// TMC2209 drivers in real time and shows live status read back over UART.
//
// STEP GENERATION: the ESP32-C3 LEDC peripheral. LEDC emits a hardware square wave at a set
// frequency on a GPIO — that square wave IS the STEP signal. No CPU bit-banging, no RMT (which
// wedged FastAccelStepper on this MCU), and both channels derive from the same 80 MHz APB clock,
// so the two wheels' frequency ratio is exact and drift-free (true synchronization — unlike each
// TMC's independent VACTUAL oscillator). Speed = frequency; we ramp the frequency in software
// (coarse 20 ms updates, NOT per step) for smooth accel/decel. Direction = TMC shaft bit over
// UART (DIR pins are tied to GND in hardware).
//
// UART reliability: every microsteps write is verified by reading the register back and retrying
// on mismatch (write -> read-back -> retry), and IFCNT (the driver's own received-write counter)
// is shown. Current and direction are also read back from the driver, not just echoed.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
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
static const uint8_t PIN_STEP_L     = 0;   // left  wheel STEP  (LEDC ch 0)
static const uint8_t PIN_STEP_R     = 1;   // right wheel STEP  (LEDC ch 1)
static const uint8_t PIN_MOTOR_PWR  = 5;   // high-side switch for VM (HIGH = supply on)
static const uint8_t PIN_STATUS_LED = 8;   // onboard LED, active-low

// LEDC channels are paired to timers as timer=(chan/2), and one timer has ONE frequency. So the
// two motors MUST sit on channels of different timers to run at independent speeds: ch0 -> timer0,
// ch2 -> timer1. Both timers still derive from the single 80 MHz clock, so the frequency ratio
// stays exact (synchronized). (Using ch0+ch1 forces both motors to the same frequency.)
#define LEDC_CH_L     0
#define LEDC_CH_R     2
#define LEDC_CLK_HZ   80000000.0   // APB clock feeding LEDC

#define TMC_R_SENSE   0.11f
#define TMC_UART_BAUD 115200
#define TMC_ADDR_L    0b11
#define TMC_ADDR_R    0b10
#define RAMP_UPDATE_MS 20          // how often the software frequency ramp advances

HardwareSerial TMC_SERIAL(1);
TMC2209Stepper drvL(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_L);
TMC2209Stepper drvR(&TMC_SERIAL, TMC_R_SENSE, TMC_ADDR_R);

AsyncWebServer server(80);

// ----------------------------- Motor state ----------------------------------
struct MotorCfg {
  uint16_t microsteps = 16;       // 1..256 (1 = full step)
  uint16_t currentMa  = 200;      // IRUN (running) RMS current
  uint8_t  holdPct    = 50;       // IHOLD as % of run current at standstill (0 = release/freewheel)
  float    stepDegree = 1.8f;
  float    accelRpmS  = 300.0f;   // acceleration in rpm per second
  float    decelRpmS  = 300.0f;   // deceleration in rpm per second
  bool     spread     = false;    // false = stealthChop (quiet), true = spreadCycle
  bool     interpolate = true;    // MicroPlyer: interpolate microsteps up to 256 internally
  float    speedRpm   = 60.0f;    // target speed
  bool     dirCW      = true;     // requested direction
  bool     run        = false;    // should this motor be running
};
static MotorCfg g_cfg[2];         // [0]=L, [1]=R
static bool  g_enabled = false;   // drivers powered/enabled
static float g_curHz[2] = {0, 0}; // ramp's current commanded step rate per motor
static float g_tgtHz[2] = {0, 0}; // target step rate per motor
static float g_actHz[2] = {0, 0}; // actual frequency LEDC achieved (read back from the hardware)
static uint32_t g_rampMs = 0;

// Per-apply UART verification results.
struct WriteCheck {
  uint8_t  ifcntDelta   = 0;      // IFCNT increment during the apply (driver-acked writes)
  uint16_t ustepsWanted = 16;     // microsteps we asked for
  uint16_t ustepsRead   = 0;      // microsteps read back from the driver
  uint8_t  usTries      = 0;      // write attempts needed until read-back matched (1 = first try)
  bool     checked      = false;
};
static WriteCheck g_wc[2];

// Deferred web actions handled in loop() (keep all UART/LEDC access on the main task).
static volatile bool g_doApply[2]  = {false, false};
static volatile bool g_doEnable    = false, g_doDisable = false;

static inline TMC2209Stepper &drv(int i) { return i ? drvR : drvL; }
static inline int ledcCh(int i) { return i ? LEDC_CH_R : LEDC_CH_L; }

// ----------------------------- Helpers ---------------------------------------
static float rpmToHz(const MotorCfg &c, float rpm) {
  float fullStepsPerRev = 360.0f / (c.stepDegree > 0.01f ? c.stepDegree : 1.8f);
  return (rpm / 60.0f) * fullStepsPerRev * (float)c.microsteps;
}
static float hzToRpm(const MotorCfg &c, float hz) {
  float fullStepsPerRev = 360.0f / (c.stepDegree > 0.01f ? c.stepDegree : 1.8f);
  return hz / (fullStepsPerRev * c.microsteps) * 60.0f;
}

// Write microsteps and VERIFY over UART: the library encodes full-step (1) as register value 0,
// and has no case for 1, so we map 1->0 on write; read the register back (0 means 1) and retry up
// to 3x if it didn't take (handles the occasional dropped UART write).
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

static void applyDriver(int i) {
  MotorCfg &c = g_cfg[i];
  TMC2209Stepper &d = drv(i);
  WriteCheck &w = g_wc[i];

  uint8_t before = d.IFCNT();
  d.rms_current(c.currentMa, c.holdPct / 100.0f);   // IRUN, and IHOLD = holdPct% of it
  w.ustepsRead   = setMicrostepsVerified(d, c.microsteps, w.usTries);
  d.intpol(c.interpolate);              // MicroPlyer: smooth 256-microstep output from low usteps
  d.en_spreadCycle(c.spread);
  d.iholddelay(4);                      // gentle IRUN->IHOLD ramp when the motor stops
  d.TPOWERDOWN(20);                     // ~0.3 s at standstill before dropping to IHOLD
  d.freewheel(c.holdPct > 0 ? 0 : 1);   // holdPct>0: keep hold torque; 0: release coils (freewheel)
  uint8_t after = d.IFCNT();

  w.ifcntDelta   = (uint8_t)(after - before);
  w.ustepsWanted = c.microsteps;
  w.checked      = true;
  Serial.printf("apply M%d: usteps want=%u read=%u (tries %u), IFCNT+%u, I=%umA mode=%s microplyer=%d\n",
                i, w.ustepsWanted, w.ustepsRead, w.usTries, w.ifcntDelta, c.currentMa,
                c.spread ? "spread" : "stealth", c.interpolate);
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
    d.pwm_autoscale(true);      // StealthChop auto current regulation (needed for open-load sensing)
    d.pwm_autograd(true);
    applyDriver(i);
  }
}

static void ledcStop(int i) { ledcWrite(ledcCh(i), 0); g_actHz[i] = 0; }  // duty 0 -> STEP low

// Emit a hardware square wave at f Hz on the motor's STEP pin, choosing the LEDC resolution so the
// clock divider is well-conditioned (small resolution -> large divider -> fine fractional-divider
// granularity -> accurate frequency, unlike ledcWriteTone's fixed 10-bit that mis-divides fast
// tones). Records the ACTUAL achieved frequency (ledcSetup returns it) for honest status display.
static void stepEmit(int i, float f) {
  int ch = ledcCh(i);
  if (f < 1.0f) { ledcStop(i); return; }
  int res = 1;
  while (res < 14 && (LEDC_CLK_HZ / (f * (double)(1u << res))) > 1023.0) res++;  // keep int div <=1023
  uint32_t act = ledcSetup(ch, (uint32_t)(f + 0.5f), res);   // reconfigures the timer; returns Hz
  ledcWrite(ch, (1u << res) / 2);                            // 50% duty at this resolution
  g_actHz[i] = act ? (float)act : f;
}

static void enableDrivers(bool on) {
  if (on) {
    digitalWrite(PIN_MOTOR_PWR, HIGH);
    delayMicroseconds(1500);
    drvL.toff(4); drvR.toff(4);
    digitalWrite(PIN_TMC_EN, LOW);
  } else {
    ledcStop(0); ledcStop(1);
    g_curHz[0] = g_curHz[1] = g_tgtHz[0] = g_tgtHz[1] = 0;
    g_actHz[0] = g_actHz[1] = 0;
    digitalWrite(PIN_TMC_EN, HIGH);
    drvL.toff(0); drvR.toff(0);
    digitalWrite(PIN_MOTOR_PWR, LOW);
  }
  g_enabled = on;
}

// Set target frequency + direction for one motor from its cfg. Direction via TMC shaft bit.
static void driveMotor(int i) {
  MotorCfg &c = g_cfg[i];
  drv(i).shaft(!c.dirCW);                          // shaft bit selects direction
  g_tgtHz[i] = (c.run && g_enabled) ? rpmToHz(c, c.speedRpm) : 0.0f;
}

// Software frequency ramp: nudge each LEDC channel toward its target and emit the tone. Accel rate
// while speeding up, decel rate while slowing. The LEDC hardware keeps producing the square wave
// between updates, so a busy CPU/Wi-Fi never interrupts the step train.
static void serviceRamp() {
  uint32_t now = millis();
  if (now - g_rampMs < RAMP_UPDATE_MS) return;
  float dt = (now - g_rampMs) / 1000.0f;
  g_rampMs = now;

  for (int i = 0; i < 2; i++) {
    MotorCfg &c = g_cfg[i];
    float tgt = g_tgtHz[i], cur = g_curHz[i];
    bool speeding = tgt > cur;
    float rate = rpmToHz(c, speeding ? c.accelRpmS : c.decelRpmS);   // Hz per second
    float stepv = fmaxf(rate, 1.0f) * dt;
    if (cur < tgt) { cur += stepv; if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur -= stepv; if (cur < tgt) cur = tgt; }
    g_curHz[i] = cur;

    if (g_enabled) stepEmit(i, cur);
    else           g_actHz[i] = 0;
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
    TMC2209Stepper &dd = drv(i);
    WriteCheck &w = g_wc[i];

    JsonObject m = ms.add<JsonObject>();
    m["id"]      = ids[i];
    m["running"] = g_actHz[i] > 1.0f;
    m["cmd_rpm"] = c.run ? (c.dirCW ? c.speedRpm : -c.speedRpm) : 0.0f;
    m["cur_rpm"] = hzToRpm(c, g_actHz[i]) * (c.dirCW ? 1 : -1);   // from the ACTUAL LEDC frequency
    m["cur_hz"]  = g_actHz[i];

    JsonObject cf = m["cfg"].to<JsonObject>();
    cf["microsteps"] = c.microsteps;
    cf["current_ma"] = c.currentMa;
    cf["hold_pct"]   = c.holdPct;
    cf["step_degree"]= c.stepDegree;
    cf["accel"]      = c.accelRpmS;
    cf["decel"]      = c.decelRpmS;
    cf["mode"]       = c.spread ? "spread" : "stealth";
    cf["microplyer"] = c.interpolate;
    cf["dir"]        = c.dirCW ? "CW" : "CCW";
    cf["run"]        = c.run;

    // Live driver read-back over UART — what the chip actually reports right now.
    JsonObject t = tm.add<JsonObject>();
    t["id"]      = ids[i];
    uint8_t ver  = dd.version();
    t["version"] = ver;
    t["conn"]    = (dd.test_connection() == 0);          // 0 = OK
    uint16_t usRb = dd.microsteps(); if (usRb == 0) usRb = 1;
    t["usteps_rb"] = usRb;                               // microsteps read back (0 -> 1)
    bool shaftRb = dd.shaft();
    t["dir_rb"]  = shaftRb ? "CCW" : "CW";               // direction read back from GCONF
    uint16_t cs  = dd.cs_actual();
    t["cs_actual"] = cs;
    // Open-load / disconnected coil. cs_actual is only the SETPOINT scale (identical whether or not
    // a coil is connected), so it can't tell us if current actually flows. Instead watch the
    // StealthChop PWM autoscaler: with an open winding it can't reach the target current, so
    // PWM_SCALE_SUM pins at its 255 ceiling AND PWM_SCALE_AUTO rails high. In SpreadCycle use OLA/OLB.
    uint8_t ps  = dd.pwm_scale_sum();
    int16_t pa  = dd.pwm_scale_auto();
    bool moving = g_actHz[i] > 1.0f;
    bool openLoad = moving && (c.spread ? (dd.ola() || dd.olb())
                                        : (ps >= 250 && pa >= 100));
    t["pwm_sum"]  = ps;
    t["pwm_auto"] = pa;
    t["open"]     = openLoad;
    // Delivered current: an open coil carries ~0 mA regardless of the commanded scale.
    t["irms_rb"] = openLoad ? 0 : dd.cs2rms(cs);
    t["sg"]      = dd.SG_RESULT();
    t["stst"]    = dd.stst();
    t["stealth"] = dd.stealth();
    t["intpol"]  = dd.intpol();          // MicroPlyer state read back from CHOPCONF
    t["ola"]     = dd.ola();
    t["olb"]     = dd.olb();
    t["s2ga"]    = dd.s2ga();
    t["s2gb"]    = dd.s2gb();
    t["ot"]      = dd.ot();
    t["otpw"]    = dd.otpw();
    t["ifcnt"]   = dd.IFCNT();
    // Write verification (microsteps) from the last apply.
    t["us_want"] = w.ustepsWanted;
    t["us_read"] = w.ustepsRead;
    t["us_tries"]= w.usTries;
    t["ifc_d"]   = w.ifcntDelta;
    t["wr_ok"]   = w.checked && (w.ustepsRead == w.ustepsWanted) && (w.ifcntDelta > 0);
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
 .mcol{display:flex;gap:12px;flex-wrap:wrap} .m{flex:1;min-width:270px}
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
   <label>Hold (%)</label><input id=hold_pct type=number value=50 title="IHOLD as % of run current at standstill; 0 = release/freewheel">
   <label>Step angle°</label><input id=step_degree type=number step=0.1 value=1.8>
 </div>
 <div class=row><label>Accel (rpm/s)</label><input id=accel type=number value=300>
   <label>Decel (rpm/s)</label><input id=decel type=number value=300>
   <label>Chopper</label><select id=mode><option value=stealth selected>stealthChop (quiet)</option><option value=spread>spreadCycle (normal)</option></select>
   <label>MicroPlyer</label><select id=microplyer><option value=1 selected>on (interp. to 256)</option><option value=0>off</option></select>
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
<div class=row><button onclick="applyAll()">Apply parameters</button>
  <button class=g onclick="run('both',1)">Run both (sync)</button>
  <button class=r onclick="run('both',0)">Stop both</button></div>

<div class="card mcol">
 <div class=m><h2>Status L</h2><table id=stL></table></div>
 <div class=m><h2>Status R</h2><table id=stR></table></div>
</div>

<script>
const $=id=>document.getElementById(id);
function cfg(){return{microsteps:+$('microsteps').value,current_ma:+$('current_ma').value,
  hold_pct:+$('hold_pct').value,step_degree:+$('step_degree').value,accel:+$('accel').value,
  decel:+$('decel').value,mode:$('mode').value,microplyer:$('microplyer').value=='1'};}
function motor(m){return{speed:+$(m+'speed').value,dir:$(m+'dir').value};}
function post(p,o){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});}
function en(on){post('/api/enable',{on:!!on});}
function applyAll(){post('/api/apply',{cfg:cfg(),L:motor('L'),R:motor('R')});}
function run(which,on){post('/api/run',{cfg:cfg(),L:motor('L'),R:motor('R'),which:which,run:!!on});}
function b(v){return v?'<span class=ok>yes</span>':'<span class=bad>no</span>';}
function flag(v){return v?'<span class=warn>SET</span>':'<span class=ok>-</span>';}
function row(k,v){return '<tr><td>'+k+'</td><td>'+v+'</td></tr>';}
function chk(want,read){return read==want?'<span class=ok>'+read+'</span>':'<span class=bad>'+read+' (want '+want+')</span>';}
function fill(el,m,t){
  let h='';
  h+=row('running',b(m.running));
  h+=row('cmd rpm','<span class=big>'+m.cmd_rpm.toFixed(1)+'</span>');
  h+=row('actual rpm','<span class=big>'+m.cur_rpm.toFixed(1)+'</span> <span style="color:#888">('+m.cur_hz.toFixed(0)+' Hz</span>)');
  h+=row('connected',t.conn?'<span class=ok>OK</span>':'<span class=bad>DISCONNECTED</span>');
  h+=row('version','0x'+t.version.toString(16)+(t.version==0x21?' <span class=ok>(TMC2209)</span>':' <span class=bad>?</span>'));
  h+=row('microsteps (set/read)',chk(m.cfg.microsteps,t.usteps_rb));
  h+=row('direction (set/read)',chk(m.cfg.dir,t.dir_rb));
  h+=row('current set / delivered',m.cfg.current_ma+' mA (hold '+m.cfg.hold_pct+'%) / <b>'+t.irms_rb+' mA</b>'+(t.open?' <span class=bad>(open load!)</span>':''));
  h+=row('coil',t.open?'<span class=bad>OPEN LOAD / DISCONNECTED</span>':'<span class=ok>connected</span>')+
     '<tr><td>PWM scale sum/auto</td><td><span style="color:#888">'+t.pwm_sum+' / '+t.pwm_auto+'</span></td></tr>';
  h+=row('StallGuard',t.sg+(m.running&&t.sg==0?' <span class=bad>(stuck?)</span>':''));
  h+=row('standstill',b(t.stst));
  h+=row('mode',t.stealth?'stealthChop':'spreadCycle');
  h+=row('MicroPlyer (set/read)',chk(m.cfg.microplyer?'on':'off',t.intpol?'on':'off'));
  h+=row('open load A/B',flag(t.ola)+' / '+flag(t.olb));
  h+=row('short GND A/B',flag(t.s2ga)+' / '+flag(t.s2gb));
  h+=row('over-temp / pre-warn',flag(t.ot)+' / '+flag(t.otpw));
  h+=row('IFCNT',t.ifcnt);
  let wok=t.wr_ok?'<span class=ok>OK</span>':'<span class=bad>CHECK</span>';
  h+=row('last write',wok+' (usteps want '+t.us_want+' read '+t.us_read+', tries '+t.us_tries+', IFCNT +'+t.ifc_d+')');
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
      g_cfg[i].microsteps = c["microsteps"] | g_cfg[i].microsteps;   // uint16 -> 256 fits
      g_cfg[i].currentMa  = c["current_ma"] | g_cfg[i].currentMa;
      g_cfg[i].holdPct    = c["hold_pct"]   | g_cfg[i].holdPct;
      g_cfg[i].stepDegree = c["step_degree"]| g_cfg[i].stepDegree;
      g_cfg[i].accelRpmS  = c["accel"]      | g_cfg[i].accelRpmS;
      g_cfg[i].decelRpmS  = c["decel"]      | g_cfg[i].decelRpmS;
      g_cfg[i].spread     = String((const char*)(c["mode"] | "stealth")) == "spread";
      if (!c["microplyer"].isNull()) g_cfg[i].interpolate = c["microplyer"].as<bool>();
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
      g_doApply[0] = g_doApply[1] = true;    // (re)apply params, then update motion in loop()
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
  Serial.println("\n==== TMC2209 Bench Test (LEDC step gen) ====");

  pinMode(PIN_MOTOR_PWR, OUTPUT);  digitalWrite(PIN_MOTOR_PWR, LOW);
  pinMode(PIN_TMC_EN, OUTPUT);     digitalWrite(PIN_TMC_EN, HIGH);
  pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, HIGH);

  // LEDC: attach each STEP pin to a channel on its own timer (ch0/timer0, ch2/timer1). stepEmit()
  // sets the frequency (= step rate) with a per-frequency resolution for accuracy.
  ledcSetup(LEDC_CH_L, 1000, 8); ledcAttachPin(PIN_STEP_L, LEDC_CH_L); ledcWrite(LEDC_CH_L, 0);
  ledcSetup(LEDC_CH_R, 1000, 8); ledcAttachPin(PIN_STEP_R, LEDC_CH_R); ledcWrite(LEDC_CH_R, 0);

  tmcBringup();

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
  g_rampMs = millis();
}

void loop() {
  uint32_t period = (WiFi.status() == WL_CONNECTED) ? 1500 : 400;
  digitalWrite(PIN_STATUS_LED, (millis() % period) < 80 ? LOW : HIGH);

  if (g_doEnable)  { g_doEnable  = false; enableDrivers(true); }
  if (g_doDisable) { g_doDisable = false; enableDrivers(false); }
  for (int i = 0; i < 2; i++) {
    if (g_doApply[i]) { g_doApply[i] = false; applyDriver(i); driveMotor(i); }
  }
  serviceRamp();

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 8000) { lastTry = millis(); WiFi.reconnect(); }
  }

  static uint32_t lastDbg = 0;
  if (millis() - lastDbg > 3000) {
    lastDbg = millis();
    bool up = WiFi.status() == WL_CONNECTED;
    Serial.printf("[%lus] wifi=%s%s en=%d | ", (unsigned long)(millis()/1000),
                  up ? "OK " : "..", up ? WiFi.localIP().toString().c_str() : "", g_enabled);
    for (int i = 0; i < 2; i++) {
      TMC2209Stepper &dd = drv(i);
      uint16_t usRb = dd.microsteps(); if (usRb == 0) usRb = 1;
      Serial.printf("M%c[%s hz=%.0f us=%u mp=%d dir=%s I=%umA sg=%u ifc=%u wr:%s] ",
                    i ? 'R' : 'L', dd.test_connection() == 0 ? "conn" : "DISC",
                    g_actHz[i], usRb, dd.intpol(), dd.shaft() ? "CCW" : "CW", dd.cs2rms(dd.cs_actual()),
                    dd.SG_RESULT(), dd.IFCNT(),
                    g_wc[i].checked && g_wc[i].ustepsRead == g_wc[i].ustepsWanted ? "ok" : "?");
    }
    Serial.println();
  }
}
