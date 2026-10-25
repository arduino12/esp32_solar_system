// sun_web_ui.h — single-page web UI for the Sun controller, stored in flash (PROGMEM) and
// served at "/". Talks to the firmware over the /api/* JSON endpoints. Included only by
// sun_main.cpp. Pattern mirrors esp32s3_pov_cylinder/src/web_ui.h.
#pragma once
#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Solar System</title>
<style>
:root{--bg:#0b0e14;--card:#141a24;--edge:#232c3a;--tx:#e6edf3;--mut:#8b98a9;
--grn:#2ea043;--yel:#d4a017;--red:#da3633;--acc:#f0a020}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
h1{text-align:center;font-weight:600;letter-spacing:.06em;margin:18px 0 6px;
background:linear-gradient(90deg,#f7b733,#fc4a1a);-webkit-background-clip:text;background-clip:text;color:transparent}
.wrap{max-width:760px;margin:0 auto;padding:0 14px 40px}
.card{background:var(--card);border:1px solid var(--edge);border-radius:12px;padding:16px;margin:14px 0}
.card h2{margin:0 0 12px;font-size:15px;color:var(--mut);text-transform:uppercase;letter-spacing:.08em}
.cardhead{display:flex;align-items:center;justify-content:space-between;margin:0 0 12px}
.cardhead h2{margin:0}
.row{display:flex;gap:10px;flex-wrap:wrap}
.ctl button{flex:1;padding:13px 10px;font-size:16px}
.net{display:flex;flex-wrap:wrap;gap:8px 20px;margin-top:12px;font:12px ui-monospace,Consolas,monospace}
.net span{color:var(--mut)}.net b{color:var(--tx);font-weight:600}
.net a{color:var(--acc);font-weight:600;text-decoration:none}.net a:hover{text-decoration:underline}
button{border:0;border-radius:9px;padding:11px 20px;font-size:15px;font-weight:600;color:#fff;cursor:pointer}
button:active{transform:translateY(1px)}
.b-start{background:var(--grn)}.b-test{background:var(--yel)}.b-stop{background:var(--red)}
.b-reset{background:var(--red);padding:8px 16px}
.mode{font-size:14px;color:var(--mut)}
.timer{font-size:46px;font-weight:700;font-variant-numeric:tabular-nums;text-align:center;margin:6px 0}
svg{display:block;width:100%;max-width:400px;margin:0 auto}
.ptab{width:100%;border-collapse:collapse;margin-top:12px;font:12px ui-monospace,Consolas,monospace}
.ptab th{color:var(--mut);text-align:right;font-weight:600;padding:3px 6px;border-bottom:1px solid var(--edge)}
.ptab th:first-child{text-align:left}
.ptab td{padding:3px 6px;text-align:right;border-bottom:1px solid var(--edge);color:var(--tx)}
.ptab td:first-child{text-align:left;color:var(--mut)}
.ptab tr.off td{color:#4a5568}
.ptab .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;vertical-align:middle}
.ptab .hot{color:var(--red);font-weight:700}
.ptab .er{color:var(--red)}
.ptab .ok{color:var(--grn)}.ptab .wa{color:var(--acc)}
.stats{display:flex;gap:24px;flex-wrap:wrap;margin-bottom:12px}
.stat b{display:block;font-size:30px}
.stat span{color:var(--mut);font-size:13px}
#log{font:12px ui-monospace,Consolas,monospace;color:var(--mut);background:#0d1017;border:1px solid var(--edge);
border-radius:8px;padding:8px;max-height:130px;overflow:auto;white-space:pre-wrap;margin-top:10px}
fieldset{border:1px solid var(--edge);border-radius:8px;margin:8px 0;padding:8px 12px}
legend{color:var(--acc);font-size:13px;text-transform:capitalize}
.f{display:flex;align-items:center;gap:8px;margin:4px 0}
.f label{flex:0 0 46%;color:var(--mut);font-size:13px}
input,select{background:#0d1017;border:1px solid var(--edge);color:var(--tx);border-radius:6px;padding:6px 8px;font:inherit}
input:focus,select:focus{outline:none;border-color:var(--acc)}
.f input,.f select{flex:1}
.arr{display:flex;gap:6px;flex-wrap:wrap}.arr input{width:64px;flex:0 0 auto;padding:6px 4px;text-align:center}
.foot{text-align:center;color:var(--mut);font-size:12px;margin-top:22px}
.toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:#1f6feb;color:#fff;
padding:10px 18px;border-radius:8px;opacity:0;transition:.3s;pointer-events:none}
.toast.on{opacity:1}
</style></head><body>
<div class="wrap">
<h1>Solar System</h1>

<div class="card"><h2>Control</h2><div class="row ctl">
<button class="b-start" onclick="ctl('start')">Start</button>
<button class="b-test"  onclick="ctl('test')">Test</button>
<button class="b-stop"  onclick="ctl('stop')">Stop</button>
</div>
<div class="net" id="net">
<div><span>STA&nbsp;IP:</span> <a id="n_sta" target="_blank" rel="noopener">--</a></div>
<div><span>AP&nbsp;IP:</span> <b id="n_ap">--</b></div>
<div><span>mDNS:</span> <a id="n_mdns" target="_blank" rel="noopener">--</a></div>
</div></div>

<div class="card"><h2>Status</h2>
<div class="mode" id="mode">--</div>
<div class="timer" id="timer">00:00:00</div>
<svg viewBox="0 0 400 400" id="sky"></svg>
<table class="ptab"><thead><tr><th>Planet</th><th>State</th><th>Signal</th><th>Temp</th><th>Cfg</th><th>FW</th><th>Fault</th></tr></thead>
<tbody id="ptab"></tbody></table>
<div id="log"></div>
</div>

<div class="card">
<div class="cardhead"><h2>Statistics</h2>
<button class="b-reset" onclick="resetStats()">Reset</button></div>
<div class="stats">
<div class="stat"><b id="st_trig">0</b><span>Presses that started the show</span></div>
<div class="stat"><b id="st_ign">0</b><span>Presses ignored</span></div>
</div>
</div>

<div class="card"><h2>Cart firmware</h2>
<div class="mode" id="fw_info">--</div>
<div class="row" style="margin-top:10px;align-items:center">
<input type="file" id="fw_file" accept=".bin" style="flex:1;min-width:0">
<button class="b-test" onclick="fwUpload()">Upload</button>
</div>
<div class="mode" style="margin-top:8px">Upload .pio/build/solar_planet_esp32c3/firmware.bin. While the show is
idle, every cart running an older build downloads it from the Sun (all at once), checks it and reboots;
a cart flashed with a newer build (e.g. over USB) is left as is.
A show pauses the downloads. Turn off planets &rsaquo; auto_update_firmware in Settings to hold the carts.</div>
</div>

<div class="card"><h2>Settings</h2>
<div id="settings"></div>
<div class="row" style="margin-top:10px">
<button class="b-start" onclick="saveCfg()">Save &amp; apply</button>
<button class="b-stop" onclick="resetCfg()">Restore defaults</button>
</div></div>

<div class="foot">By Didi, Arad and Calude 2026</div>
</div>
<div class="toast" id="toast"></div>

<script>
const $=s=>document.querySelector(s);
const COLORS=['#f4d35e','#e07a5f','#81b29a','#5f9ea0','#c77dff','#6ca6ff','#f28482','#b5e48c'];
const PLANETS=['Mercury','Venus','Earth','Mars','Jupiter','Saturn','Uranus','Neptune'];
function toast(m){const t=$('#toast');t.textContent=m;t.classList.add('on');setTimeout(()=>t.classList.remove('on'),1600)}
function setLink(a,ip,fallback){if(ip){a.textContent=ip;a.href='http://'+ip+'/';}
 else{a.textContent=fallback;a.removeAttribute('href');}}
function hms(s){s=Math.max(0,s|0);const h=String(s/3600|0).padStart(2,'0'),
m=String(s/60%60|0).padStart(2,'0'),x=String(s%60).padStart(2,'0');return h+':'+m+':'+x}

/* ---- control ---- */
function ctl(a){fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},
body:JSON.stringify({action:a})}).then(()=>toast(a))}
function resetStats(){if(confirm('Reset both statistics counters to zero?'))
fetch('/api/stats/reset',{method:'POST'}).then(()=>{toast('reset');loadStats()})}
function loadStats(){fetch('/api/stats').then(r=>r.json()).then(d=>{
$('#st_trig').textContent=d.presses_triggered;$('#st_ign').textContent=d.presses_ignored})}

/* ---- solar-system diagram ---- */
const SVGNS='http://www.w3.org/2000/svg',CX=200,CY=200,N=8;
const view=[];let sky;
function buildSky(){
 sky=$('#sky');
 const sun=document.createElementNS(SVGNS,'circle');
 sun.setAttribute('cx',CX);sun.setAttribute('cy',CY);sun.setAttribute('r',16);
 sun.setAttribute('fill','#fdb813');sun.setAttribute('filter','url(#g)');
 const defs=document.createElementNS(SVGNS,'defs');
 defs.innerHTML='<filter id="g"><feGaussianBlur stdDeviation="3"/></filter>';
 sky.appendChild(defs);
 for(let i=0;i<N;i++){
  const r=34+i*21;
  const ring=document.createElementNS(SVGNS,'circle');
  ring.setAttribute('cx',CX);ring.setAttribute('cy',CY);ring.setAttribute('r',r);
  ring.setAttribute('fill','none');ring.setAttribute('stroke','#2a3342');ring.setAttribute('stroke-width',1);
  sky.appendChild(ring);
  const aura=document.createElementNS(SVGNS,'circle');
  aura.setAttribute('r',11);aura.setAttribute('fill','none');
  const dot=document.createElementNS(SVGNS,'circle');
  dot.setAttribute('r',6);
  const err=document.createElementNS(SVGNS,'path');
  err.setAttribute('d','M0,-7 L7,6 L-7,6 Z');err.setAttribute('fill','#da3633');
  err.setAttribute('stroke','#fff');err.setAttribute('stroke-width',.7);err.style.display='none';
  sky.appendChild(aura);sky.appendChild(dot);sky.appendChild(err);
  view.push({r,angle:i%2?0:180,orbital:0,online:false,light:false,error:0,
             homeDeg:i%2?0:180,homeCount:-1,aura,dot,err});
 }
 sky.appendChild(sun);
 requestAnimationFrame(anim);
}
let last=0;
function anim(t){
 const dt=last?t-last:16;last=t;
 for(const p of view){
  if(p.online) p.angle-=p.orbital*360/60000*dt;  // rpm -> deg/ms; SVG y points down, so a
                                                 // decreasing angle is counter-clockwise
  const a=p.angle*Math.PI/180, x=CX+p.r*Math.cos(a), y=CY+p.r*Math.sin(a);
  p.dot.setAttribute('cx',x);p.dot.setAttribute('cy',y);
  p.dot.setAttribute('fill',p.online?p.color:'#3b4557');
  p.aura.setAttribute('cx',x);p.aura.setAttribute('cy',y);
  p.aura.setAttribute('stroke',p.light?p.color:'none');
  p.aura.setAttribute('stroke-width',p.light?3:0);p.aura.setAttribute('opacity',.5);
  p.err.setAttribute('transform','translate('+x+','+(y-2)+')');
  p.err.style.display=p.error?'block':'none';
 }
 requestAnimationFrame(anim);
}

/* ---- status poll ---- */
function poll(){fetch('/api/status').then(r=>r.json()).then(d=>{
 $('#mode').textContent='Mode: '+d.mode;
 $('#timer').textContent=hms(d.countdown?d.phase_remaining_s:d.phase_elapsed_s);
 if(d.net){setLink($('#n_sta'),d.net.sta_ip,'not connected');
  $('#n_ap').textContent=d.net.ap_ip||'--';
  setLink($('#n_mdns'),d.net.mdns,'--');}
 const ERRTXT=['','motor disconnected','motor stuck'];let rows='';
 const cf=d.cart_fw||{};
 if(!fwBusy)$('#fw_info').textContent=cf.present?('On the Sun: build '+fwtxt(cf.stamp,1)+' · '+
  Math.round(cf.size/1024)+' KB · auto-update '+(cf.auto?'on':'OFF')):'No cart firmware uploaded to the Sun yet.';
 d.planets.forEach((pl,i)=>{const p=view[i];if(!p)return;
  p.color=COLORS[i];p.online=pl.online;p.light=pl.light_on;p.error=pl.error;
  p.orbital=pl.orbital_rpm;p.homeDeg=pl.home_deg;
  if(pl.home_count!==p.homeCount){p.homeCount=pl.home_count;if(p.homeCount>0)p.angle=p.homeDeg}
  const state=pl.online?'online':(pl.configured?'offline':'—');
  const dot='<span class="dot" style="background:'+(pl.online?COLORS[i]:'#3b4557')+'"></span>';
  const sig=pl.online?(pl.rssi_pct+'% ('+pl.rssi+')'):'—';
  const tc=pl.online?pl.temp_c:null;
  const temp=tc==null?'—':('<span class="'+(tc>=70?'hot':'')+'">'+tc+'&deg;C</span>');
  const fault=pl.error?('<span class="er">'+(ERRTXT[pl.error]||('err '+pl.error))+'</span>'):(pl.online?'ok':'—');
  const cfg=!pl.online?'—':(pl.cfg_ok?'ok':'<span class="er">CHECK</span>');
  let fw='—';                                   // build; download progress / result while updating
  if(pl.online)fw=pl.ota===1?'<span class="wa">&darr; '+pl.ota_pct+'%</span>'
   :pl.ota===2?'<span class="wa">rebooting…</span>'
   :'<span class="'+(pl.ota===3?'er':!cf.present||pl.fw===cf.stamp?'ok':pl.fw<cf.stamp?'wa':'')+'"'+
    ' title="'+(pl.ota===3?'update failed, error '+pl.ota_err:!cf.present||pl.fw===cf.stamp?'up to date':
    pl.fw<cf.stamp?'older than the Sun\'s image: will update while idle':'newer than the Sun\'s image: left as is')+
    '">'+fwtxt(pl.fw)+(cf.present&&pl.fw>cf.stamp?' (newer)':'')+'</span>';
  rows+='<tr class="'+(pl.online?'':'off')+'"><td>'+dot+(PLANETS[i]||('P'+(i+1)))+
   '</td><td>'+state+'</td><td>'+sig+'</td><td>'+temp+'</td><td>'+cfg+'</td><td>'+fw+'</td><td>'+fault+'</td></tr>';
 });
 $('#ptab').innerHTML=rows;
 $('#log').textContent=(d.log||[]).join('\n');
}).catch(()=>{})}

/* ---- cart firmware (OTA) ---- */
let fwBusy=false;                                 // uploading: keep the progress text on screen
function fwtxt(s,full){if(!s)return '—';          // build stamp (UTC Unix s) -> local time
 const d=new Date(s*1000),p=n=>String(n).padStart(2,'0');
 const t=p(d.getMonth()+1)+'-'+p(d.getDate())+' '+p(d.getHours())+':'+p(d.getMinutes());
 return full?d.getFullYear()+'-'+t:t}
function fwUpload(){const f=$('#fw_file').files[0];if(!f){toast('choose a .bin file');return}
 const fd=new FormData();fd.append('fw',f,f.name);const x=new XMLHttpRequest();fwBusy=true;
 x.upload.onprogress=e=>{if(e.lengthComputable)
  $('#fw_info').textContent='Uploading '+Math.round(e.loaded*100/e.total)+'%…'};
 x.onloadend=()=>{fwBusy=false;let r={};try{r=JSON.parse(x.responseText)}catch(e){}
  toast(r.ok?'uploaded':('upload failed: '+(r.error||('HTTP '+x.status))))};
 x.open('POST','/api/planet_fw');x.send(fd)}

/* ---- settings (recursive form bound to a live cfg object) ---- */
let cfg=null;
function setPath(o,path,v){let n=o;for(let i=0;i<path.length-1;i++)n=n[path[i]];n[path[path.length-1]]=v}
function mkInput(v,path){
 const inp=document.createElement('input');
 if(typeof v==='boolean'){inp.type='checkbox';inp.checked=v;
  inp.onchange=()=>setPath(cfg,path,inp.checked)}
 else{inp.type=(typeof v==='number')?'number':'text';inp.step='any';inp.value=v;
  inp.onchange=()=>setPath(cfg,path,inp.type==='number'?parseFloat(inp.value):inp.value)}
 return inp;
}
const MICROSTEPS=[1,2,4,8,16,32,64,128,256];
const CHOPPER=[['stealth','stealth (quiet)'],['spread','spread (normal)']];  // [value,label]
const REVERSE=[['none','none'],['left','left wheel'],['right','right wheel'],['both','both (cart backwards)']];
function perPlanet(title,val,path,mk){          // one labelled line per planet
 const fs=document.createElement('fieldset');const lg=document.createElement('legend');
 lg.textContent=title;fs.appendChild(lg);
 val.forEach((el,i)=>{const f=document.createElement('div');f.className='f';
  const l=document.createElement('label');l.textContent=(i+1)+'. '+(PLANETS[i]||('Planet '+(i+1)));
  f.appendChild(l);f.appendChild(mk(el,path.concat(i)));fs.appendChild(f);});
 return fs;
}
function mkSelect(v,path,opts){                 // restrict a value to a fixed numeric option set
 const sel=document.createElement('select');
 opts.forEach(o=>{const op=document.createElement('option');op.value=o;op.textContent=o;
  if(+o===+v)op.selected=true;sel.appendChild(op)});
 sel.onchange=()=>setPath(cfg,path,parseInt(sel.value));
 return sel;
}
function mkSelectStr(v,path,opts){              // restrict a value to a fixed string option set;
 const sel=document.createElement('select');   // opts entries may be 'value' or ['value','label']
 opts.forEach(o=>{const val=Array.isArray(o)?o[0]:o,txt=Array.isArray(o)?o[1]:o;
  const op=document.createElement('option');op.value=val;op.textContent=txt;
  if(val===v)op.selected=true;sel.appendChild(op)});
 sel.onchange=()=>setPath(cfg,path,sel.value);
 return sel;
}
function render(key,val,path){
 if(key==='microsteps'&&!Array.isArray(val)&&typeof val!=='object'){
  const f=document.createElement('div');f.className='f';
  const l=document.createElement('label');l.textContent=key;
  f.appendChild(l);f.appendChild(mkSelect(val,path,MICROSTEPS));return f;
 }
 if(key==='chopper_mode'&&typeof val==='string'){
  const f=document.createElement('div');f.className='f';
  const l=document.createElement('label');l.textContent=key;
  f.appendChild(l);f.appendChild(mkSelectStr(val,path,CHOPPER));return f;
 }
 if(key==='mac_address'&&Array.isArray(val))return perPlanet(key,val,path,mkInput);
 if(key==='light_brightness_per'&&Array.isArray(val))   // per-planet light brightness, 0..100 %
  return perPlanet('light brightness (%)',val,path,(v,p)=>{const inp=mkInput(v,p);
   inp.min=0;inp.max=100;inp.step=1;return inp});
 if(key==='reverse_per'&&Array.isArray(val))            // per-cart wheel direction (mounting/wiring)
  return perPlanet('reverse wheel',val,path,(v,p)=>mkSelectStr(v,p,REVERSE));
 if(Array.isArray(val)){
  const fs=document.createElement('fieldset');const lg=document.createElement('legend');
  lg.textContent=key;fs.appendChild(lg);
  const box=document.createElement('div');box.className='arr';
  val.forEach((el,i)=>box.appendChild(mkInput(el,path.concat(i))));
  fs.appendChild(box);return fs;
 }
 if(val&&typeof val==='object'){
  const fs=document.createElement('fieldset');const lg=document.createElement('legend');
  lg.textContent=key;fs.appendChild(lg);
  for(const k in val)fs.appendChild(render(k,val[k],path.concat(k)));
  return fs;
 }
 const f=document.createElement('div');f.className='f';
 const l=document.createElement('label');l.textContent=key;
 f.appendChild(l);f.appendChild(mkInput(val,path));return f;
}
function loadCfg(){fetch('/api/settings').then(r=>r.json()).then(d=>{cfg=d;
 const box=$('#settings');box.innerHTML='';
 for(const k in d)box.appendChild(render(k,d[k],[k]));})}
function saveCfg(){fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify(cfg)}).then(()=>{toast('saved');setTimeout(loadCfg,300)})}
function resetCfg(){if(confirm('Restore ALL settings to factory defaults? This also clears the planet MAC addresses.'))
 fetch('/api/settings/reset',{method:'POST'}).then(()=>{toast('defaults restored');setTimeout(loadCfg,700)})}

buildSky();loadStats();loadCfg();poll();setInterval(poll,1000);setInterval(loadStats,5000);
</script></body></html>)HTML";
