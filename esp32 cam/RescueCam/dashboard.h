// RESCUE CAM dashboard - served once from flash at "/". Live data: /api/status (1 s).
#pragma once
static const char DASHBOARD_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="theme-color" content="#05080d">
<title>Rescue Cam</title>
<style>
:root{--bg:#05080d;--g1:rgba(15,25,37,.76);--g2:rgba(22,34,50,.6);--line:rgba(120,170,220,.14);
--text:#e7eff7;--sub:#8093a6;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
--cyan:#22d3ee;--ok:#34d399;--warn:#fbbf24;--err:#f43f5e;--gray:#64748b;--violet:#a78bfa}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0}
body{background:var(--bg);color:var(--text);font-family:-apple-system,'Segoe UI',Roboto,Arial,sans-serif;font-size:14px;min-height:100vh}
body::before{content:"";position:fixed;inset:0;z-index:-1;pointer-events:none;
 background:radial-gradient(900px 520px at 0% -10%,rgba(34,211,238,.11),transparent 60%),radial-gradient(800px 480px at 110% 0%,rgba(167,139,250,.09),transparent 60%)}
.wrap{max-width:1200px;margin:0 auto;padding:12px 12px 30px}
.glass{background:linear-gradient(160deg,var(--g1),var(--g2));border:1px solid var(--line);border-radius:18px;box-shadow:0 10px 30px rgba(0,0,0,.35),inset 0 1px 0 rgba(255,255,255,.04)}
.sec{font-size:10.5px;letter-spacing:2px;text-transform:uppercase;color:var(--sub);margin:18px 4px 9px;display:flex;align-items:center;gap:10px}
.sec::after{content:"";flex:1;height:1px;background:linear-gradient(90deg,var(--line),transparent)}
.mono{font-family:var(--mono)}
.top{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:4px 2px 12px}
.brand{display:flex;align-items:center;gap:11px;min-width:0}
.logo{width:42px;height:42px;border-radius:13px;display:grid;place-items:center;font-size:21px;flex:none;background:radial-gradient(circle at 30% 25%,rgba(34,211,238,.42),rgba(59,130,246,.14) 60%,transparent);border:1px solid rgba(34,211,238,.35)}
h1{margin:0;font-size:16px;letter-spacing:1.8px;font-weight:800;white-space:nowrap}
.tag{font-size:10.5px;color:var(--sub);margin-top:2px}
.pill{display:flex;align-items:center;gap:7px;font-size:10.5px;font-weight:800;letter-spacing:1.1px;padding:6px 11px;border-radius:20px;border:1px solid var(--line);background:rgba(10,16,24,.75);flex:none}
.dot{width:8px;height:8px;border-radius:50%;background:var(--gray)}
.pill[data-s=ok]{color:var(--ok)}.pill[data-s=ok] .dot{background:var(--ok)}
.pill[data-s=lost]{color:var(--err)}.pill[data-s=lost] .dot{background:var(--err);animation:blink .8s infinite}
@keyframes blink{50%{opacity:.25}}
.main{display:grid;grid-template-columns:1fr;gap:12px}
@media(min-width:980px){.main{grid-template-columns:1.45fr 1fr;align-items:start}}
.video{padding:10px}
.vbox{position:relative;border-radius:12px;overflow:hidden;background:#000;aspect-ratio:4/3}
.vbox img{width:100%;height:100%;object-fit:contain;display:block}
.vmsg{position:absolute;inset:0;display:grid;place-items:center;color:var(--sub);font-size:13px;text-align:center;padding:20px}
.corner{position:absolute;width:22px;height:22px;border-color:rgba(34,211,238,.7);border-style:solid;pointer-events:none}
.c1{top:8px;left:8px;border-width:2px 0 0 2px}.c2{top:8px;right:8px;border-width:2px 2px 0 0}.c3{bottom:8px;left:8px;border-width:0 0 2px 2px}.c4{bottom:8px;right:8px;border-width:0 2px 2px 0}
.ov{position:absolute;font-size:10.5px;font-weight:800;letter-spacing:1px;padding:4px 9px;border-radius:9px;background:rgba(0,0,0,.55);color:#fff;display:flex;align-items:center;gap:6px}
.ovl{top:12px;left:36px}.ovr{top:12px;right:36px;font-family:var(--mono)}.ovb{bottom:12px;left:36px;font-family:var(--mono)}
.ovl i{width:8px;height:8px;border-radius:50%;background:var(--ok)}
.rec-on .ovl{background:rgba(244,63,94,.9)} .rec-on .ovl i{background:#fff;animation:blink 1s infinite}
.vbox.rec-on{box-shadow:0 0 0 3px rgba(244,63,94,.85)}
.bar{display:flex;gap:8px;margin-top:10px;flex-wrap:wrap}
.btn{flex:1;min-width:130px;padding:12px 10px;border-radius:12px;border:1px solid var(--line);background:rgba(20,30,44,.75);color:var(--text);font-size:13px;font-weight:800;letter-spacing:.4px;text-align:center;cursor:pointer;font-family:inherit;text-decoration:none}
.btn:active{filter:brightness(1.3)} .btn:disabled{opacity:.45;cursor:not-allowed}
.btn.photo{border-color:rgba(34,211,238,.5);color:#a5f3fc}
.btn.rec{border-color:rgba(244,63,94,.55);color:#fecdd3;background:rgba(244,63,94,.12)}
.btn.stop{border-color:rgba(251,191,36,.6);color:#fde68a;background:rgba(251,191,36,.12)}
.btn.sm{flex:0 0 auto;min-width:0;padding:6px 10px;font-size:11.5px;border-radius:9px}
.btn.go{border-color:rgba(52,211,153,.6);color:#a7f3d0;background:rgba(52,211,153,.14);font-size:14px;padding:14px 10px}
.btn.lg{font-size:14px;padding:14px 10px}
.states{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:10px}
.st{border:1px solid var(--line);border-radius:12px;padding:8px 10px;background:rgba(255,255,255,.03);min-width:0}
.st small{display:block;font-size:9.5px;letter-spacing:1.3px;color:var(--sub);font-weight:800}
.st b{display:flex;align-items:center;gap:6px;font-size:13px;font-weight:800;letter-spacing:.5px;margin-top:3px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.st b::before{content:"";width:8px;height:8px;border-radius:50%;background:var(--gray);flex:none}
.st[data-s=on]{border-color:rgba(52,211,153,.45)} .st[data-s=on] b{color:var(--ok)} .st[data-s=on] b::before{background:var(--ok)}
.st[data-s=rec]{border-color:rgba(244,63,94,.6);background:rgba(244,63,94,.1)} .st[data-s=rec] b{color:#fda4af} .st[data-s=rec] b::before{background:var(--err);animation:blink 1s infinite}
.st[data-s=warn]{border-color:rgba(251,191,36,.5)} .st[data-s=warn] b{color:var(--warn)} .st[data-s=warn] b::before{background:var(--warn)}
.st[data-s=err]{border-color:rgba(244,63,94,.45)} .st[data-s=err] b{color:var(--err)} .st[data-s=err] b::before{background:var(--err)}
.sdban{margin-top:10px;border:1px solid rgba(251,191,36,.5);background:rgba(251,191,36,.1);color:#fde68a;border-radius:12px;padding:10px 12px;font-weight:800;font-size:12.5px;letter-spacing:.4px}
.sdban small{display:block;font-weight:500;color:#e8d9a8;letter-spacing:0;margin-top:3px;font-size:11.5px}
.vmsg b{display:block;font-size:17px;letter-spacing:2px;color:var(--text);margin-bottom:4px}
.offico{font-size:34px;opacity:.6;margin-bottom:6px}
.rnote{font-size:11.5px;color:var(--sub);margin-top:6px;line-height:1.45}
.rnote:empty{display:none}
.saved{margin-top:10px}
.srow{display:flex;justify-content:space-between;align-items:center;gap:8px;padding:6px 0;border-bottom:1px dashed var(--line);font-size:12.5px;min-width:0}
.srow span{font-family:var(--mono);overflow:hidden;text-overflow:ellipsis;white-space:nowrap;min-width:0}
.srow em{font-style:normal;color:var(--ok);font-size:10.5px;font-weight:800;margin-left:6px}
details.help{margin-top:12px;border:1px solid var(--line);border-radius:12px;padding:10px 12px;background:rgba(34,211,238,.05)}
details.help summary{cursor:pointer;font-weight:800;font-size:12.5px;color:#a5f3fc}
details.help ol{margin:8px 0 0;padding-left:18px;font-size:12px;line-height:1.55;color:#c3cfdb}
details.help li{margin-bottom:6px}
.qrow{display:flex;align-items:center;gap:8px;margin-top:10px;min-width:0}
.ql{flex:0 0 74px;font-size:9.5px;letter-spacing:1.3px;color:var(--sub);font-weight:800}
.seg{display:flex;flex:1;min-width:0;border:1px solid var(--line);border-radius:12px;overflow:hidden;background:rgba(255,255,255,.03)}
.seg button{flex:1;min-width:0;padding:10px 4px;border:0;border-right:1px solid var(--line);background:transparent;color:var(--sub);font:800 11.5px/1.15 inherit;font-family:inherit;letter-spacing:.4px;cursor:pointer}
.seg button:last-child{border-right:0}
.seg button small{display:block;font-weight:600;font-size:9.5px;letter-spacing:0;opacity:.8;margin-top:2px}
.seg button.on{background:rgba(34,211,238,.16);color:#cffafe;box-shadow:inset 0 -2px 0 var(--cyan)}
.seg button:disabled{opacity:.45;cursor:not-allowed}
.duse{display:flex;justify-content:space-between;gap:10px;margin-top:10px;padding:8px 10px;border-radius:12px;border:1px dashed var(--line);font-size:12px;color:var(--sub);min-width:0}
.duse b{font-family:var(--mono);color:#d7e0e9;font-weight:700;text-align:right;min-width:0;overflow-wrap:anywhere}
.toast{margin-top:8px;font-size:12px;color:var(--sub);min-height:17px}
.toast.ok{color:var(--ok)} .toast.err{color:var(--err)}
.cards{display:grid;grid-template-columns:1fr;gap:12px}
@media(min-width:620px){.cards{grid-template-columns:1fr 1fr}}
.card{padding:14px}
.ch{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:8px}
.ct{font-size:12.5px;font-weight:800;letter-spacing:.6px}
.badge{display:inline-flex;align-items:center;gap:6px;padding:3px 9px;border-radius:20px;font-size:10.5px;font-weight:800;letter-spacing:.5px;background:rgba(100,116,139,.15);color:var(--gray);white-space:nowrap}
.badge i{width:7px;height:7px;border-radius:50%;background:currentColor}
.b-ok{background:rgba(52,211,153,.12);color:var(--ok)}.b-err{background:rgba(244,63,94,.14);color:var(--err)}.b-warn{background:rgba(251,191,36,.12);color:var(--warn)}.b-rec{background:rgba(244,63,94,.9);color:#fff}
.b-rec i{animation:blink 1s infinite}
.kv{display:flex;justify-content:space-between;font-size:12px;color:var(--sub);padding:3px 0;gap:10px}
.kv b{color:#d7e0e9;font-family:var(--mono);font-weight:600;text-align:right;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;max-width:65%}
.big{font-family:var(--mono);font-size:26px;font-weight:800;margin:2px 0 6px}
.meter{position:relative;height:8px;border-radius:6px;background:rgba(255,255,255,.07);overflow:hidden;margin:6px 0}
.fill{height:100%;width:0;border-radius:6px;background:var(--violet);transition:width .4s}
.fill.low{background:var(--err)}
.warn{font-size:11.5px;color:var(--warn);margin-top:6px}
.warn:empty{display:none}
.thumb{width:100%;aspect-ratio:4/3;object-fit:cover;border-radius:10px;background:#000;display:block;margin-top:6px;cursor:pointer}
/* file manager */
.tabs{display:flex;gap:6px;margin-bottom:10px;flex-wrap:wrap}
.tab{padding:7px 12px;border-radius:10px;border:1px solid var(--line);background:rgba(255,255,255,.03);color:var(--sub);font-size:12px;font-weight:800;cursor:pointer;font-family:inherit}
.tab.on{color:var(--text);border-color:rgba(34,211,238,.55);background:rgba(34,211,238,.1)}
.fm{padding:12px}
.ftable{width:100%;border-collapse:collapse;font-size:12px}
.ftable th{text-align:left;color:var(--sub);font-weight:700;font-size:10.5px;letter-spacing:1px;padding:6px 6px;border-bottom:1px solid var(--line)}
.ftable td{padding:7px 6px;border-bottom:1px dashed var(--line);vertical-align:middle}
.ftable td.n{font-family:var(--mono);word-break:break-all}
.ftable td.a{white-space:nowrap;text-align:right}
.ftable .muted{color:var(--sub)}
@media(max-width:560px){.hide-s{display:none}}
/* settings */
.set{display:grid;grid-template-columns:minmax(0,1fr);gap:10px}.set>div{min-width:0}.kv b{min-width:0;overflow-wrap:anywhere}
@media(min-width:620px){.set{grid-template-columns:minmax(0,1fr) minmax(0,1fr)}}
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;font-size:12.5px;color:#c3cfdb}
.row select,.row input[type=range]{flex:0 0 55%;min-width:0}
.row>span{flex:0 1 auto;min-width:0}
select{background:rgba(20,30,44,.9);color:var(--text);border:1px solid var(--line);border-radius:9px;padding:7px;font-family:inherit}
input[type=range]{accent-color:var(--cyan)}
.tog{position:relative;width:48px;height:26px;border-radius:14px;border:1px solid rgba(148,163,184,.3);background:rgba(51,65,85,.6);cursor:pointer;flex:none;padding:0}
.tog::after{content:"";position:absolute;top:3px;left:3px;width:18px;height:18px;border-radius:50%;background:#94a3b8;transition:left .2s}
.tog.on{background:rgba(34,211,238,.35);border-color:rgba(34,211,238,.7)} .tog.on::after{left:25px;background:#cffafe}
.note{font-size:11px;color:var(--sub);margin-top:8px;line-height:1.5}
.inp{flex:1 1 55%;width:0;min-width:0;background:rgba(20,30,44,.9);color:var(--text);border:1px solid var(--line);border-radius:9px;padding:8px;font-family:inherit;font-size:13px}
/* modal */
.modal{position:fixed;inset:0;background:rgba(0,0,0,.85);display:none;align-items:center;justify-content:center;z-index:10;padding:14px}
.modal.on{display:flex}
.mbox{max-width:960px;width:100%}
.mbox img{width:100%;max-height:78vh;object-fit:contain;border-radius:12px;background:#000;display:block}
.mhead{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:8px;font-family:var(--mono);font-size:12px;color:#dbe7f3}
footer{text-align:center;font-size:10px;color:var(--sub);margin-top:18px}
@media(prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
</style>
</head>
<body>
<div class="wrap">
<header class="top">
  <div class="brand"><div class="logo">🎥</div><div><h1>RESCUE CAM</h1><div class="tag">ESP32-S3-CAM · live stream · auto recording · microSD</div></div></div>
  <div class="pill" id="pill"><span class="dot"></span><span id="pillTxt">CONNECTING</span></div>
</header>

<div class="main">
  <section class="glass video">
    <div class="vbox" id="vbox">
      <img id="cam" alt="" style="visibility:hidden">
      <div class="vmsg" id="vmsg"><div><div class="offico">⏻</div><b id="vTitle">STREAM OFF</b><div id="vSub">Tap START STREAM to watch live video</div></div></div>
      <i class="corner c1"></i><i class="corner c2"></i><i class="corner c3"></i><i class="corner c4"></i>
      <div class="ov ovl" id="ovl" style="display:none"><i></i><span id="ovRec">LIVE</span></div>
      <div class="ov ovr" id="ovRes">--</div>
      <div class="ov ovb" id="ovClock">--:--:--</div>
    </div>
    <div class="bar">
      <button class="btn go" id="btnStart" onclick="streamCmd(true)">▶ START STREAM</button>
      <button class="btn stop lg" id="btnStop" onclick="streamCmd(false)" disabled>■ STOP STREAM</button>
    </div>
    <div class="qrow"><span class="ql">QUALITY</span><div class="seg" id="segQ">
      <button data-p="0" onclick="setPreset(0)">LOW / FAST<small>320×240</small></button>
      <button data-p="1" onclick="setPreset(1)">MEDIUM<small>640×480</small></button>
      <button data-p="2" onclick="setPreset(2)">HIGH<small>1280×720</small></button></div></div>
    <div class="qrow"><span class="ql">FRAME RATE</span><div class="seg" id="segF">
      <button data-f="0" onclick="setFpsMode(0)">DATA SAVER<small>5 fps</small></button>
      <button data-f="1" onclick="setFpsMode(1)">NORMAL<small id="fNorm">10 fps</small></button>
      <button data-f="2" onclick="setFpsMode(2)">MAX<small>no limit</small></button></div></div>
    <div class="duse"><span>Live data use</span><b id="dUse">stream off</b></div>
    <div class="states">
      <div class="st" id="stStream"><small>STREAM</small><b>--</b></div>
      <div class="st" id="stRec"><small>RECORDING</small><b>--</b></div>
      <div class="st" id="stSd"><small>SD CARD</small><b>--</b></div>
    </div>
    <div class="sdban" id="sdBan" style="display:none">⚠ SD CARD NOT INSERTED / RECORDING DISABLED<small>The live stream works normally. Insert a FAT32 card — while the stream is ON, recording starts automatically.</small></div>
    <div class="bar">
      <button class="btn photo" id="btnPhoto" onclick="takePhoto()">📸 CAPTURE PHOTO</button>
      <button class="btn sm" onclick="fullscreen()" title="Full screen">⛶ Full screen</button>
    </div>
    <div class="toast" id="toast"></div>
  </section>

  <div class="cards">
    <section class="glass card">
      <div class="ch"><div class="ct" style="color:var(--cyan)">CAMERA</div><span class="badge" id="bCam"><i></i><span>--</span></span></div>
      <div class="kv"><span>Sensor</span><b id="cSensor">--</b></div>
      <div class="kv"><span>Resolution</span><b id="cRes">--</b></div>
      <div class="kv"><span>Quality mode</span><b id="cMode">--</b></div>
      <div class="kv"><span>JPEG quality</span><b id="cQ">--</b></div>
      <div class="kv"><span>Camera FPS</span><b id="cFps">--</b></div>
      <div class="kv"><span>Live viewers</span><b id="cView">--</b></div>
      <div class="kv"><span>Memory free</span><b id="cMem">--</b></div>
    </section>

    <section class="glass card">
      <div class="ch"><div class="ct" style="color:#60a5fa">NETWORK</div><span class="badge" id="bNet"><i></i><span>--</span></span></div>
      <div class="kv"><span>Connection</span><b id="cNet">--</b></div>
      <div class="kv"><span>WiFi joined</span><b id="cSsid">--</b></div>
      <div class="kv"><span>Camera address</span><b id="cIp">--</b></div>
      <div class="kv"><span>Signal / clients</span><b id="cSig">--</b></div>
      <div class="kv"><span>Uptime</span><b id="cUp">--</b></div>
    </section>

    <section class="glass card">
      <div class="ch"><div class="ct" style="color:var(--violet)">SD CARD</div><span class="badge" id="bSd"><i></i><span>--</span></span></div>
      <div class="kv"><span>Type</span><b id="sType">--</b></div>
      <div class="kv"><span>Capacity</span><b id="sCap">--</b></div>
      <div class="kv"><span>Used</span><b id="sUsed">--</b></div>
      <div class="kv"><span>Free</span><b id="sFree">--</b></div>
      <div class="meter"><div class="fill" id="sBar"></div></div>
      <div class="kv"><span>Photos / Videos</span><b id="sCount">--</b></div>
      <div class="warn" id="sWarn"></div>
    </section>

    <section class="glass card">
      <div class="ch"><div class="ct" style="color:#fb7185">RECORDING</div><span class="badge" id="bRec"><i></i><span>OFF</span></span></div>
      <div class="big" id="rDur">00:00:00</div>
      <div class="kv"><span>Current file</span><b id="rFile">--</b></div>
      <div class="kv"><span>Frames / FPS</span><b id="rFr">--</b></div>
      <div class="kv"><span>Size</span><b id="rSize">--</b></div>
      <div class="kv"><span>Auto-record</span><b id="rAuto">--</b></div>
      <div class="kv"><span>Last saved</span><b id="rLast">--</b></div>
      <div class="rnote" id="rNote"></div>
      <div class="bar" id="recBar" style="display:none"><button class="btn sm" id="btnRec" onclick="toggleRec()">⏹ Stop recording (keep stream)</button></div>
    </section>

    <section class="glass card">
      <div class="ch"><div class="ct" style="color:#67e8f9">PHOTO CAPTURE</div><span class="badge b-ok" id="bPh"><i></i><span>READY</span></span></div>
      <div class="kv"><span>Last photo</span><b id="pName">--</b></div>
      <img class="thumb" id="pThumb" alt="" style="display:none" onclick="viewPhoto(lastPhoto)">
    </section>
  </div>
</div>

<div class="sec">File manager</div>
<section class="glass fm">
  <div class="tabs">
    <button class="tab on" data-d="/photos" onclick="setDir(this)">📷 /photos</button>
    <button class="tab" data-d="/videos" onclick="setDir(this)">🎞 /videos</button>
    <button class="tab" data-d="/" onclick="setDir(this)">🗂 / (card root)</button>
    <button class="btn sm" onclick="loadFiles()">↻ Refresh</button>
  </div>
  <table class="ftable"><thead><tr><th>NAME</th><th class="hide-s">TYPE</th><th>SIZE</th><th class="hide-s">DATE</th><th style="text-align:right">ACTIONS</th></tr></thead>
  <tbody id="flist"><tr><td colspan="5" class="muted">Loading…</td></tr></tbody></table>
  <div class="note">Videos are <b>MJPEG in AVI</b> (a sequence of JPEG frames with an index). Browsers cannot play .avi directly — <b>▶ Play</b> streams it here frame by frame; <b>Download</b> plays in VLC / Windows Media Player / most phones' VLC.</div>
</section>

<div class="sec">Camera settings</div>
<section class="glass card">
  <div class="set">
    <div class="row"><span>Custom resolution</span><select id="setFs" onchange="setCtl('framesize',this.value)">
      <option value="5">QVGA 320×240</option><option value="8">VGA 640×480</option><option value="9">SVGA 800×600</option>
      <option value="10">XGA 1024×768</option><option value="11">HD 1280×720</option><option value="12">SXGA 1280×1024</option><option value="13">UXGA 1600×1200</option></select></div>
    <div class="row"><span>JPEG quality <small class="muted">(lower = better)</small></span><input type="range" id="setQ" min="6" max="40" step="1" onchange="setCtl('quality',this.value)"></div>
    <div class="row"><span>Brightness</span><input type="range" id="setBri" min="-2" max="2" step="1" onchange="setCtl('brightness',this.value)"></div>
    <div class="row"><span>Contrast</span><input type="range" id="setCon" min="-2" max="2" step="1" onchange="setCtl('contrast',this.value)"></div>
    <div class="row"><span>Saturation</span><input type="range" id="setSat" min="-2" max="2" step="1" onchange="setCtl('saturation',this.value)"></div>
    <div class="row"><span>Horizontal flip</span><button class="tog" id="setHm" onclick="flip('hmirror',this)"></button></div>
    <div class="row"><span>Vertical flip</span><button class="tog" id="setVf" onclick="flip('vflip',this)"></button></div>
    <div class="row"><span>Auto-record with stream</span><button class="tog" id="setAuto" onclick="setAuto(this)"></button></div>
  </div>
  <div class="note"><b>LOW / FAST / MEDIUM / HIGH</b> (under the video) set resolution + quality together; the two controls above are for a custom mix. Every START STREAM begins in LOW / FAST. Changing the resolution while recording saves the current video and continues in a <b>new file</b> (one video file = one frame size). The frame-rate limit only affects the live view — the SD recording always uses every camera frame. This board has no camera flash LED; the small on-board LED blinks while recording.</div>
</section>
<div class="sec">Network &amp; WiFi</div>
<section class="glass card">
  <div class="set">
    <div>
      <div class="kv"><span>Connection</span><b id="wMode">--</b></div>
      <div class="kv"><span>WiFi joined</span><b id="wSsid">--</b></div>
      <div class="kv"><span>Open camera at</span><b id="wAddr">--</b></div>
      <div class="kv"><span>Signal</span><b id="wSig">--</b></div>
      <div class="note" id="wHint"></div>
      <div class="saved"><div class="ct" style="font-size:11px;color:var(--sub)">SAVED NETWORKS (max 3)</div><div id="wSaved"><div class="note">none</div></div></div>
    </div>
    <div>
      <div class="row"><span>Network</span><select id="wList" style="flex:1 1 55%;width:0;min-width:0"><option value="">— tap Scan —</option></select></div>
      <div class="row" style="margin-top:8px"><span>or type name</span><input id="wName" maxlength="32" placeholder="WiFi / hotspot name" class="inp"></div>
      <div class="row" style="margin-top:8px"><span>Password</span><input id="wPass" type="password" maxlength="64" placeholder="password" class="inp"></div>
      <div class="bar">
        <button class="btn sm" id="wScan" onclick="wifiScan()">📶 Scan</button>
        <button class="btn sm" id="wSave" onclick="wifiSave()" style="border-color:rgba(52,211,153,.5);color:var(--ok)">💾 Save &amp; connect</button>
      </div>
      <div class="note">Add your home WiFi and/or your <b>phone's hotspot</b>. The camera restarts and joins the network (last saved first, otherwise the strongest). If none is reachable, its own WiFi <b>RescueCam</b> (password rover1234, http://192.168.4.1) is always there.</div>
    </div>
  </div>
  <details class="help"><summary>📱 Watching while your phone uses MOBILE DATA</summary>
    <ol>
      <li><b>Phone hotspot (recommended, no extra hardware):</b> turn on your phone's hotspot, add its name/password above, and the camera joins it. Your phone keeps its mobile-data internet <i>and</i> sees the camera. Open the camera's IP (the hotspot's “connected devices” list shows <b>rescuecam</b>; or Serial Monitor) — iPhone/laptop can also use http://rescuecam.local.</li>
      <li><b>RescueCam WiFi:</b> connect the phone to RescueCam, open http://192.168.4.1. If the page does not open, the phone is sending it over mobile data: tap “Stay connected / Use this network”, or turn mobile data off.</li>
      <li><b>From anywhere over the internet:</b> the ESP32 has no cellular modem and a WiFi camera cannot be reached from the mobile network by itself. The camera needs an internet uplink (home WiFi, phone hotspot or 4G pocket router) <i>plus</i> a tunnel/VPN — e.g. Tailscale (subnet router) or Cloudflare Tunnel — running on a laptop / Raspberry Pi in the same network. Set a web password (WEB_PASS) first.</li>
      <li><b>Save data:</b> keep LOW / FAST + DATA SAVER (about 100-150 MB per hour). The live view pauses automatically when this tab is in the background; recording on the SD card continues.</li>
    </ol>
  </details>
</section>
<footer>RESCUE CAM · ESP32-S3 · /stream · /api/status</footer>
</div>

<div class="modal" id="modal" onclick="if(event.target===this)closeModal()">
  <div class="mbox"><div class="mhead"><span id="mTitle"></span><span><a class="btn sm" id="mDl">⬇ Download</a> <button class="btn sm" onclick="closeModal()">✕ Close</button></span></div><img id="mImg" alt=""></div>
</div>

<script>
const $=id=>document.getElementById(id);
function T(id,v){const e=$(id);v=String(v);if(e&&e.textContent!==v)e.textContent=v;}
function badge(id,txt,cls){const e=$(id);const k='badge '+cls;if(e.className!==k)e.className=k;const s=e.lastElementChild;if(s.textContent!==txt)s.textContent=txt;}
function gb(b){return b>=1e9?(b/1e9).toFixed(2)+' GB':(b>=1e6?(b/1e6).toFixed(1)+' MB':(b/1e3).toFixed(0)+' KB');}
function hms(ms){const s=Math.floor(ms/1000);return [s/3600|0,(s%3600)/60|0,s%60].map(x=>String(x).padStart(2,'0')).join(':');}
function esc(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
function toast(msg,ok){const t=$('toast');t.textContent=msg;t.className='toast '+(ok?'ok':'err');clearTimeout(t._t);t._t=setTimeout(()=>{t.textContent='';t.className='toast';},6000);}

// ---------- clock sync (phone/PC time -> file names + file dates) ----------
fetch('/api/time?epoch='+Math.floor(Date.now()/1000)+'&tz='+new Date().getTimezoneOffset()).catch(()=>{});
setInterval(()=>{const d=new Date();T('ovClock',d.toTimeString().slice(0,8));},1000);

// ---------- live view (only while the stream session is ON) ----------
const cam=$('cam'); let camTimer=null, camLoaded=false, viewing=false, streamOn=false, lastD=null;
function showMsg(t,sub){ $('vmsg').style.display=''; T('vTitle',t); T('vSub',sub); }
// Two ways to show live video:
//  'mjpeg' = one endless /stream response (lowest latency, normal case)
//  'snap'  = single JPEGs from /capture, paced by the frame-rate limit. Used
//            automatically when the stream cannot get through (some tunnels /
//            proxies buffer endless responses) - works through anything.
let vmode='mjpeg', vfails=0;
function connectImg(){ camLoaded=false; clearTimeout(camTimer);
  if(vmode==='snap'){ snapNext(); return; }
  cam.src='/stream?t='+Date.now();
  camTimer=setTimeout(()=>{ if(viewing&&!camLoaded) streamFailed('Video not reachable yet — retrying'); },7000); }
function streamFailed(why){ if(!viewing) return; cam.removeAttribute('src');
  if(vmode==='mjpeg'&&++vfails>=2){ vmode='snap'; toast('Live video switched to SNAPSHOT mode (the network/tunnel holds back the video stream)',true); }
  showMsg('RECONNECTING…',why); clearTimeout(camTimer); camTimer=setTimeout(()=>{ if(viewing) connectImg(); },2000); }
function snapNext(){ if(!viewing||vmode!=='snap') return; clearTimeout(camTimer); snapT=performance.now(); cam.src='/capture?t='+Date.now();
  camTimer=setTimeout(()=>{ if(viewing) snapNext(); },8000); }
let snapT=0;
function startView(){ if(viewing||document.hidden) return; viewing=true; cam.style.visibility=''; showMsg('STARTING…','Connecting to live video'); connectImg(); }
function stopView(){ viewing=false; clearTimeout(camTimer); cam.removeAttribute('src'); cam.style.visibility='hidden'; showMsg('STREAM OFF','Tap START STREAM to watch live video'); }
cam.onload=()=>{ if(!viewing) return; camLoaded=true; $('vmsg').style.display='none';
  if(vmode==='mjpeg'){ vfails=0; return; }
  clearTimeout(camTimer); const lim=(lastD&&lastD.stream.lim)||8, gap=1000/lim-(performance.now()-snapT);
  camTimer=setTimeout(snapNext,Math.max(0,gap)); };
cam.onerror=()=>{ if(!viewing) return; const v=lastD?lastD.stream:null;
  if(vmode==='snap'){ clearTimeout(camTimer); camTimer=setTimeout(snapNext,1500); return; }
  streamFailed(v&&v.viewers>=v.max?'Too many viewers (max '+v.max+') — waiting for a free slot':'Live video lost — reconnecting'); };
function fullscreen(){ const v=$('vbox'); (v.requestFullscreen||v.webkitRequestFullscreen||function(){}).call(v); }
// data saving: no video download while this tab/app is in the background
document.addEventListener('visibilitychange',()=>{ if(document.hidden){ if(viewing){ clearTimeout(camTimer); cam.removeAttribute('src'); viewing=false; bgPaused=true; } }
  else if(bgPaused){ bgPaused=false; if(streamOn) startView(); } });
let bgPaused=false;
let streamBusy=false;
async function streamCmd(on){ if(streamBusy) return; streamBusy=true;
  const a=$('btnStart'), b=$('btnStop'); a.disabled=b.disabled=true;
  if(on) a.textContent='Starting…'; else b.textContent='Saving video…';
  try{ const j=await post(on?'/api/stream/start':'/api/stream/stop'); toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok);
    if(on&&j.ok){ streamOn=true; vmode='mjpeg'; vfails=0; startView(); } if(!on){ streamOn=false; stopView(); } }
  catch(e){ toast('✗ Request failed — check the WiFi connection',false); }
  a.textContent='▶ START STREAM'; b.textContent='■ STOP STREAM'; streamBusy=false; await poll(); setTimeout(()=>{ if(curDir==='/videos') loadFiles(); },600); }

// ---------- actions ----------
async function post(url){ const r=await fetch(url,{method:'POST',cache:'no-store'}); return r.json(); }
let lastPhoto='';
async function takePhoto(){ const b=$('btnPhoto'); b.disabled=true; b.textContent='Saving…';
  try{ const j=await post('/api/photo'); toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok); if(j.ok){ showPhoto(j.file); if(curDir==='/photos') loadFiles(); } }
  catch(e){ toast('✗ Photo failed — connection problem',false); }
  b.disabled=false; b.textContent='📸 CAPTURE PHOTO'; }
function showPhoto(p){ if(!p||p===lastPhoto) return; lastPhoto=p; T('pName',p.split('/').pop()); const t=$('pThumb'); t.src='/api/file?path='+encodeURIComponent(p); t.style.display='block'; }
let recOn=false;
async function toggleRec(){ const b=$('btnRec'); b.disabled=true;
  try{ const j=await post(recOn?'/api/rec/stop':'/api/rec/start'); toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok); }
  catch(e){ toast('✗ Request failed — recording state will refresh',false); }
  b.disabled=false; poll(); setTimeout(()=>{ if(curDir==='/videos') loadFiles(); },800); }
let vBusy=false;
async function setPreset(p){ if(vBusy) return; vBusy=true; document.querySelectorAll('#segQ button').forEach(b=>b.disabled=true);
  toast(recOn?'Switching… current video is saved, recording continues in a new file':'Switching…',true);
  try{ const j=await post('/api/video?preset='+p); toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok); }catch(e){ toast('✗ Request failed',false); }
  document.querySelectorAll('#segQ button').forEach(b=>b.disabled=false); vBusy=false; await poll(); if(curDir==='/videos') loadFiles(); }
async function setFpsMode(m){ try{ const j=await post('/api/video?fps='+m); toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok); }catch(e){ toast('✗ Request failed',false); } poll(); }
async function setAuto(el){ const on=!el.classList.contains('on');
  try{ const j=await post('/api/autorec?on='+(on?1:0)); toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok); }catch(e){ toast('✗ setting failed',false); } poll(); }
async function setCtl(v,val){ const big=(v==='framesize'); if(big&&recOn) toast('Switching… current video is saved, recording continues in a new file',true);
  try{ const r=await fetch('/control?var='+v+'&val='+val); const j=await r.json(); if(!j.ok) toast('✗ '+j.msg,false); else if(big) toast('✓ Resolution changed',true); }catch(e){ toast('✗ setting failed',false); }
  await poll(); if(big&&curDir==='/videos') loadFiles(); }
function flip(v,el){ setCtl(v, el.classList.contains('on')?0:1); }

// ---------- status polling ----------
let fails=0, pollBusy=false;
async function poll(){ if(pollBusy) return; pollBusy=true;
  try{ const r=await fetch('/api/status',{cache:'no-store'}); const d=await r.json(); fails=0; lastD=d; render(d); $('pill').dataset.s='ok'; T('pillTxt','ONLINE · '+d.net.ip); }
  catch(e){ if(++fails>=3){ $('pill').dataset.s='lost'; T('pillTxt','RECONNECTING'); } }
  pollBusy=false; }
setInterval(poll,1000); poll();

let setInit=false;
function st(id,txt,state){ const e=$(id); if(e.dataset.s!==state) e.dataset.s=state; const b=e.lastElementChild; if(b.textContent!==txt) b.textContent=txt; }
function upt(s){ return s>=3600?Math.floor(s/3600)+' h '+Math.floor(s%3600/60)+' min':(s>=60?Math.floor(s/60)+' min':s+' s'); }
function render(d){
  const c=d.cam, s=d.sd, r=d.rec, n=d.net, v=d.stream;
  // stream session (server is the truth: survives page reloads / other phones)
  streamOn=!!v.on;
  if(!streamBusy){ if(streamOn) startView(); else if(viewing) stopView(); }
  if(!streamBusy){ $('btnStart').disabled=streamOn||!c.ok; $('btnStop').disabled=!streamOn; }
  $('ovl').style.display=streamOn?'':'none';
  recOn=!!r.on;
  st('stStream',streamOn?'ON':'OFF',streamOn?'on':'');
  st('stRec',recOn?'ON':(s.ok?'OFF':'DISABLED'),recOn?'rec':(streamOn&&!s.ok?'warn':''));
  st('stSd',s.ok?(s.low?'ALMOST FULL':'INSERTED'):'NO CARD',s.ok?(s.low?'warn':'on'):'err');
  $('sdBan').style.display=s.ok?'none':'';
  // camera + network
  badge('bCam',c.ok?'ONLINE':'ERROR',c.ok?'b-ok':'b-err');
  T('cSensor',c.sensor); T('cRes',c.fs+' '+c.w+'×'+c.h); T('cQ',c.q); T('cFps',c.fps.toFixed(1)+' fps'+(c.drop?' · '+c.drop+' dropped':''));
  T('cView',streamOn?(v.viewers+' / '+v.max+(viewing&&vmode==='snap'?' · this phone: snapshots':'')):'stream off'); T('cMem',(d.heap/1024|0)+' KB RAM · '+(d.psram/1048576).toFixed(1)+' MB PSRAM');
  badge('bNet',n.sta?'WIFI JOINED':(n.ap?'ACCESS POINT':'RECONNECTING'),n.sta||n.ap?'b-ok':'b-warn');
  T('cNet',n.mode); T('cSsid',n.sta?n.ssid:'—'); T('cIp',n.ip+(n.sta&&n.apip?' (+ '+n.apip+')':'')); T('cSig',n.sta?(n.rssi+' dBm'):(n.clients>=0?n.clients+' connected':'—')); T('cUp',upt(d.up));
  T('wMode',n.mode); T('wSsid',n.sta?n.ssid:'—'); T('wAddr','http://'+n.ip+(n.mdns?'  ·  http://'+n.mdns:'')); T('wSig',n.sta?(n.rssi+' dBm'+(n.rssi>-60?' (good)':(n.rssi>-72?' (ok)':' (weak)'))):'—');
  T('wHint',n.ap&&n.nets.length&&!n.sta?'⚠ Saved network(s) not reachable — retrying every 30 s (paused while a phone is connected to RescueCam).':(n.ap&&!n.nets.length?'Tip: add your home WiFi or your phone\'s hotspot here.':''));
  const sk=JSON.stringify([n.nets,n.cur]);
  if(render._sk!==sk){ render._sk=sk; $('wSaved').innerHTML=n.nets.length?n.nets.map((x,i)=>'<div class="srow"><span>'+esc(x)+(i===n.cur?'<em>● CONNECTED</em>':'')+'</span><button class="btn sm" onclick="wifiForget('+i+')">✕ Remove</button></div>').join(''):'<div class="note">none — camera works as access point RescueCam</div>'; }
  T('ovRes',c.w+'×'+c.h+(streamOn&&v.lim?' · ≤'+v.lim+' fps':' · '+c.fps.toFixed(0)+' fps'));
  // quality mode + live data use (from bytes the camera really sent)
  const vd=d.video; T('cMode',vd.name+' · '+['Data saver','Normal','Max'][vd.fpsMode]+(v.lim?' ('+v.lim+' fps)':''));
  document.querySelectorAll('#segQ button').forEach(b=>b.classList.toggle('on',+b.dataset.p===vd.preset));
  document.querySelectorAll('#segF button').forEach(b=>b.classList.toggle('on',+b.dataset.f===vd.fpsMode));
  T('fNorm',[10,12,15][vd.preset]?([10,12,15][vd.preset]+' fps'):'12 fps');
  const now=performance.now();
  if(streamOn){ if(render._s!=null&&v.sent>=render._s&&now>render._t){ const r=(v.sent-render._s)/((now-render._t)/1000); render._r=render._r==null?r:render._r*0.6+r*0.4; }
    render._s=v.sent; render._t=now;
    T('dUse',(v.viewers||render._r>100)?((render._r||0)/1024).toFixed(0)+' KB/s ≈ '+((render._r||0)*3600/1048576).toFixed(0)+' MB/h · session '+gb(v.sent):'no viewer · session '+gb(v.sent)); }
  else { render._s=null; render._r=null; T('dUse','stream off'); }
  // SD
  badge('bSd',s.ok?'INSERTED':'NOT INSERTED',s.ok?(s.low?'b-warn':'b-ok'):'b-err');
  T('sType',s.ok?s.type:'—'); T('sCap',s.ok?gb(s.total):'—'); T('sUsed',s.ok?gb(s.used):'—'); T('sFree',s.ok?gb(s.total-s.used):'—');
  const pct=s.ok&&s.total?s.used/s.total*100:0; $('sBar').style.width=pct.toFixed(1)+'%'; $('sBar').className='fill'+(s.low?' low':'');
  T('sCount',s.ok?(s.photos+' / '+s.videos):'—');
  T('sWarn',!s.ok?('⚠ SD CARD NOT INSERTED / RECORDING DISABLED'+(s.err&&s.err!=='no card / not FAT32'?' ('+s.err+')':'')+' — live stream still works'):(s.low?'⚠ SD card almost full — delete old videos':''));
  // recording
  badge('bRec',recOn?'RECORDING':(s.ok?'OFF':'DISABLED'),recOn?'b-rec':(s.ok?'b-gray':'b-err'));
  T('rDur',hms(recOn?r.dur:0)); T('rFile',recOn?r.file:'—'); T('rFr',recOn?(r.frames+' · '+r.fps.toFixed(1)+' fps'):'—'); T('rSize',recOn?gb(r.bytes):'—');
  T('rAuto',v.auto?'ON (starts with the stream)':'OFF (live view only)'); T('rLast',r.last?r.last.split('/').pop():'—');
  T('rNote',!s.ok?'SD CARD NOT INSERTED / RECORDING DISABLED'+(v.wait?' — recording starts as soon as a card is inserted':''):(v.note||r.msg||''));
  $('recBar').style.display=streamOn&&s.ok?'':'none';
  const b=$('btnRec'); if(!b.disabled){ b.textContent=recOn?'⏹ Stop recording (keep stream)':'⏺ Record again'; b.className='btn sm'+(recOn?' stop':' rec'); }
  $('btnPhoto').disabled=!s.ok||!c.ok;
  $('vbox').classList.toggle('rec-on',recOn); T('ovRec',recOn?'REC '+hms(r.dur):'LIVE');
  badge('bPh',s.ok&&c.ok?'READY':'UNAVAILABLE',s.ok&&c.ok?'b-ok':'b-err');
  if(d.photo) showPhoto(d.photo);
  // settings (fill once, lock size/quality while recording)
  if(!setInit){ setInit=true; $('setBri').value=c.bri; $('setCon').value=c.con; $('setSat').value=c.sat; }
  if(document.activeElement!==$('setFs')&&$('setFs').value!=String(c.fsId)) $('setFs').value=c.fsId;   // follows the presets
  if(document.activeElement!==$('setQ')&&$('setQ').value!=String(c.q)) $('setQ').value=c.q;
  $('setHm').classList.toggle('on',!!c.hm); $('setVf').classList.toggle('on',!!c.vf); $('setAuto').classList.toggle('on',!!v.auto);
  if(d.rst&&/CRASH|WATCHDOG|BROWNOUT/.test(d.rst)&&!render._w){ render._w=1; toast('⚠ Camera restarted because of: '+d.rst,false); }
}

// ---------- file manager ----------
let curDir='/photos';
function setDir(el){ document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('on',t===el)); curDir=el.dataset.d; loadFiles(); }
function ftype(n,d){ if(d) return 'folder'; const e=n.split('.').pop().toLowerCase(); return e==='jpg'||e==='jpeg'?'JPEG photo':(e==='avi'?'AVI video (MJPEG)':e.toUpperCase()+' file'); }
async function loadFiles(){ const tb=$('flist');
  try{ const j=await (await fetch('/api/files?dir='+encodeURIComponent(curDir),{cache:'no-store'})).json();
    if(!j.ok){ tb.innerHTML='<tr><td colspan="5" class="muted">SD card not available</td></tr>'; return; }
    const f=j.files.sort((a,b)=>(b.d-a.d)||(b.t-a.t)||b.n.localeCompare(a.n));
    if(!f.length){ tb.innerHTML='<tr><td colspan="5" class="muted">No files yet</td></tr>'; return; }
    tb.innerHTML=f.map(x=>{ const p=encodeURIComponent(x.p), jpg=/\.jpe?g$/i.test(x.n), avi=/\.avi$/i.test(x.n), del=(x.p.startsWith('/photos/')||x.p.startsWith('/videos/'));
      const date=x.t?new Date(x.t*1000).toLocaleString():'—';
      let a='';
      if(x.rec) a='<span class="badge b-rec"><i></i>RECORDING</span>';
      else if(!x.d){
        if(jpg) a+='<button class="btn sm" onclick="viewPhoto(\''+esc(x.p)+'\')">View</button> ';
        if(avi) a+='<button class="btn sm" onclick="playVideo(\''+esc(x.p)+'\')">▶ Play</button> ';
        a+='<a class="btn sm" href="/api/file?dl=1&path='+p+'">Download</a> ';
        if(del) a+='<button class="btn sm" onclick="delFile(\''+esc(x.p)+'\')">Delete</button>';
      }
      return '<tr><td class="n">'+esc(x.n)+'</td><td class="hide-s muted">'+ftype(x.n,x.d)+'</td><td class="mono">'+(x.d?'—':gb(x.s))+'</td><td class="hide-s muted">'+date+'</td><td class="a">'+a+'</td></tr>'; }).join('');
  }catch(e){ tb.innerHTML='<tr><td colspan="5" class="muted">Could not load file list</td></tr>'; } }
async function delFile(p){ if(!confirm('Delete '+p+' ?')) return; try{ const j=await post('/api/delete?path='+encodeURIComponent(p)); toast((j.ok?'✓ ':'✗ ')+j.msg+' — '+p,j.ok); }catch(e){ toast('✗ delete failed',false); } loadFiles(); poll(); }
function openModal(title,src,dl){ T('mTitle',title); $('mImg').src=src; $('mDl').href=dl; $('modal').classList.add('on'); }
function closeModal(){ $('mImg').src=''; $('modal').classList.remove('on'); }
function viewPhoto(p){ if(!p) return; const e=encodeURIComponent(p); openModal(p,'/api/file?path='+e,'/api/file?dl=1&path='+e); }
function playVideo(p){ const e=encodeURIComponent(p); openModal('▶ '+p+' (MJPEG playback)','/api/play?path='+e+'&t='+Date.now(),'/api/file?dl=1&path='+e); }
document.addEventListener('keydown',e=>{ if(e.key==='Escape') closeModal(); });

// ---------- WiFi setup ----------
async function wifiScan(){ const b=$('wScan'); b.disabled=true; b.textContent='Scanning…';
  try{ const j=await (await fetch('/api/wifi/scan',{cache:'no-store'})).json(); const l=$('wList');
    l.innerHTML='<option value="">'+(j.nets.length?'— choose —':'no networks found')+'</option>'+j.nets.sort((a,b)=>b.r-a.r).map(n=>'<option value="'+esc(n.s)+'">'+esc(n.s)+'  ('+n.r+' dBm'+(n.lock?' 🔒':'')+')</option>').join('');
  }catch(e){ toast('✗ Scan failed',false); }
  b.disabled=false; b.textContent='📶 Scan'; }
$('wList').onchange=()=>{ if($('wList').value) $('wName').value=$('wList').value; };
async function wifiSave(){ const ss=$('wName').value.trim(), pw=$('wPass').value;
  if(!ss){ toast('✗ Choose or type the WiFi / hotspot name',false); return; }
  if(pw&&pw.length<8){ toast('✗ WiFi passwords have at least 8 characters',false); return; }
  if(!confirm('Save "'+ss+'" and restart the camera to join it?')) return;
  try{ const r=await fetch('/api/wifi/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ssid='+encodeURIComponent(ss)+'&pass='+encodeURIComponent(pw)}); const j=await r.json();
    toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok);
    if(j.ok) setTimeout(()=>alert('Camera is restarting.\n\n1) Connect this phone/laptop to "'+ss+'" (for a phone hotspot: just keep the hotspot ON)\n2) Open the camera IP (hotspot/router device list shows "rescuecam", or Serial Monitor)\n   iPhone / laptop: http://rescuecam.local\n\nIf it cannot join, WiFi "RescueCam" comes back (http://192.168.4.1).'),300);
  }catch(e){ toast('✗ Save failed',false); } }
async function wifiForget(i){ const nm=lastD&&lastD.net.nets[i]; if(!confirm('Remove saved network "'+nm+'"? (used from the next restart)')) return;
  try{ const j=await post('/api/wifi/forget?i='+i); toast((j.ok?'✓ ':'✗ ')+j.msg,j.ok); }catch(e){ toast('✗ failed',false); } poll(); }
loadFiles();
</script>
</body>
</html>)rawliteral";
