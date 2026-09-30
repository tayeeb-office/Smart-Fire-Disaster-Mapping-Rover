/*
 * ============================================================================
 *  RESCUE CAM - ESP32-S3-CAM camera system
 *  START STREAM -> live stream + automatic MJPEG/AVI recording to microSD
 *  STOP STREAM  -> stream off + video finalized. No SD -> stream still works.
 *  + photo capture, web file manager, camera settings, WiFi manager
 *  (camera-only system, no radar).
 * ============================================================================
 *  Board : ESP32-S3 WROOM CAM (Freenove-type, ESP32-S3-EYE camera pinout,
 *          CAMERA_MODEL_CUSTOM in board_config.h / camera_pins.h)
 *  Camera: OV2640 (auto-detected; OV3660/OV5640 also handled)
 *  SD    : on-board slot, SD_MMC 1-bit, CLK 39 / CMD 38 / D0 40
 *
 *  Files in this sketch folder:
 *    RescueCam.ino   setup/loop, camera init, WiFi, status JSON
 *    cam_system.h    settings + shared declarations
 *    frame_hub.cpp   one camera reader -> stream/recorder/photo (PSRAM slots)
 *    session.cpp     START/STOP STREAM session + automatic recording
 *    storage.cpp     microSD mount/info/names/path safety/clock
 *    recorder.cpp    MJPEG-in-AVI recorder task + photo capture
 *    avi_writer.h    AVI (RIFF) writer
 *    app_httpd.cpp   web server + API
 *    dashboard.h     web dashboard (HTML/CSS/JS)
 *    board_config.h, camera_pins.h   (unchanged from your project)
 *
 *  Arduino IDE: ESP32 core 3.x - Board "ESP32S3 Dev Module", PSRAM "OPI PSRAM",
 *  Flash Size 8MB (or your module size), Partition "Huge APP (3MB No OTA/1MB SPIFFS)",
 *  USB CDC On Boot as you normally use. No extra libraries required.
 * ============================================================================
 */
#include "esp_camera.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "esp_system.h"
#include "board_config.h"
#include "cam_system.h"

static DNSServer dns;
static bool apMode = true;
static const char* resetReason = "unknown";

// ============================== CAMERA ==============================
static const char* fsName(framesize_t f) {
  switch (f) {
    case FRAMESIZE_QVGA: return "QVGA"; case FRAMESIZE_VGA: return "VGA"; case FRAMESIZE_SVGA: return "SVGA";
    case FRAMESIZE_XGA: return "XGA"; case FRAMESIZE_HD: return "HD"; case FRAMESIZE_SXGA: return "SXGA";
    case FRAMESIZE_UXGA: return "UXGA"; default: return "other";
  }
}
bool cameraFrameSize(uint16_t& w, uint16_t& h, const char** name) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return false;
  framesize_t f = s->status.framesize;
  if (f >= FRAMESIZE_INVALID) return false;
  w = resolution[f].width; h = resolution[f].height;
  if (name) *name = fsName(f);
  return true;
}
const char* cameraSensorName() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return "none";
  switch (s->id.PID) {
    case OV2640_PID: return "OV2640"; case OV3660_PID: return "OV3660"; case OV5640_PID: return "OV5640";
    case OV7725_PID: return "OV7725"; default: return "unknown";
  }
}

// Same pins/clock/format as your CameraWebServer-based code (pins from camera_pins.h).
// LOW-FIRST: the sensor is switched to LOW / FAST (320x240, JPEG q20) right here,
// before the frame hub starts - the very first frame any browser or the recorder
// ever gets is a small 320x240 JPEG. config.frame_size below only sizes the two
// DMA/JPEG buffers in PSRAM (the driver cannot enlarge them later), so MEDIUM /
// HIGH / custom sizes up to UXGA can be selected without re-initializing.
static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;                // 20 MHz XCLK
  config.frame_size = FRAMESIZE_UXGA;            // buffer SIZE only (2 x ~384 KB PSRAM), see above
  config.pixel_format = PIXFORMAT_JPEG;          // JPEG straight from the sensor (needed for stream/AVI)
  config.grab_mode = CAMERA_GRAB_LATEST;         // always the newest frame -> low latency, no backlog
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = PRESET_LOW_Q;
  config.fb_count = 2;                           // 2 = capture next frame while one is copied (no more needed)
  if (!psramFound()) {                           // fallback without PSRAM (should not happen on this board)
    config.frame_size = FRAMESIZE_VGA;           // small DRAM buffer: LOW / MEDIUM only
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_count = 1;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("[CAM] init failed 0x%x\n", err); return false; }
  sensor_t* s = esp_camera_sensor_get();
  if (!s) { Serial.println("[CAM] no sensor"); return false; }
  if (s->id.PID == OV3660_PID) { s->set_vflip(s, 1); s->set_brightness(s, 1); s->set_saturation(s, -2); }
  s->set_framesize(s, PRESET_LOW_FS);            // LOW / FAST: 320x240 - fast first picture, little data
  s->set_quality(s, PRESET_LOW_Q);
#if defined(CAMERA_MODEL_ESP32S3_EYE) || defined(CAMERA_MODEL_CUSTOM)
  s->set_vflip(s, 1);                            // as in your code (S3-EYE orientation)
#endif
  // drop the frames captured before the switch (they have the buffer-init size)
  for (int i = 0; i < 3; i++) { camera_fb_t* fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); }
  return true;
}

// ============================== WIFI ==============================
// The camera never NEEDS another network: it always works as its own access
// point "RescueCam" (http://192.168.4.1). Additionally it can join up to 3
// saved networks - e.g. home WiFi, your PHONE'S HOTSPOT (so the phone keeps
// its mobile-data internet while you watch the camera), a 4G pocket router.
//   boot:   scan -> join the preferred / strongest saved network that is on air
//           none reachable -> access point, saved networks retried every 30 s
//   joined: http://<ip>, http://rescuecam.local, name "rescuecam" in the
//           router / hotspot "connected devices" list
//   joined network lost > 60 s -> access point comes back as well (you can
//           always reach the camera), while it keeps trying to rejoin.
struct Net { char ssid[33]; char pass[65]; };
static Net nets[MAX_SAVED_NETS];
static int netCount = 0, prefNet = -1, curNet = -1, tryNet = 0;
static bool staUp = false, mdnsUp = false;
static uint32_t restartAt = 0, staLostAt = 0;

static void jsonEscape(const char* in, char* out, size_t n) {
  size_t o = 0;
  for (; *in && o + 2 < n; in++) { if (*in == '"' || *in == '\\') out[o++] = '\\'; out[o++] = (*in < 32) ? ' ' : *in; }
  out[o] = 0;
}
static void loadNets() {
  netCount = 0; prefNet = -1;
  Preferences p;
  if (p.begin("wifi", true)) {
    for (int i = 0; i < MAX_SAVED_NETS; i++) {
      char ks[12], kp[12];
      snprintf(ks, sizeof(ks), "s%d", i); snprintf(kp, sizeof(kp), "p%d", i);
      if (!p.isKey(ks)) continue;
      String ss = p.getString(ks, ""), pw = p.isKey(kp) ? p.getString(kp, "") : String();
      if (!ss.length()) continue;
      strlcpy(nets[netCount].ssid, ss.c_str(), 33); strlcpy(nets[netCount].pass, pw.c_str(), 65);
      netCount++;
    }
    if (!netCount && p.isKey("ssid")) {                    // settings saved by the previous firmware version
      String ss = p.getString("ssid", ""), pw = p.isKey("pass") ? p.getString("pass", "") : String();
      if (ss.length()) { strlcpy(nets[0].ssid, ss.c_str(), 33); strlcpy(nets[0].pass, pw.c_str(), 65); netCount = 1; }
    }
    if (p.isKey("pref")) prefNet = p.getChar("pref", -1);
    p.end();
  }
  if (strlen(JOIN_SSID) && netCount < MAX_SAVED_NETS) {   // default from cam_system.h
    bool have = false;
    for (int i = 0; i < netCount; i++) if (!strcmp(nets[i].ssid, JOIN_SSID)) have = true;
    if (!have) { strlcpy(nets[netCount].ssid, JOIN_SSID, 33); strlcpy(nets[netCount].pass, JOIN_PASS, 65); netCount++; }
  }
  if (prefNet >= netCount) prefNet = -1;
}
static bool storeNets() {
  Preferences p;
  if (!p.begin("wifi", false)) return false;
  p.clear();
  for (int i = 0; i < netCount; i++) {
    char ks[12], kp[12];
    snprintf(ks, sizeof(ks), "s%d", i); snprintf(kp, sizeof(kp), "p%d", i);
    p.putString(ks, nets[i].ssid); p.putString(kp, nets[i].pass);
  }
  if (prefNet >= 0) p.putChar("pref", (int8_t)prefNet);
  p.end();
  return true;
}
static void startMdns() {
  if (mdnsUp) return;
  if (MDNS.begin(MDNS_NAME)) { MDNS.addService("http", "tcp", 80); mdnsUp = true; }
}
static void startAP() {
  WiFi.mode(WIFI_AP_STA);                       // AP for you + STA for scanning / retrying saved networks
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);                 // core auto-reconnect rescans non-stop while a network is
                                                // missing -> AP clients would stutter; wifiService() retries every 30 s
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4);
  delay(100);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());          // phones' "internet check" resolves to us
  apMode = true;
  Serial.printf("[WiFi] access point \"%s\" (password %s)  ->  http://%s\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}
static void printJoined() {
  Serial.println("==================================================");
  Serial.printf("[WiFi] CONNECTED to \"%s\"  (signal %d dBm)\n", curNet >= 0 ? nets[curNet].ssid : "?", (int)WiFi.RSSI());
  Serial.printf("[WiFi] OPEN:  http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[WiFi]   or:  http://%s.local   (device name \"%s\")\n", MDNS_NAME, MDNS_NAME);
  Serial.println("==================================================");
}
static void startWiFi() {
  WiFi.persistent(false);
  WiFi.setHostname(MDNS_NAME);                  // shown in router / phone-hotspot device lists
  loadNets();
  if (netCount) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                       // no power-save gaps -> smooth video
    WiFi.setAutoReconnect(true);
    // order: preferred (last saved) first, then by signal; only networks that are on air
    int rssi[MAX_SAVED_NETS], order[MAX_SAVED_NETS], cnt = 0;
    int found = WiFi.scanNetworks(false, true);   // ~2-3 s (hidden networks included)
    for (int i = 0; i < netCount; i++) {
      rssi[i] = found < 0 ? -90 : -1000;          // scan failed -> just try all
      for (int k = 0; k < found; k++) if (WiFi.SSID(k) == nets[i].ssid && WiFi.RSSI(k) > rssi[i]) rssi[i] = WiFi.RSSI(k);
    }
    WiFi.scanDelete();
    for (int i = 0; i < netCount; i++) if (rssi[i] > -1000) order[cnt++] = i;
    for (int a = 0; a < cnt; a++) for (int b = a + 1; b < cnt; b++) {
      int ia = order[a], ib = order[b];
      bool bFirst = (ib == prefNet) || (ia != prefNet && rssi[ib] > rssi[ia]);
      if (bFirst) { order[a] = ib; order[b] = ia; }
    }
    for (int a = 0; a < cnt; a++) {
      int i = order[a];
      WiFi.begin(nets[i].ssid, nets[i].pass);
      Serial.printf("[WiFi] connecting to \"%s\" (%d dBm)", nets[i].ssid, rssi[i]);
      uint32_t t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < JOIN_TIMEOUT_MS) { delay(250); Serial.print("."); }
      Serial.println();
      if (WiFi.status() == WL_CONNECTED) {
        apMode = false; staUp = true; curNet = tryNet = i;
        startMdns();
        printJoined();
        return;
      }
      WiFi.disconnect(false);
      delay(100);
    }
    Serial.printf("[WiFi] none of the %d saved network(s) reachable -> access point (retrying every 30 s)\n", netCount);
    WiFi.disconnect(true);
    delay(100);
  } else {
    Serial.println("[WiFi] no WiFi network saved -> access point only (add home WiFi / phone hotspot in the dashboard)");
  }
  startAP();
}
// called from loop(): reconnect / retry / AP fallback / restart after saving settings
static void wifiService() {
  static uint32_t lastTry = 0;
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !staUp) {                    // (re)connected
    staUp = true; staLostAt = 0; curNet = tryNet;
    startMdns();
    printJoined();
  } else if (!connected && staUp) {
    staUp = false; staLostAt = millis();
    Serial.println("[WiFi] joined network lost -> reconnecting (stream session / recording continue)");
  }
  if (!connected && !apMode && staLostAt && millis() - staLostAt > STA_LOST_AP_MS) {
    Serial.println("[WiFi] still no network -> starting access point RescueCam as well");
    startAP();                                  // STA keeps retrying in the background
  }
  // Retry. STA-only mode: the core's auto-reconnect is active; this is only a
  // safety kick every 20 s. AP mode: one join attempt every 30 s, cycling through
  // the saved networks - skipped while a phone is connected to RescueCam (the
  // channel scan of a join attempt would make its stream stutter).
  if (!connected && netCount && millis() - lastTry > (apMode ? 30000UL : 20000UL)) {
    lastTry = millis();
    if (!apMode) WiFi.reconnect();
    else if (WiFi.softAPgetStationNum() == 0) {
      tryNet = (tryNet + 1) % netCount;
      WiFi.begin(nets[tryNet].ssid, nets[tryNet].pass);
    }
  }
  if (restartAt && (int32_t)(millis() - restartAt) >= 0) ESP.restart();
}
size_t wifiScanJson(char* out, size_t n) {
  int cnt = WiFi.scanNetworks(false, false);    // ~2-4 s
  size_t o = snprintf(out, n, "{\"ok\":%s,\"nets\":[", cnt >= 0 ? "true" : "false");
  int shown = 0;
  for (int i = 0; i < cnt && shown < 25 && o < n - 120; i++) {
    String ss = WiFi.SSID(i);
    if (!ss.length()) continue;
    bool dup = false;                           // same name on several access points -> show once (strongest)
    for (int k = 0; k < i && !dup; k++) if (WiFi.SSID(k) == ss && WiFi.RSSI(k) >= WiFi.RSSI(i)) dup = true;
    if (dup) continue;
    char esc[70];
    jsonEscape(ss.c_str(), esc, sizeof(esc));
    o += snprintf(out + o, n - o, "%s{\"s\":\"%s\",\"r\":%d,\"lock\":%d}", shown ? "," : "", esc, (int)WiFi.RSSI(i), WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    shown++;
  }
  WiFi.scanDelete();
  o += snprintf(out + o, n - o, "]}");
  return o < n ? o : 0;
}
int wifiSaveNet(const char* ssid, const char* pass) {
  if (!ssid || !ssid[0] || strlen(ssid) > 32 || !pass || strlen(pass) > 64) return 2;
  int idx = -1;
  for (int i = 0; i < netCount; i++) if (!strcmp(nets[i].ssid, ssid)) idx = i;   // existing -> new password
  if (idx < 0) {
    if (netCount >= MAX_SAVED_NETS) return 1;
    idx = netCount++;
    strlcpy(nets[idx].ssid, ssid, 33);
  }
  strlcpy(nets[idx].pass, pass, 65);
  prefNet = idx;                                // "Save & connect" -> try this one first after the restart
  if (!storeNets()) return 2;
  Serial.printf("[WiFi] saved \"%s\" -> restarting to connect\n", ssid);
  restartAt = millis() + 1500;                  // let the HTTP reply reach the browser first
  return 0;
}
bool wifiForgetNet(int i) {
  if (i < 0 || i >= netCount) return false;
  Serial.printf("[WiFi] removed \"%s\"\n", nets[i].ssid);
  for (int k = i; k < netCount - 1; k++) nets[k] = nets[k + 1];
  netCount--;
  if (prefNet == i) prefNet = -1; else if (prefNet > i) prefNet--;
  if (curNet == i) curNet = -1; else if (curNet > i) curNet--;
  if (tryNet >= netCount) tryNet = 0;
  return storeNets();
}
void netStatus(char* mode, size_t mn, char* ip, size_t in, int& rssi, int& clients) {
  bool c = WiFi.status() == WL_CONNECTED;
  // (network names are reported separately, JSON-escaped)
  if (!apMode) strlcpy(mode, c ? "Joined WiFi" : "Reconnecting", mn);
  else strlcpy(mode, c ? "Joined WiFi + AP " AP_SSID : "AP " AP_SSID, mn);
  strlcpy(ip, c ? WiFi.localIP().toString().c_str() : (apMode ? WiFi.softAPIP().toString().c_str() : "-"), in);
  rssi = c ? WiFi.RSSI() : 0;
  clients = apMode ? WiFi.softAPgetStationNum() : -1;
}

// ============================== STATUS JSON ==============================
size_t statusJson(char* out, size_t n) {
  SdInfo sd; storageGetInfo(sd);
  RecInfo ri; recorderGetInfo(ri);
  uint16_t w = 0, h = 0; const char* fs = "-";
  cameraFrameSize(w, h, &fs);
  sensor_t* s = esp_camera_sensor_get();
  char mode[48], ip[20], lastPhoto[48];
  int rssi, clients;
  netStatus(mode, sizeof(mode), ip, sizeof(ip), rssi, clients);
  photoLast(lastPhoto, sizeof(lastPhoto));
  bool c = WiFi.status() == WL_CONNECTED;
  char ssidEsc[70] = "";                          // network joined (or being tried), JSON-escaped
  int shownNet = c ? curNet : (netCount ? tryNet : -1);
  if (shownNet >= 0 && shownNet < netCount) jsonEscape(nets[shownNet].ssid, ssidEsc, sizeof(ssidEsc));
  char netList[240]; size_t nl = 0;               // saved networks
  netList[0] = 0;
  for (int i = 0; i < netCount && nl < sizeof(netList) - 72; i++) {
    char e2[70]; jsonEscape(nets[i].ssid, e2, sizeof(e2));
    nl += snprintf(netList + nl, sizeof(netList) - nl, "%s\"%s\"", i ? "," : "", e2);
  }
  char note[160], noteEsc[330];
  sessionNote(note, sizeof(note)); jsonEscape(note, noteEsc, sizeof(noteEsc));
  char recMsgEsc[140]; jsonEscape(ri.msg, recMsgEsc, sizeof(recMsgEsc));
  char sdErrEsc[80]; jsonEscape(sd.err, sdErrEsc, sizeof(sdErrEsc));
  int len = snprintf(out, n,
    "{\"cam\":{\"ok\":%d,\"sensor\":\"%s\",\"fs\":\"%s\",\"fsId\":%d,\"w\":%u,\"h\":%u,\"q\":%d,\"fps\":%.1f,\"drop\":%lu,"
    "\"bri\":%d,\"con\":%d,\"sat\":%d,\"hm\":%d,\"vf\":%d},"
    "\"stream\":{\"on\":%d,\"viewers\":%d,\"max\":%d,\"auto\":%d,\"wait\":%d,\"note\":\"%s\",\"sent\":%llu,\"lim\":%d},"
    "\"video\":{\"preset\":%d,\"name\":\"%s\",\"fpsMode\":%d},"
    "\"net\":{\"mode\":\"%s\",\"ip\":\"%s\",\"apip\":\"%s\",\"rssi\":%d,\"clients\":%d,\"ap\":%d,\"sta\":%d,\"ssid\":\"%s\",\"mdns\":\"%s\",\"nets\":[%s],\"cur\":%d},"
    "\"sd\":{\"ok\":%d,\"type\":\"%s\",\"total\":%llu,\"used\":%llu,\"photos\":%lu,\"videos\":%lu,\"low\":%d,\"err\":\"%s\"},"
    "\"rec\":{\"on\":%d,\"file\":\"%s\",\"frames\":%lu,\"bytes\":%llu,\"dur\":%lu,\"fps\":%.1f,\"w\":%u,\"h\":%u,\"last\":\"%s\",\"msg\":\"%s\"},"
    "\"photo\":\"%s\",\"clock\":%d,\"up\":%lu,\"heap\":%lu,\"psram\":%lu,\"rst\":\"%s\"}",
    camOk ? 1 : 0, cameraSensorName(), fs, s ? (int)s->status.framesize : -1, w, h, s ? s->status.quality : 0, hubFps(), (unsigned long)hubDropped(),
    s ? s->status.brightness : 0, s ? s->status.contrast : 0, s ? s->status.saturation : 0, s ? s->status.hmirror : 0, s ? s->status.vflip : 0,
    streamIsOn() ? 1 : 0, streamViewers(), STREAM_MAX_VIEWERS, autoRecGet() ? 1 : 0, autoRecPending() ? 1 : 0, noteEsc,
    (unsigned long long)streamBytesSent(), streamFpsLimit(),
    videoPreset(), videoPresetName(videoPreset()), videoFpsMode(),
    mode, ip, apMode ? WiFi.softAPIP().toString().c_str() : "", rssi, clients, apMode ? 1 : 0, c ? 1 : 0, ssidEsc, mdnsUp ? MDNS_NAME ".local" : "", netList, c ? curNet : -1,
    sd.ok ? 1 : 0, sd.type, (unsigned long long)sd.total, (unsigned long long)sd.used, (unsigned long)sd.photos, (unsigned long)sd.videos, sd.low ? 1 : 0, sdErrEsc,
    ri.active ? 1 : 0, ri.file, (unsigned long)ri.frames, (unsigned long long)ri.bytes, (unsigned long)ri.durMs, ri.fps, ri.w, ri.h, ri.lastFile, recMsgEsc,
    lastPhoto, clockValid() ? 1 : 0, (unsigned long)(millis() / 1000), (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram(), resetReason);
  return (len > 0 && (size_t)len < n) ? (size_t)len : 0;
}

// ============================== SETUP / LOOP ==============================
static const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power-on"; case ESP_RST_SW: return "software"; case ESP_RST_PANIC: return "CRASH";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT (weak power supply)"; case ESP_RST_EXT: return "reset button";
    case ESP_RST_USB: return "USB"; default: return "other";
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  resetReason = resetName(esp_reset_reason());
  Serial.println("\n==== RESCUE CAM - ESP32-S3-CAM camera system ====");
  Serial.printf("[BOOT] reset reason: %s | PSRAM: %s (%lu KB free)\n", resetReason, psramFound() ? "yes" : "NO - select OPI PSRAM", (unsigned long)(ESP.getFreePsram() / 1024));
  if (STATUS_LED_PIN >= 0) { pinMode(STATUS_LED_PIN, OUTPUT); digitalWrite(STATUS_LED_PIN, LOW); }

  bool cam = initCamera();
  Serial.printf("[CAM] %s  sensor: %s\n", cam ? "OK" : "FAILED - check ribbon cable / PSRAM setting", cam ? cameraSensorName() : "-");
  if (cam && !hubStart()) Serial.println("[CAM] frame hub could not start (PSRAM?)");

  startWiFi();
  startWebServer();
  storageBegin();                                // SD mounts in the background (never blocks boot)
  recorderBegin();
  sessionBegin();                                // stream starts OFF: press START STREAM in the dashboard
  Serial.println("[READY] open the address shown above in a browser, then press START STREAM (starts in LOW / FAST)");
}

void loop() {
  if (apMode) dns.processNextRequest();

  wifiService();                                 // reconnect / background retry / restart after WiFi setup
  sessionService();                              // auto-record when an SD card appears during streaming

  // on-board LED: blinks while recording
  if (STATUS_LED_PIN >= 0) digitalWrite(STATUS_LED_PIN, recorderActive() ? ((millis() / 500) & 1) : LOW);

  static uint32_t lastLog = 0;
  if (millis() - lastLog > 5000) {
    lastLog = millis();
    SdInfo sd; storageGetInfo(sd);
    RecInfo ri; recorderGetInfo(ri);
    Serial.printf("[STATUS] cam:%s %.1f fps %s | stream:%s %d viewer(s) | sd:%s free %.2f GB | rec:%s %lu frames | heap %lu KB psram %lu KB\n",
                  camOk ? "OK" : "FAIL", hubFps(), videoPresetName(videoPreset()), streamIsOn() ? "ON" : "off", streamViewers(), sd.ok ? "OK" : "NONE", (sd.total - sd.used) / 1e9,
                  ri.active ? "ON" : "off", (unsigned long)ri.frames, (unsigned long)(ESP.getFreeHeap() / 1024), (unsigned long)(ESP.getFreePsram() / 1024));
  }
  delay(10);
}
