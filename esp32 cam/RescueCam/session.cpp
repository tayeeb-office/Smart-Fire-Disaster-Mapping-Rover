// ============================================================================
//  Stream session - streaming and recording designed together
//
//   START STREAM -> live stream ON for all viewers
//                   + recording starts automatically if an SD card is present
//                     (and "auto-record" is enabled, default ON)
//                   + no SD card -> stream still works, recording stays
//                     disabled ("SD CARD NOT INSERTED / RECORDING DISABLED");
//                     if a card is inserted later, recording starts then
//   STOP STREAM  -> live stream OFF + the video file is finalized and closed
//
//  The session lives on the camera, not in the browser: if the phone loses
//  WiFi or the page is closed, recording continues until STOP STREAM.
//  If the recorder stops by itself (card pulled, write error) it is restarted
//  automatically when the card is back (max AUTO_REC_RETRIES per session).
//  A full card is never retried (it would only fail again).
//
//  Video quality: every START STREAM begins in LOW / FAST (320x240, small JPEGs)
//  so the first picture appears quickly even on a weak / mobile connection.
//  MEDIUM / HIGH (or a custom size) can be chosen while streaming. A video file
//  must keep one frame size, so a size change during recording closes the
//  current AVI properly and continues in a NEW file - nothing is lost.
// ============================================================================
#include "cam_system.h"
#include <Preferences.h>
#include <mutex>
#include <atomic>

namespace {
std::atomic<bool> on{false};
std::atomic<bool> pending{false};        // start recording as soon as the SD card is ready
std::atomic<bool> autoRec{AUTO_RECORD_DEFAULT};
std::mutex mtx;                          // serializes start/stop (web handlers + loop)
bool wasRec = false;                     // we started the recorder in this session
bool userStopped = false;                // recording stopped on purpose (don't auto-restart)
int  retries = 0;                        // auto-restarts after the recorder ended by itself
int  attempts = 0;                       // background start attempts (faulty card guard)
std::mutex noteMtx;                      // separate: status reads never wait for a slow stop
char note[160] = "";

void setNote(const char* m) { std::lock_guard<std::mutex> lk(noteMtx); strlcpy(note, m, sizeof(note)); }
void tryRecord(char* msg, size_t n);

struct Preset { framesize_t fs; int q; int fps; const char* name; };
const Preset PRESETS[3] = {
  {PRESET_LOW_FS,  PRESET_LOW_Q,  PRESET_LOW_FPS,  "LOW / FAST"},
  {PRESET_MED_FS,  PRESET_MED_Q,  PRESET_MED_FPS,  "MEDIUM"},
  {PRESET_HIGH_FS, PRESET_HIGH_Q, PRESET_HIGH_FPS, "HIGH"},
};
std::atomic<int> preset{VQ_LOW};         // camera boots in LOW (see initCamera)
std::atomic<int> fpsMode{FPSM_NORMAL};

// call with mtx held. Applies frame size + JPEG quality to the sensor.
// Frame size change while recording: finalize this file -> switch -> new file.
bool applyVideo(framesize_t fs, int q, char* msg, size_t n) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s || !camOk) { strlcpy(msg, "Camera not ready", n); return false; }
  bool sizeChange = s->status.framesize != fs;
  if (!sizeChange) {
    if (s->status.quality != q && s->set_quality(s, q) != 0) { strlcpy(msg, "Quality not supported", n); return false; }
    strlcpy(msg, "ok", n);
    return true;
  }
  bool rec = recorderActive();
  char m[112] = "";
  if (rec) recorderStop(m, sizeof(m));                // current AVI finalized + closed
  hubHold(true);                                      // no frame read while the sensor switches
  int r = s->set_framesize(s, fs);
  if (r == 0) s->set_quality(s, q);
  hubHold(false);                                     // first 2 frames afterwards are dropped
  if (r != 0) { strlcpy(msg, "Frame size not supported by this sensor", n); }
  else strlcpy(msg, "ok", n);
  if (rec) {                                          // continue recording in a new file
    char r2[160];
    tryRecord(r2, sizeof(r2));
    char nt[220];
    snprintf(nt, sizeof(nt), "Resolution changed - previous video saved, %.150s", r2);
    setNote(nt);
    Serial.printf("[STREAM] %s\n", nt);
  }
  return r == 0;
}

// call with mtx held. Writes a short result into msg.
void tryRecord(char* msg, size_t n) {
  SdInfo sd; storageGetInfo(sd);
  if (!sd.ok) { pending = true; strlcpy(msg, "SD CARD NOT INSERTED / RECORDING DISABLED", n); return; }
  char m[112];
  if (recorderStart(m, sizeof(m))) {
    pending = false; wasRec = true; userStopped = false;
    snprintf(msg, n, "Recording to %s", m);
  } else {
    storageGetInfo(sd);
    pending = !sd.ok;                    // card vanished meanwhile -> keep waiting for it
    snprintf(msg, n, "Recording not started: %s", m);
  }
}
}  // namespace

void sessionBegin() {
  Preferences p;
  if (p.begin("cam", true)) { autoRec = p.getBool("autorec", AUTO_RECORD_DEFAULT); p.end(); }
}
bool streamIsOn() { return on; }
bool autoRecGet() { return autoRec; }
bool autoRecPending() { return on && pending && autoRec; }
void sessionNote(char* out, size_t n) { std::lock_guard<std::mutex> lk(noteMtx); strlcpy(out, note, n); }

void autoRecSet(bool v) {
  autoRec = v;
  if (!v) pending = false;
  Preferences p;
  if (p.begin("cam", false)) { p.putBool("autorec", v); p.end(); }
}

bool streamStart(char* msg, size_t n) {
  std::lock_guard<std::mutex> lk(mtx);
  if (!camOk) { strlcpy(msg, "Camera not ready - check the camera ribbon cable", n); return false; }
  bool was = on.load();
  if (!was) {                                         // every new stream starts in LOW / FAST
    char vm[64];
    if (applyVideo(PRESETS[VQ_LOW].fs, PRESETS[VQ_LOW].q, vm, sizeof(vm))) preset = VQ_LOW;
    streamBytesReset();
  }
  on = true;
  retries = 0; attempts = 0; userStopped = false;
  char r[160];
  if (recorderActive()) strlcpy(r, "recording already running", sizeof(r));
  else if (autoRec) tryRecord(r, sizeof(r));
  else { pending = false; strlcpy(r, "auto-record is OFF (only live view)", sizeof(r)); }
  setNote(r);
  snprintf(msg, n, "Stream %s (%s) - %s", was ? "already ON" : "ON", videoPresetName(preset), r);
  Serial.printf("[STREAM] ON - %s\n", r);
  return true;
}

bool streamStop(char* msg, size_t n) {
  std::lock_guard<std::mutex> lk(mtx);
  on = false; pending = false; userStopped = true;
  if (recorderActive()) {
    char m[112];
    recorderStop(m, sizeof(m));          // waits until the AVI index/header are written and the file is closed
    snprintf(msg, n, "Stream OFF - video saved (%s)", m);
  } else strlcpy(msg, "Stream OFF", n);
  wasRec = false;
  setNote(msg);
  Serial.printf("[STREAM] %s\n", msg);
  return true;
}

bool sessionRecStart(char* msg, size_t n) {
  std::lock_guard<std::mutex> lk(mtx);
  if (!on) { strlcpy(msg, "Start the stream first (recording runs together with the stream)", n); return false; }
  if (recorderActive()) { strlcpy(msg, "Already recording", n); return false; }
  retries = 0; attempts = 0;
  char r[160];
  tryRecord(r, sizeof(r));
  setNote(r);
  strlcpy(msg, r, n);
  return recorderActive();
}

bool sessionRecStop(char* msg, size_t n) {
  std::lock_guard<std::mutex> lk(mtx);
  pending = false; userStopped = true;
  if (!recorderActive()) { strlcpy(msg, "Not recording", n); return false; }
  char m[112];
  recorderStop(m, sizeof(m));
  wasRec = false;
  snprintf(msg, n, "Recording stopped, stream continues (%s)", m);
  setNote(msg);
  return true;
}

void sessionService() {
  static uint32_t last = 0;
  if (millis() - last < 500) return;
  last = millis();
  std::unique_lock<std::mutex> lk(mtx, std::try_to_lock);
  if (!lk.owns_lock()) return;           // a web request is starting/stopping right now
  if (!on) return;
  bool rec = recorderActive();
  if (wasRec && !rec && !userStopped) {  // recorder ended by itself: card pulled / write error / full
    wasRec = false;
    RecInfo ri; recorderGetInfo(ri);
    bool full = strstr(ri.msg, "SD card full") != nullptr;   // only the real "card full" stop (not "card removed or full")
    if (autoRec && !full && retries < AUTO_REC_RETRIES) { retries++; pending = true; }
    char m[160];
    snprintf(m, sizeof(m), "%s%s", ri.msg, pending ? " - will resume when the SD card is ready" : "");
    setNote(m);
    Serial.printf("[STREAM] recorder ended: %s\n", m);
  }
  if (pending && autoRec && !rec) {
    SdInfo sd; storageGetInfo(sd);
    if (sd.ok && ++attempts > AUTO_REC_RETRIES + 1) {
      pending = false;
      setNote("Recording disabled - SD card keeps failing (check / replace the card)");
      Serial.println("[STREAM] SD keeps failing -> auto-record gives up for this session");
    } else if (sd.ok) {
      char r[160];
      tryRecord(r, sizeof(r));
      setNote(r);
      Serial.printf("[STREAM] SD ready - %s\n", r);
    }
  }
}

// ---------------- video quality ----------------
const char* videoPresetName(int p) { return (p >= 0 && p < 3) ? PRESETS[p].name : "CUSTOM"; }
int videoPreset() { return preset; }
int videoFpsMode() { return fpsMode; }
void videoFpsModeSet(int m) { if (m >= FPSM_SAVER && m <= FPSM_MAX) fpsMode = m; }
int streamFpsLimit() {
  int m = fpsMode;
  if (m == FPSM_SAVER) return FPS_DATA_SAVER;
  if (m == FPSM_MAX) return 0;
  int p = preset;
  return (p >= 0 && p < 3) ? PRESETS[p].fps : 12;
}
bool videoPresetSet(int p, char* msg, size_t n) {
  if (p < VQ_LOW || p > VQ_HIGH) { strlcpy(msg, "unknown preset", n); return false; }
  std::lock_guard<std::mutex> lk(mtx);
  char m[64];
  if (!applyVideo(PRESETS[p].fs, PRESETS[p].q, m, sizeof(m))) { strlcpy(msg, m, n); return false; }
  preset = p;
  snprintf(msg, n, "%s: %ux%u, JPEG quality %d, stream %d fps (Normal)", PRESETS[p].name,
           resolution[PRESETS[p].fs].width, resolution[PRESETS[p].fs].height, PRESETS[p].q, PRESETS[p].fps);
  Serial.printf("[VIDEO] %s\n", msg);
  return true;
}
bool videoCustomSet(bool isSize, int v, char* msg, size_t n) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) { strlcpy(msg, "camera not ready", n); return false; }
  std::lock_guard<std::mutex> lk(mtx);
  framesize_t fs = s->status.framesize;
  int q = s->status.quality;
  if (isSize) {
    if (!(v == FRAMESIZE_QVGA || v == FRAMESIZE_VGA || v == FRAMESIZE_SVGA || v == FRAMESIZE_XGA || v == FRAMESIZE_HD ||
          v == FRAMESIZE_SXGA || v == FRAMESIZE_UXGA)) { strlcpy(msg, "not supported / out of range", n); return false; }
    fs = (framesize_t)v;
  } else {
    if (v < 6 || v > 40) { strlcpy(msg, "not supported / out of range", n); return false; }
    q = v;
  }
  if (!applyVideo(fs, q, msg, n)) return false;
  int match = VQ_CUSTOM;                              // still exactly a preset?
  for (int i = 0; i < 3; i++) if (PRESETS[i].fs == fs && PRESETS[i].q == q) match = i;
  preset = match;
  strlcpy(msg, "ok", n);
  return true;
}
