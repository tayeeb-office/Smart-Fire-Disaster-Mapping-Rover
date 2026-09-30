// ============================================================================
//  RESCUE CAM - shared declarations (ESP32-S3-CAM camera + microSD system)
// ============================================================================
#pragma once
#include <Arduino.h>
#include "esp_camera.h"

// ---------------- user settings ----------------
#define AP_SSID            "RescueCam"      // access point made by the camera
#define AP_PASS            "rover1234"      // >= 8 characters
#define AP_CHANNEL         1
// Other WiFi networks the camera should join (home WiFi, your PHONE HOTSPOT,
// a 4G pocket router ...): easiest is the dashboard (WiFi settings, up to 3,
// saved in the board's memory). You can also put one here as the default:
#define JOIN_SSID          ""               // e.g. "MyPhoneHotspot"
#define JOIN_PASS          ""               // e.g. "mypassword"
#define JOIN_TIMEOUT_MS    12000            // per network, then the RescueCam access point
#define MAX_SAVED_NETS     3
#define STA_LOST_AP_MS     60000            // joined WiFi lost this long -> RescueCam AP comes back too
#define MDNS_NAME          "rescuecam"      // -> http://rescuecam.local + name in router/hotspot lists

#define AUTO_RECORD_DEFAULT true            // START STREAM also starts SD recording (dashboard setting)
#define STREAM_MAX_VIEWERS 3                // simultaneous live viewers
#define AUTO_REC_RETRIES   3                // auto-restart recording after SD error/re-insert, per stream session

// Video quality presets. The camera boots in LOW / FAST and EVERY "START STREAM"
// starts in LOW / FAST (fast first picture, least mobile data). Raise it from the
// dashboard once the picture is running.   fps = live-stream limit in "Normal" mode
//                        frame size        JPEG q (higher = smaller file)  fps
#define PRESET_LOW_FS      FRAMESIZE_QVGA   // 320x240
#define PRESET_LOW_Q       20
#define PRESET_LOW_FPS     10               // ~5-8 KB/frame -> ~0.4-0.6 Mbit/s
#define PRESET_MED_FS      FRAMESIZE_VGA    // 640x480
#define PRESET_MED_Q       14
#define PRESET_MED_FPS     12               // ~1.5-2.5 Mbit/s
#define PRESET_HIGH_FS     FRAMESIZE_HD     // 1280x720
#define PRESET_HIGH_Q      12
#define PRESET_HIGH_FPS    15               // ~4-7 Mbit/s (good WiFi only)
#define FPS_DATA_SAVER     5                // "Data saver" frame-rate mode (any preset)

// Optional password for the web dashboard (HTTP Basic auth). Leave WEB_PASS
// empty for no password (fine on your own WiFi). SET IT before you make the
// camera reachable from the internet (Cloudflare Tunnel etc.).
#define WEB_USER           "admin"
#define WEB_PASS           ""

#define SD_MMC_CLK         39               // on-board microSD slot, SD_MMC 1-bit
#define SD_MMC_CMD         38               // (Freenove ESP32-S3-WROOM CAM: "do not modify")
#define SD_MMC_D0          40

#define STATUS_LED_PIN     2                // on-board LED: blinks while recording (-1 = unused)

#ifndef REC_MAX_FILE_BYTES
#define REC_MAX_FILE_BYTES (1000ULL * 1024 * 1024)  // AVI 1.0 safe size: roll to a new file at ~1 GB
#endif
#ifndef REC_MAX_FRAMES
#define REC_MAX_FRAMES     60000            // index capacity per file (8 bytes/frame in PSRAM)
#endif
#define REC_CHECKPOINT_MS  5000             // header/size update so a power cut leaves a readable file
#define SD_MIN_FREE_START  (64ULL * 1024 * 1024)    // refuse to start recording below this
#define SD_MIN_FREE_STOP   (32ULL * 1024 * 1024)    // stop recording safely below this
#define SD_LOW_WARN_PCT    10               // "almost full" warning

// ---------------- frame hub (one camera reader, many consumers) ----------------
struct HubFrame {
  const uint8_t* buf;
  size_t len;
  uint32_t seq;
  int64_t tsUs;
  int slot;
};
bool     hubStart();
bool     hubAcquire(uint32_t afterSeq, HubFrame& out, uint32_t timeoutMs);   // newest frame with seq > afterSeq
void     hubRelease(const HubFrame& f);
uint32_t hubLatestSeq();
float    hubFps();
uint32_t hubDropped();
bool     hubHold(bool hold);                  // pause the camera reader (safe sensor reconfiguration)
extern volatile bool camOk;
bool     cameraFrameSize(uint16_t& w, uint16_t& h, const char** name);
const char* cameraSensorName();

// ---------------- microSD storage ----------------
struct SdInfo {
  bool ok;
  char type[8];
  uint64_t total, used;
  uint32_t photos, videos;
  bool low;
  char err[64];
};
void     storageBegin();                      // mounts in a background task
void     storageGetInfo(SdInfo& out);
bool     sdUseBegin();                        // true = card mounted, keeps it mounted until sdUseEnd()
void     sdUseEnd();
uint64_t storageFreeBytes();
void     storageAddUsed(int64_t delta);       // fast estimate after writes/deletes
void     storageCountChange(bool video, int delta);
void     storageRefreshSoon();                // exact usage recalculated in the background
void     storageMarkError(const char* why);   // write failure -> remount later
bool     storageUniqueName(const char* dir, const char* prefix, const char* ext, char* out, size_t n);
bool     pathAllowed(const char* path, bool forDelete);
bool     fileBusy(const char* path);
bool     fileBusyAdd(const char* path);
void     fileBusyRemove(const char* path);

// ---------------- recorder (MJPEG in AVI) ----------------
struct RecInfo {
  bool active;
  char file[48];
  uint32_t frames;
  uint64_t bytes;
  uint32_t durMs;
  float fps;
  uint16_t w, h;
  char lastFile[48];
  char msg[112];
};
void recorderBegin();
bool recorderStart(char* msg, size_t n);
bool recorderStop(char* msg, size_t n);
void recorderGetInfo(RecInfo& out);
bool recorderActive();

// ---------------- photo ----------------
bool photoCapture(char* nameOut, size_t n, char* msg, size_t mn);
void photoLast(char* out, size_t n);

// ---------------- clock (set from the browser) ----------------
bool clockValid();
void clockSet(uint32_t epoch, int tzOffsetMin);

// ---------------- stream session: START STREAM = live view + auto recording ----------------
void sessionBegin();
bool streamStart(char* msg, size_t n);   // stream ON (+ recording if SD present and auto-record on)
bool streamStop(char* msg, size_t n);    // stream OFF + recording finalized
bool streamIsOn();
bool sessionRecStart(char* msg, size_t n);   // manual record while streaming
bool sessionRecStop(char* msg, size_t n);    // stop recording, keep streaming
bool autoRecGet();
void autoRecSet(bool on);
bool autoRecPending();                   // waiting for an SD card
void sessionNote(char* out, size_t n);
void sessionService();                   // from loop(): auto-start when SD appears, restart after errors

// ---------------- video quality (presets / custom / frame-rate mode) ----------------
enum { VQ_LOW = 0, VQ_MEDIUM = 1, VQ_HIGH = 2, VQ_CUSTOM = 3 };
enum { FPSM_SAVER = 0, FPSM_NORMAL = 1, FPSM_MAX = 2 };
bool videoPresetSet(int preset, char* msg, size_t n);           // LOW / MEDIUM / HIGH
bool videoCustomSet(bool isSize, int value, char* msg, size_t n); // advanced: framesize or quality
void videoFpsModeSet(int mode);
int  videoPreset();
int  videoFpsMode();
int  streamFpsLimit();                    // 0 = no limit
const char* videoPresetName(int p);

// ---------------- web / status ----------------
void   startWebServer();
int    streamViewers();
uint64_t streamBytesSent();              // live-stream bytes since START STREAM (data use)
void   streamBytesReset();
size_t statusJson(char* out, size_t n);
void   netStatus(char* mode, size_t mn, char* ip, size_t in, int& rssi, int& clients);
size_t wifiScanJson(char* out, size_t n);                 // nearby networks
int    wifiSaveNet(const char* ssid, const char* pass);   // 0 saved (restarts), 1 list full, 2 invalid
bool   wifiForgetNet(int index);                          // no restart
