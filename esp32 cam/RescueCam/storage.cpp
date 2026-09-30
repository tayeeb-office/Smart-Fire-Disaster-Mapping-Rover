// ============================================================================
//  microSD storage (on-board slot, SD_MMC 1-bit: CLK 39, CMD 38, D0 40)
//  - Mounted in a background task: a missing / slow card never blocks boot.
//  - Never formats the card.
//  - "Users" counter: the card is only unmounted/remounted when nothing is
//    using it (no recording, no download, no photo write in progress).
//  - Busy-file list: a file being recorded/downloaded/played cannot be deleted
//    (FATFS here has no file locking; deleting an open file corrupts the FAT).
// ============================================================================
#include "cam_system.h"
#include "FS.h"
#include "SD_MMC.h"
#include <Preferences.h>
#include "esp_heap_caps.h"
#include <mutex>
#include <atomic>

namespace {
std::mutex mtx;                         // protects the fields below + mount state
bool mounted = false;
int users = 0;
char cardType[8] = "NONE";
uint64_t total = 0, used = 0;
uint32_t photos = 0, videos = 0;
char lastErr[64] = "starting";
std::atomic<bool> refreshReq{false};
std::atomic<bool> errorReq{false};
const int NBUSY = 8;
char busy[NBUSY][48];
std::mutex busyMtx;
std::mutex nameMtx;

uint32_t countFiles(const char* dir) {
  File d = SD_MMC.open(dir);
  if (!d || !d.isDirectory()) return 0;
  uint32_t n = 0;
  File f = d.openNextFile();
  while (f && n < 20000) { if (!f.isDirectory()) n++; f.close(); f = d.openNextFile(); }
  d.close();
  return n;
}

// Runs WITHOUT the lock (it can take seconds). Safe: while !mounted nobody can
// become a user (sdUseBegin() refuses), so nothing else touches the card.
bool doMount() {
  const char* err = nullptr;
  char t8[8] = "NONE";
  uint64_t tot = 0, u = 0; uint32_t np = 0, nv = 0;
  SD_MMC.end();
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  // mount point, 1-bit mode, format_if_mount_failed = false (never erase the user's card)
  if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 8)) err = "no card / not FAT32";
  else {
    uint8_t t = SD_MMC.cardType();
    if (t == CARD_NONE) err = "no card";
    else {
      strcpy(t8, t == CARD_MMC ? "MMC" : (t == CARD_SDHC ? "SDHC" : "SD"));
      if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");
      if (!SD_MMC.exists("/videos")) SD_MMC.mkdir("/videos");
      if (!SD_MMC.exists("/photos") || !SD_MMC.exists("/videos")) err = "cannot create folders (write-protected?)";
      else { tot = SD_MMC.totalBytes(); u = SD_MMC.usedBytes(); np = countFiles("/photos"); nv = countFiles("/videos"); }
    }
    if (err) SD_MMC.end();
  }
  std::lock_guard<std::mutex> lk(mtx);
  if (err) { strlcpy(lastErr, err, sizeof(lastErr)); return false; }
  strcpy(cardType, t8); total = tot; used = u; photos = np; videos = nv; lastErr[0] = 0;
  return true;
}

// Card still there? One raw sector read (real bus transfer; FATFS caches
// could hide a removed card). Only when nobody is using the card.
bool cardPresent(uint8_t* buf) {
  return buf ? SD_MMC.readRAW(buf, 0) : true;
}

void storageTask(void*) {
  vTaskDelay(pdMS_TO_TICKS(800));                       // camera + WiFi come first
  uint32_t lastTry = 0, lastRefresh = millis(), lastProbe = millis();
  int lastState = -1;                                   // log only when the state changes
  char lastLogErr[64] = "";
  uint8_t* probeBuf = (uint8_t*)heap_caps_malloc(512, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  for (;;) {
    bool wantMount, wantRefresh, wantProbe;
    {
      std::lock_guard<std::mutex> lk(mtx);
      if (errorReq.exchange(false) && mounted && users == 0) { SD_MMC.end(); mounted = false; }
      wantMount = !mounted && users == 0 && (lastTry == 0 || millis() - lastTry > 5000);
      wantRefresh = mounted && users == 0 && !recorderActive() && (refreshReq.load() || millis() - lastRefresh > 60000);
      wantProbe = mounted && users == 0 && !recorderActive() && millis() - lastProbe > 3000;
    }
    if (wantProbe && !wantRefresh) {
      lastProbe = millis();
      if (!cardPresent(probeBuf)) {
        std::lock_guard<std::mutex> lk(mtx);
        if (users == 0) {                               // nobody started using it meanwhile
          SD_MMC.end(); mounted = false;
          strlcpy(lastErr, "card removed", sizeof(lastErr));
          lastTry = millis();
        }
      }
    }
    if (wantMount) {
      lastTry = millis();
      bool m = doMount();
      { std::lock_guard<std::mutex> lk(mtx); mounted = m; }
      if ((int)m != lastState || (!m && strcmp(lastLogErr, lastErr))) {
        Serial.printf("[SD] %s %s total %.2f GB used %.2f GB\n", m ? "CONNECTED" : "NOT INSERTED / NOT USABLE -", m ? cardType : lastErr, total / 1e9, used / 1e9);
        strlcpy(lastLogErr, lastErr, sizeof(lastLogErr));
      }
      lastState = m;
      lastProbe = millis();
    } else if (wantRefresh) {
      // exact usage (can take a moment on big cards) - only when idle
      if (sdUseBegin()) {
        uint64_t u = SD_MMC.usedBytes();
        uint32_t p = countFiles("/photos"), v = countFiles("/videos");
        sdUseEnd();
        std::lock_guard<std::mutex> lk(mtx);
        used = u; photos = p; videos = v;
      }
      refreshReq = false;
      lastRefresh = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
}  // namespace

void storageBegin() {
  for (int i = 0; i < NBUSY; i++) busy[i][0] = 0;
  xTaskCreatePinnedToCore(storageTask, "storage", 6144, nullptr, 1, nullptr, 0);
}

bool sdUseBegin() {
  std::lock_guard<std::mutex> lk(mtx);
  if (!mounted || errorReq) return false;
  users++;
  return true;
}
void sdUseEnd() { std::lock_guard<std::mutex> lk(mtx); if (users > 0) users--; }

void storageGetInfo(SdInfo& o) {
  std::lock_guard<std::mutex> lk(mtx);
  o.ok = mounted && !errorReq;
  strlcpy(o.type, mounted ? cardType : "NONE", sizeof(o.type));
  o.total = mounted ? total : 0;
  o.used = mounted ? (used > total ? total : used) : 0;
  o.photos = photos; o.videos = videos;
  o.low = mounted && total && (total - o.used) < total * SD_LOW_WARN_PCT / 100;
  strlcpy(o.err, lastErr, sizeof(o.err));
}
uint64_t storageFreeBytes() {
  std::lock_guard<std::mutex> lk(mtx);
  return (mounted && total > used) ? total - used : 0;
}
void storageAddUsed(int64_t d) {
  std::lock_guard<std::mutex> lk(mtx);
  if (d < 0 && (uint64_t)(-d) > used) used = 0; else used += d;
}
void storageCountChange(bool video, int d) {
  std::lock_guard<std::mutex> lk(mtx);
  uint32_t& c = video ? videos : photos;
  if (d < 0 && (uint32_t)(-d) > c) c = 0; else c += d;
}
void storageRefreshSoon() { refreshReq = true; }
void storageMarkError(const char* why) {
  { std::lock_guard<std::mutex> lk(mtx); strlcpy(lastErr, why, sizeof(lastErr)); }
  errorReq = true;
  Serial.printf("[SD] error: %s -> will remount when idle\n", why);
}

// ---- clock (set by the dashboard from the phone/PC time) ----
static bool clockOk = false;
bool clockValid() { return clockOk && time(nullptr) > 1700000000; }
void clockSet(uint32_t epoch, int tzOffsetMin) {
  if (epoch < 1700000000UL) return;
  // JS getTimezoneOffset(): UTC+6 -> -360. POSIX TZ uses the same sign: "UTC-6:00"
  char tz[16];
  int a = tzOffsetMin < 0 ? -tzOffsetMin : tzOffsetMin;
  snprintf(tz, sizeof(tz), "UTC%c%d:%02d", tzOffsetMin < 0 ? '-' : '+', a / 60, a % 60);
  setenv("TZ", tz, 1); tzset();
  struct timeval tv = { (time_t)epoch, 0 };
  settimeofday(&tv, nullptr);
  clockOk = true;
}

// Unique file name. With a valid clock: <prefix>_YYYYMMDD_HHMMSS.<ext> (+ _2, _3 ...)
// otherwise: <prefix>_0001.<ext> using a counter stored in NVS, never overwriting.
bool storageUniqueName(const char* dir, const char* prefix, const char* ext, char* out, size_t n) {
  std::lock_guard<std::mutex> lk(nameMtx);
  if (clockValid()) {
    time_t t = time(nullptr); struct tm tmv; localtime_r(&t, &tmv);
    char ts[20]; strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);
    snprintf(out, n, "%s/%s_%s.%s", dir, prefix, ts, ext);
    for (int k = 2; SD_MMC.exists(out) && k < 100; k++) snprintf(out, n, "%s/%s_%s_%d.%s", dir, prefix, ts, k, ext);
    return !SD_MMC.exists(out);
  }
  Preferences pr;
  pr.begin("camnames", false);
  char key[12]; snprintf(key, sizeof(key), "n_%s", prefix);
  uint32_t num = pr.getUInt(key, 0);
  for (int tries = 0; tries < 10000; tries++) {
    num++;
    snprintf(out, n, "%s/%s_%04lu.%s", dir, prefix, (unsigned long)num, ext);
    if (!SD_MMC.exists(out)) { pr.putUInt(key, num); pr.end(); return true; }
  }
  pr.end();
  return false;
}

// Only "/photos/<name>" and "/videos/<name>" (and root files for viewing).
// Names: letters, digits, '_', '-', '.', no "..", no further '/'.
bool pathAllowed(const char* path, bool forDelete) {
  if (!path || path[0] != '/') return false;
  size_t L = strlen(path);
  if (L < 2 || L > 47 || strstr(path, "..")) return false;
  const char* name = path + 1;
  if (!strncmp(path, "/photos/", 8)) name = path + 8;
  else if (!strncmp(path, "/videos/", 8)) name = path + 8;
  else if (forDelete) return false;                       // deletes only inside /photos and /videos
  if (!*name) return false;
  for (const char* c = name; *c; c++)
    if (!((*c >= 'A' && *c <= 'Z') || (*c >= 'a' && *c <= 'z') || (*c >= '0' && *c <= '9') || *c == '_' || *c == '-' || *c == '.')) return false;
  return true;
}

bool fileBusy(const char* p) {
  std::lock_guard<std::mutex> lk(busyMtx);
  for (int i = 0; i < NBUSY; i++) if (busy[i][0] && !strcmp(busy[i], p)) return true;
  return false;
}
bool fileBusyAdd(const char* p) {
  std::lock_guard<std::mutex> lk(busyMtx);
  for (int i = 0; i < NBUSY; i++) if (!busy[i][0]) { strlcpy(busy[i], p, sizeof(busy[0])); return true; }
  return false;
}
void fileBusyRemove(const char* p) {
  std::lock_guard<std::mutex> lk(busyMtx);
  for (int i = 0; i < NBUSY; i++) if (busy[i][0] && !strcmp(busy[i], p)) { busy[i][0] = 0; return; }
}
