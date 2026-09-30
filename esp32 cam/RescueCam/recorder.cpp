// ============================================================================
//  Recorder: MJPEG-in-AVI to /videos, in its own task.
//  - Takes frames from the frame hub (never blocks the camera or the stream).
//  - If the SD card is slower than the camera, it simply takes the newest
//    frame each time (recording fps adapts; the AVI frame rate is set from the
//    real measured duration, so playback speed is correct).
//  - Header checkpoint every 5 s -> a power cut leaves a readable file.
//  - Rolls to a new file at ~1 GB / index full, stops safely when the card is
//    almost full or a write fails. Independent of any browser connection.
// ============================================================================
#include "cam_system.h"
#include "avi_writer.h"
#include "FS.h"
#include "SD_MMC.h"
#include "esp_heap_caps.h"
#include <mutex>
#include <atomic>

namespace {
struct FileSink {                    // adapter: Arduino File -> AviWriter
  File* f;
  size_t write(const uint8_t* d, size_t n) { return f->write(d, n); }
  bool seek(uint32_t p) { return f->seek(p); }
  size_t position() { return f->position(); }
  void flush() { f->flush(); }
};

TaskHandle_t recTaskH = nullptr;
SemaphoreHandle_t doneSem = nullptr;             // start/stop handshake with the web handler
std::atomic<bool> active{false}, stopReq{false};
std::atomic<int> cmd{0};                         // 1 = start
bool startOk = false;
std::mutex infoMtx;
RecInfo info = {};

void setMsg(const char* m) { std::lock_guard<std::mutex> lk(infoMtx); strlcpy(info.msg, m, sizeof(info.msg)); }

// Width/height from the JPEG's SOF marker. false = not found (frame is then accepted).
bool jpegSize(const uint8_t* b, size_t n, uint16_t& w, uint16_t& h) {
  if (n < 4 || b[0] != 0xFF || b[1] != 0xD8) return false;
  size_t i = 2;
  while (i + 9 < n && i < 4096) {                   // SOF is in the first few hundred bytes
    if (b[i] != 0xFF) return false;
    uint8_t m = b[i + 1];
    if (m == 0xFF) { i++; continue; }               // fill byte
    if (m == 0xD8 || m == 0x01 || (m >= 0xD0 && m <= 0xD7)) { i += 2; continue; }
    uint16_t L = (b[i + 2] << 8) | b[i + 3];
    if (m == 0xC0 || m == 0xC1 || m == 0xC2) {
      h = (b[i + 5] << 8) | b[i + 6];
      w = (b[i + 7] << 8) | b[i + 8];
      return true;
    }
    if (m == 0xDA || L < 2) return false;           // start of scan: no SOF found
    i += 2 + L;
  }
  return false;
}

// Opens a new AVI file. false = could not create.
bool openFile(File& file, FileSink& sink, AviWriter<FileSink>& avi, uint32_t* idx, uint16_t w, uint16_t h, char* path, size_t pn) {
  if (!storageUniqueName("/videos", "video", "avi", path, pn)) return false;
  file = SD_MMC.open(path, FILE_WRITE);
  if (!file) return false;
  sink.f = &file;
  if (!avi.begin(&sink, w, h, idx, REC_MAX_FRAMES)) { file.close(); SD_MMC.remove(path); return false; }
  fileBusyAdd(path);
  storageCountChange(true, 1);
  return true;
}
// Writes the index + final header and closes. Returns bytes written.
uint64_t closeFile(File& file, AviWriter<FileSink>& avi, uint32_t durMs, const char* path, bool& ok) {
  ok = avi.finish(durMs);
  uint64_t b = avi.bytes();
  file.close();
  fileBusyRemove(path);
  return b;
}

void recTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (cmd.exchange(0) != 1) continue;

    // ---------- start ----------
    startOk = false;
    uint16_t w = 0, h = 0; const char* fsName = "";
    uint32_t* idx = nullptr;
    char path[48] = "";
    File file; FileSink sink{nullptr}; AviWriter<FileSink> avi;
    if (!camOk || !cameraFrameSize(w, h, &fsName)) setMsg("Camera not ready");
    else if (!sdUseBegin()) setMsg("SD CARD NOT INSERTED / RECORDING DISABLED");
    else {
      if (storageFreeBytes() < SD_MIN_FREE_START) { setMsg("SD card almost full - recording not started"); sdUseEnd(); }
      else if (!(idx = (uint32_t*)heap_caps_malloc(REC_MAX_FRAMES * 8, MALLOC_CAP_SPIRAM))) { setMsg("Not enough PSRAM for the index"); sdUseEnd(); }
      else if (!openFile(file, sink, avi, idx, w, h, path, sizeof(path))) {
        setMsg("Could not create video file (SD removed / write-protected?)");
        sdUseEnd(); free(idx); idx = nullptr;
        storageMarkError("could not create video file");   // re-check the card
      }
      else startOk = true;
    }
    if (!startOk) { xSemaphoreGive(doneSem); continue; }

    {
      std::lock_guard<std::mutex> lk(infoMtx);
      strlcpy(info.file, path, sizeof(info.file));
      info.frames = 0; info.bytes = 0; info.durMs = 0; info.fps = 0; info.w = w; info.h = h;
      snprintf(info.msg, sizeof(info.msg), "Recording %ux%u (%s)", w, h, fsName);
    }
    stopReq = false;
    active = true;
    Serial.printf("[REC] start %s %ux%u\n", path, w, h);
    xSemaphoreGive(doneSem);

    // ---------- record loop ----------
    uint32_t sessStart = millis(), fileStart = sessStart, lastCk = sessStart, lastSeq = hubLatestSeq();
    uint32_t sessFrames = 0, noFrameMs = 0, badSize = 0;
    uint64_t sessBytes = 0, fileBaseBytes = 0;
    const char* endWhy = "Stopped";
    bool writeOk = true;
    while (!stopReq) {
      HubFrame fr;
      if (!hubAcquire(lastSeq, fr, 500)) {
        noFrameMs += 500;
        if (noFrameMs > 5000) { endWhy = "Camera stopped delivering frames"; break; }
        continue;
      }
      noFrameMs = 0;
      lastSeq = fr.seq;
      uint16_t fw, fh;                                              // one AVI = one frame size:
      if (jpegSize(fr.buf, fr.len, fw, fh) && (fw != w || fh != h)) { // skip frames of another size
        hubRelease(fr);                                             // (only during a resolution switch)
        if (++badSize > 60) { endWhy = "Camera frame size changed"; break; }
        continue;
      }
      badSize = 0;
      bool ok = avi.addFrame(fr.buf, fr.len);
      hubRelease(fr);
      uint32_t now = millis();
      if (!ok) {
        if (!avi.ok()) { writeOk = false; endWhy = "SD write failed (card removed or full)"; break; }
        // index full -> roll to a new file
        bool fok; uint64_t b = closeFile(file, avi, now - fileStart, path, fok);
        fileBaseBytes += b;
        if (!openFile(file, sink, avi, idx, w, h, path, sizeof(path))) { endWhy = "Could not create next video file"; path[0] = 0; break; }
        fileStart = now; lastCk = now;
        continue;
      }
      sessFrames++;
      if (now - lastCk >= (sessFrames < 40 ? 1000u : (uint32_t)REC_CHECKPOINT_MS)) {   // first after ~1 s, then every 5 s
        lastCk = now;
        if (!avi.checkpoint(now - fileStart)) { writeOk = false; endWhy = "SD write failed (card removed or full)"; break; }
        uint64_t curBytes = fileBaseBytes + avi.bytes();
        storageAddUsed((int64_t)(curBytes - sessBytes));
        sessBytes = curBytes;
        if (storageFreeBytes() < SD_MIN_FREE_STOP) { endWhy = "SD card full - recording stopped safely"; break; }
      }
      if (avi.bytes() >= REC_MAX_FILE_BYTES) {                      // AVI 1.0 size limit -> next file
        bool fok; uint64_t b = closeFile(file, avi, now - fileStart, path, fok);
        fileBaseBytes += b;
        if (!openFile(file, sink, avi, idx, w, h, path, sizeof(path))) { endWhy = "Could not create next video file"; path[0] = 0; break; }
        fileStart = now; lastCk = now;
      }
      {
        std::lock_guard<std::mutex> lk(infoMtx);
        info.frames = sessFrames;
        info.bytes = fileBaseBytes + avi.bytes();
        info.durMs = now - sessStart;
        info.fps = info.durMs > 1000 ? sessFrames * 1000.0f / info.durMs : 0;
        strlcpy(info.file, path, sizeof(info.file));
      }
    }

    // ---------- finalize ----------
    uint32_t now = millis();
    bool finOk = false;
    uint64_t totalBytes = fileBaseBytes;
    if (path[0]) {
      if (writeOk) totalBytes += closeFile(file, avi, now - fileStart, path, finOk);
      else { file.close(); fileBusyRemove(path); }
    }
    active = false;
    if (!writeOk) storageMarkError("write failed during recording");
    storageAddUsed((int64_t)(totalBytes - sessBytes));
    storageRefreshSoon();
    sdUseEnd();
    free(idx);
    {
      std::lock_guard<std::mutex> lk(infoMtx);
      strlcpy(info.lastFile, path, sizeof(info.lastFile));
      info.durMs = now - sessStart;
      snprintf(info.msg, sizeof(info.msg), "%s - %lu frames, %.1f s%s", endWhy, (unsigned long)sessFrames, info.durMs / 1000.0f,
               (!writeOk || (path[0] && !finOk)) ? " (file may need repair)" : "");
    }
    Serial.printf("[REC] %s\n", info.msg);
    xSemaphoreGive(doneSem);                                        // for a waiting /stop request
  }
}
}  // namespace

void recorderBegin() {
  doneSem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(recTask, "recorder", 8192, nullptr, 4, &recTaskH, 1);
}
bool recorderActive() { return active; }

bool recorderStart(char* msg, size_t n) {
  if (active) { strlcpy(msg, "Already recording", n); return false; }
  xSemaphoreTake(doneSem, 0);                                       // clear a stale signal
  cmd = 1;
  xTaskNotifyGive(recTaskH);
  if (xSemaphoreTake(doneSem, pdMS_TO_TICKS(4000)) != pdTRUE) { strlcpy(msg, "Recorder did not respond", n); return false; }
  std::lock_guard<std::mutex> lk(infoMtx);
  strlcpy(msg, startOk ? info.file : info.msg, n);
  return startOk;
}
bool recorderStop(char* msg, size_t n) {
  if (!active) { strlcpy(msg, "Not recording", n); return false; }
  xSemaphoreTake(doneSem, 0);
  stopReq = true;
  if (xSemaphoreTake(doneSem, pdMS_TO_TICKS(8000)) != pdTRUE) { strlcpy(msg, "Stopping... (finalizing file)", n); return true; }
  std::lock_guard<std::mutex> lk(infoMtx);
  strlcpy(msg, info.msg, n);
  return true;
}
void recorderGetInfo(RecInfo& o) {
  std::lock_guard<std::mutex> lk(infoMtx);
  o = info;
  o.active = active;
}

// ============================================================================
//  Photo: newest frame -> /photos/photo_*.jpg
// ============================================================================
static std::mutex photoMtx;
static char lastPhoto[48] = "";
void photoLast(char* out, size_t n) { std::lock_guard<std::mutex> lk(photoMtx); strlcpy(out, lastPhoto, n); }

bool photoCapture(char* nameOut, size_t n, char* msg, size_t mn) {
  nameOut[0] = 0;
  if (!camOk) { strlcpy(msg, "Camera not ready", mn); return false; }
  if (!sdUseBegin()) { strlcpy(msg, "SD card not available", mn); return false; }
  bool ok = false;
  HubFrame fr;
  // a frame taken AFTER the button was pressed
  if (!hubAcquire(hubLatestSeq(), fr, 1500)) strlcpy(msg, "No camera frame", mn);
  else {
    char path[48];
    if (!storageUniqueName("/photos", "photo", "jpg", path, sizeof(path))) strlcpy(msg, "Could not create a unique file name", mn);
    else {
      File f = SD_MMC.open(path, FILE_WRITE);
      if (!f) strlcpy(msg, "Could not create file", mn);
      else {
        size_t w = f.write(fr.buf, fr.len);
        f.close();
        if (w != fr.len) { SD_MMC.remove(path); strlcpy(msg, "SD write failed", mn); storageMarkError("photo write failed"); }
        else {
          ok = true;
          strlcpy(nameOut, path, n);
          snprintf(msg, mn, "Saved %s (%u KB)", path, (unsigned)(fr.len / 1024));
          storageAddUsed(fr.len);
          storageCountChange(false, 1);
          std::lock_guard<std::mutex> lk(photoMtx);
          strlcpy(lastPhoto, path, sizeof(lastPhoto));
        }
      }
    }
    hubRelease(fr);
  }
  sdUseEnd();
  return ok;
}
