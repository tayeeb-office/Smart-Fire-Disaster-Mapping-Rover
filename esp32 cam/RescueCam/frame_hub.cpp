// ============================================================================
//  Frame hub: ONE task reads the camera, copies each JPEG into a PSRAM slot,
//  and every consumer (live streams, recorder, photo) reads from the slots.
//  - No consumer steals frames from another (with 2 fb_get() callers each
//    would only get every second frame).
//  - The camera frame buffer is returned immediately, so a slow WiFi client
//    or a slow SD write never stalls the camera.
//  - Slots are reference-counted; a slot in use is never overwritten.
// ============================================================================
#include "cam_system.h"
#include <mutex>
#include <condition_variable>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <atomic>

volatile bool camOk = false;

namespace {
const int NSLOT = 5;
struct Slot {
  uint8_t* buf = nullptr;
  size_t cap = 0, len = 0;
  uint32_t seq = 0;
  int64_t ts = 0;
  int refs = 0;
};
Slot slots[NSLOT];
int latest = -1;
uint32_t seqCounter = 0;
std::mutex mtx;
std::condition_variable cv;
volatile float fps = 0;
std::atomic<uint32_t> dropped{0};
std::atomic<bool> holdReq{false}, holding{false};   // sensor reconfiguration in progress

bool ensureCap(Slot& s, size_t need) {
  if (s.cap >= need) return true;
  size_t ncap = (need + 16383) & ~(size_t)16383;          // grow in 16 KB steps, never shrink
  uint8_t* nb = (uint8_t*)heap_caps_realloc(s.buf, ncap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!nb) return false;
  s.buf = nb; s.cap = ncap;
  return true;
}

void camTask(void*) {
  uint32_t cnt = 0;
  int64_t t0 = esp_timer_get_time();
  uint32_t fails = 0;
  int skip = 0;
  for (;;) {
    if (holdReq) {                                            // paused while the sensor is reconfigured
      holding = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (holding) { holding = false; skip = 2; }               // frames already in the driver = old settings
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      if (++fails > 20) camOk = false;                      // ~2 s without frames
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    fails = 0; camOk = true;
    if (skip > 0) { skip--; esp_camera_fb_return(fb); continue; }
    if (fb->format != PIXFORMAT_JPEG || fb->len == 0) { esp_camera_fb_return(fb); continue; }
    // pick a free slot (not referenced, not the latest), oldest first
    int pick = -1;
    {
      std::lock_guard<std::mutex> lk(mtx);
      uint32_t oldest = UINT32_MAX;
      for (int i = 0; i < NSLOT; i++)
        if (slots[i].refs == 0 && i != latest && slots[i].seq < oldest) { oldest = slots[i].seq; pick = i; }
      if (pick >= 0) slots[pick].refs = -1;                   // -1 = being written by the camera task
    }
    if (pick < 0 || !ensureCap(slots[pick], fb->len)) {
      if (pick >= 0) { std::lock_guard<std::mutex> lk(mtx); slots[pick].refs = 0; }
      dropped++;
      esp_camera_fb_return(fb);
      continue;
    }
    memcpy(slots[pick].buf, fb->buf, fb->len);
    size_t len = fb->len;
    esp_camera_fb_return(fb);                                 // camera buffer free again right away
    {
      std::lock_guard<std::mutex> lk(mtx);
      slots[pick].len = len;
      slots[pick].ts = esp_timer_get_time();
      slots[pick].seq = ++seqCounter;
      slots[pick].refs = 0;
      latest = pick;
    }
    cv.notify_all();
    cnt++;
    int64_t now = esp_timer_get_time();
    if (now - t0 >= 1000000) { fps = cnt * 1e6f / (now - t0); cnt = 0; t0 = now; }
  }
}
}  // namespace

bool hubStart() {
  for (int i = 0; i < NSLOT; i++) if (!ensureCap(slots[i], 32 * 1024)) return false;  // LOW frames fit; grows on demand
  return xTaskCreatePinnedToCore(camTask, "camHub", 4096, nullptr, 6, nullptr, 1) == pdPASS;
}

bool hubAcquire(uint32_t afterSeq, HubFrame& out, uint32_t timeoutMs) {
  std::unique_lock<std::mutex> lk(mtx);
  bool ok = cv.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&] {
    return latest >= 0 && slots[latest].seq > afterSeq && slots[latest].refs >= 0;
  });
  if (!ok) return false;
  Slot& s = slots[latest];
  s.refs++;
  out.buf = s.buf; out.len = s.len; out.seq = s.seq; out.tsUs = s.ts; out.slot = latest;
  return true;
}

void hubRelease(const HubFrame& f) {
  std::lock_guard<std::mutex> lk(mtx);
  if (f.slot >= 0 && f.slot < NSLOT && slots[f.slot].refs > 0) slots[f.slot].refs--;
}

uint32_t hubLatestSeq() { std::lock_guard<std::mutex> lk(mtx); return seqCounter; }
float hubFps() { return camOk ? fps : 0; }
uint32_t hubDropped() { return dropped; }

// Pause the camera reader so no esp_camera_fb_get() runs while the sensor is
// switched to another frame size; after resuming, the first 2 frames (captured
// with the old settings) are dropped. Consumers just wait meanwhile.
bool hubHold(bool hold) {
  if (!hold) { holdReq = false; return true; }
  holdReq = true;
  for (int i = 0; i < 150 && !holding; i++) vTaskDelay(pdMS_TO_TICKS(10));   // max 1.5 s
  return holding;
}
