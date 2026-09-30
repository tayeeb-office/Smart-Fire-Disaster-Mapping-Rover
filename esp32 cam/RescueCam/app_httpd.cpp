// ============================================================================
//  Web server (port 80) - based on Espressif's CameraWebServer app_httpd.cpp
//  Long transfers (live stream, video playback, file download) run in their
//  own small task via httpd_req_async_handler_begin(), so the dashboard/API
//  stay responsive and several viewers can watch at the same time.
// ============================================================================
#include "cam_system.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "FS.h"
#include "SD_MMC.h"
#include "dashboard.h"
#include "lwip/sockets.h"
#include "mbedtls/base64.h"
#include <atomic>

static httpd_handle_t server = nullptr;
static std::atomic<int> streams{0}, transfers{0};
static const int MAX_STREAMS = STREAM_MAX_VIEWERS, MAX_TRANSFERS = 2;
static std::atomic<uint64_t> sentBytes{0};
int streamViewers() { return streams.load(); }
uint64_t streamBytesSent() { return sentBytes.load(); }
void streamBytesReset() { sentBytes = 0; }

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";
// boundary + part header in ONE send (fewer TCP packets per frame -> lower latency)
static const char* STREAM_HEAD = "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

// ---------------- helpers ----------------
static esp_err_t sendJson(httpd_req_t* req, const char* js) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, js, HTTPD_RESP_USE_STRLEN);
}
static void jsonEsc(const char* in, char* out, size_t n) {       // minimal JSON string escaping
  size_t o = 0;
  for (; *in && o + 2 < n; in++) { if (*in == '"' || *in == '\\') out[o++] = '\\'; out[o++] = (*in < 32) ? ' ' : *in; }
  out[o] = 0;
}
static esp_err_t sendResult(httpd_req_t* req, bool ok, const char* msg, const char* file) {
  char m[160], f[64], js[280];
  jsonEsc(msg ? msg : "", m, sizeof(m)); jsonEsc(file ? file : "", f, sizeof(f));
  snprintf(js, sizeof(js), "{\"ok\":%s,\"msg\":\"%s\",\"file\":\"%s\"}", ok ? "true" : "false", m, f);
  return sendJson(req, js);
}
static bool queryParam(httpd_req_t* req, const char* key, char* val, size_t n) {
  char q[160];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return false;
  if (httpd_query_key_value(q, key, val, n) != ESP_OK) return false;
  // decode %2F etc.
  char* w = val;
  for (char* r = val; *r; r++) {
    if (*r == '%' && isxdigit((unsigned char)r[1]) && isxdigit((unsigned char)r[2])) { char hx[3] = {r[1], r[2], 0}; *w++ = (char)strtol(hx, nullptr, 16); r += 2; }
    else if (*r == '+') *w++ = ' ';
    else *w++ = *r;
  }
  *w = 0;
  return true;
}

// ---------------- async task plumbing ----------------
struct Job { httpd_req_t* req; char path[48]; bool download; };
static bool spawnJob(httpd_req_t* req, TaskFunction_t fn, const char* path, bool dl, const char* name, uint32_t stack) {
  Job* j = (Job*)calloc(1, sizeof(Job));
  if (!j) return false;
  if (path) strlcpy(j->path, path, sizeof(j->path));
  j->download = dl;
  if (httpd_req_async_handler_begin(req, &j->req) != ESP_OK) { free(j); return false; }
  if (xTaskCreate(fn, name, stack, j, 5, nullptr) != pdPASS) { httpd_req_async_handler_complete(j->req); free(j); return false; }
  return true;
}
static void endJob(Job* j) { httpd_req_async_handler_complete(j->req); free(j); vTaskDelete(nullptr); }

// ---------------- live stream (MJPEG) ----------------
// Runs only while the stream session is ON (START STREAM). STOP STREAM ends
// every viewer's connection within ~1 s. A viewer that disconnects (phone
// screen off, WiFi lost) is detected by the failed send and cleaned up.
static void streamTask(void* arg) {
  Job* j = (Job*)arg;
  httpd_req_t* req = j->req;
  int fd = httpd_req_to_sockfd(req), one = 1;
  if (fd >= 0) setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));   // send frames immediately
  httpd_resp_set_type(req, STREAM_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Accel-Buffering", "no");                          // proxies/tunnels: don't buffer
  char part[160];
  uint32_t last = 0;
  int idle = 0;
  int64_t nextAt = 0;                                                            // frame-rate limit (data saving)
  while (streamIsOn()) {
    int lim = streamFpsLimit();
    if (lim > 0) {                                                               // wait in short steps so STOP
      int64_t now = esp_timer_get_time();                                        // STREAM still ends at once
      if (now < nextAt) { int64_t w = nextAt - now; vTaskDelay(pdMS_TO_TICKS(w > 100000 ? 100 : (w / 1000 + 1))); continue; }
    }
    HubFrame fr;
    if (!hubAcquire(last, fr, 1000)) { if (++idle > 10) break; continue; }   // 10 s without frames -> end
    idle = 0; last = fr.seq;
    // newest frame only: frames made while we waited are skipped, never queued
    size_t hl = snprintf(part, sizeof(part), STREAM_HEAD, (unsigned)fr.len, (int)(fr.tsUs / 1000000), (int)(fr.tsUs % 1000000));
    esp_err_t r = httpd_resp_send_chunk(req, part, hl);
    if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char*)fr.buf, fr.len);
    size_t sent = hl + fr.len;
    hubRelease(fr);
    if (r != ESP_OK) break;                                                     // viewer left
    sentBytes += sent;
    int64_t now = esp_timer_get_time();
    if (lim > 0) nextAt = (nextAt && now - nextAt < 200000) ? nextAt + 1000000 / lim : now + 1000000 / lim;
  }
  httpd_resp_send_chunk(req, nullptr, 0);                                       // clean end (ignored if gone)
  streams--;
  endJob(j);
}
static esp_err_t stream_handler(httpd_req_t* req) {
  if (!streamIsOn()) { httpd_resp_set_status(req, "503 Service Unavailable"); httpd_resp_set_hdr(req, "Cache-Control", "no-store"); return httpd_resp_send(req, "stream is OFF - press START STREAM", HTTPD_RESP_USE_STRLEN); }
  if (streams.load() >= MAX_STREAMS) { httpd_resp_set_status(req, "503 Service Unavailable"); return httpd_resp_send(req, "too many viewers", HTTPD_RESP_USE_STRLEN); }
  streams++;
  if (!spawnJob(req, streamTask, nullptr, false, "stream", 5120)) { streams--; return httpd_resp_send_500(req); }
  return ESP_OK;
}

// ---------------- single frame (no SD) ----------------
// Also the dashboard's SNAPSHOT mode: when a tunnel/proxy buffers the endless
// /stream response, the page fetches single frames from here instead.
static esp_err_t capture_handler(httpd_req_t* req) {
  HubFrame fr;
  if (!hubAcquire(0, fr, 2000)) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t r = httpd_resp_send(req, (const char*)fr.buf, fr.len);
  if (r == ESP_OK && streamIsOn()) sentBytes += fr.len;       // snapshot-mode viewers count as live data too
  hubRelease(fr);
  return r;
}

// ---------------- file download / view (async) ----------------
static void fileTask(void* arg) {
  Job* j = (Job*)arg;
  httpd_req_t* req = j->req;
  const char* p = j->path;
  bool jpg = strstr(p, ".jpg") || strstr(p, ".JPG");
  bool avi = strstr(p, ".avi") || strstr(p, ".AVI");
  const char* nm = strrchr(p, '/') + 1;
  char cd[80];
  snprintf(cd, sizeof(cd), "%s; filename=\"%s\"", j->download ? "attachment" : "inline", nm);
  File f;
  if (sdUseBegin()) {
    f = SD_MMC.open(p, FILE_READ);
    if (f && !f.isDirectory()) {
      httpd_resp_set_type(req, jpg ? "image/jpeg" : (avi ? "video/x-msvideo" : "application/octet-stream"));
      httpd_resp_set_hdr(req, "Content-Disposition", cd);
      uint8_t* buf = (uint8_t*)malloc(8192);
      esp_err_t r = buf ? ESP_OK : ESP_FAIL;
      size_t got;
      while (r == ESP_OK && (got = f.read(buf, 8192)) > 0) r = httpd_resp_send_chunk(req, (const char*)buf, got);
      if (r == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
      free(buf);
    } else httpd_resp_send_404(req);
    if (f) f.close();
    sdUseEnd();
  } else httpd_resp_send_404(req);
  fileBusyRemove(p);
  transfers--;
  endJob(j);
}
static esp_err_t file_handler(httpd_req_t* req) {
  char path[48], dl[4] = "0";
  if (!queryParam(req, "path", path, sizeof(path)) || !pathAllowed(path, false)) return httpd_resp_send_404(req);
  queryParam(req, "dl", dl, sizeof(dl));
  if (recorderActive()) { RecInfo ri; recorderGetInfo(ri); if (!strcmp(ri.file, path)) { httpd_resp_set_status(req, "409 Conflict"); return httpd_resp_send(req, "file is being recorded", HTTPD_RESP_USE_STRLEN); } }
  if (transfers.load() >= MAX_TRANSFERS || !fileBusyAdd(path)) { httpd_resp_set_status(req, "503 Service Unavailable"); return httpd_resp_send(req, "busy, try again", HTTPD_RESP_USE_STRLEN); }
  transfers++;
  if (!spawnJob(req, fileTask, path, dl[0] == '1', "file", 6144)) { transfers--; fileBusyRemove(path); return httpd_resp_send_500(req); }
  return ESP_OK;
}

// ---------------- AVI playback in the browser (async) ----------------
// Browsers cannot play .avi, but they can show an MJPEG stream in <img>:
// this reads the AVI's 'movi' chunks and sends them at the recorded frame rate.
static uint32_t rd32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static void playTask(void* arg) {
  Job* j = (Job*)arg;
  httpd_req_t* req = j->req;
  File f;
  if (!sdUseBegin()) { httpd_resp_send_404(req); goto done_nouse; }
  f = SD_MMC.open(j->path, FILE_READ);
  if (!f) { httpd_resp_send_404(req); goto done; }
  {
    uint8_t h[12];
    uint32_t usPerFrame = 100000;
    if (f.read(h, 12) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "AVI ", 4)) { httpd_resp_send_404(req); goto done; }
    // walk top-level chunks: read avih frame time, find LIST movi
    bool inMovi = false;
    while (!inMovi && f.read(h, 8) == 8) {
      uint32_t sz = rd32(h + 4);
      if (!memcmp(h, "LIST", 4)) {
        uint8_t t[4]; if (f.read(t, 4) != 4) break;
        if (!memcmp(t, "movi", 4)) { inMovi = true; break; }
        if (!memcmp(t, "hdrl", 4)) {                           // avih is the first sub-chunk
          uint8_t a[12]; if (f.read(a, 12) != 12) break;
          if (!memcmp(a, "avih", 4)) usPerFrame = rd32(a + 8);
          f.seek(f.position() - 12 + sz - 4);
          continue;
        }
        f.seek(f.position() + sz - 4 + (sz & 1));
      } else f.seek(f.position() + sz + (sz & 1));
    }
    if (!inMovi) { httpd_resp_send_404(req); goto done; }
    if (usPerFrame < 10000 || usPerFrame > 2000000) usPerFrame = 100000;
    httpd_resp_set_type(req, STREAM_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    uint8_t* buf = nullptr; size_t cap = 0;
    char part[128];
    int64_t next = esp_timer_get_time();
    while (f.read(h, 8) == 8) {
      uint32_t sz = rd32(h + 4);
      if (!memcmp(h, "idx1", 4)) break;
      if (memcmp(h + 2, "dc", 2) && memcmp(h + 2, "db", 2)) { f.seek(f.position() + sz + (sz & 1)); continue; }
      if (sz == 0 || sz > 2 * 1024 * 1024) break;
      if (sz > cap) { free(buf); cap = (sz + 16383) & ~16383u; buf = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM); if (!buf) break; }
      if (f.read(buf, sz) != sz) break;
      if (sz & 1) f.seek(f.position() + 1);
      int64_t wait = next - esp_timer_get_time();
      if (wait > 1000) vTaskDelay(pdMS_TO_TICKS(wait / 1000));
      next += usPerFrame;
      size_t hl = snprintf(part, sizeof(part), STREAM_PART, (unsigned)sz, 0, 0);
      if (httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != ESP_OK ||
          httpd_resp_send_chunk(req, part, hl) != ESP_OK ||
          httpd_resp_send_chunk(req, (const char*)buf, sz) != ESP_OK) break;
    }
    free(buf);
  }
done:
  if (f) f.close();
  sdUseEnd();
done_nouse:
  fileBusyRemove(j->path);
  transfers--;
  endJob(j);
}
static esp_err_t play_handler(httpd_req_t* req) {
  char path[48];
  if (!queryParam(req, "path", path, sizeof(path)) || !pathAllowed(path, false) || strncmp(path, "/videos/", 8)) return httpd_resp_send_404(req);
  if (recorderActive()) { RecInfo ri; recorderGetInfo(ri); if (!strcmp(ri.file, path)) return httpd_resp_send_404(req); }
  if (transfers.load() >= MAX_TRANSFERS || !fileBusyAdd(path)) { httpd_resp_set_status(req, "503 Service Unavailable"); return httpd_resp_send(req, "busy", HTTPD_RESP_USE_STRLEN); }
  transfers++;
  if (!spawnJob(req, playTask, path, false, "play", 6144)) { transfers--; fileBusyRemove(path); return httpd_resp_send_500(req); }
  return ESP_OK;
}

// ---------------- file list / delete ----------------
static esp_err_t files_handler(httpd_req_t* req) {
  char dir[16] = "/photos";
  queryParam(req, "dir", dir, sizeof(dir));
  if (strcmp(dir, "/photos") && strcmp(dir, "/videos") && strcmp(dir, "/")) return httpd_resp_send_404(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (!sdUseBegin()) return httpd_resp_send(req, "{\"ok\":false,\"files\":[]}", HTTPD_RESP_USE_STRLEN);
  RecInfo ri; recorderGetInfo(ri);
  httpd_resp_send_chunk(req, "{\"ok\":true,\"files\":[", HTTPD_RESP_USE_STRLEN);
  File d = SD_MMC.open(dir);
  int n = 0;
  if (d && d.isDirectory()) {
    File f = d.openNextFile();
    char item[200];
    while (f && n < 500) {
      const char* nm = strrchr(f.name(), '/'); nm = nm ? nm + 1 : f.name();
      if (nm[0] && nm[0] != '.' && strcmp(nm, "System Volume Information")) {
        char full[48];
        snprintf(full, sizeof(full), "%s%s%s", strcmp(dir, "/") ? dir : "", "/", nm);
        time_t t = f.getLastWrite();
        bool rec = ri.active && !strcmp(ri.file, full);
        snprintf(item, sizeof(item), "%s{\"n\":\"%s\",\"p\":\"%s\",\"d\":%d,\"s\":%lu,\"t\":%ld,\"rec\":%d}",
                 n ? "," : "", nm, full, f.isDirectory() ? 1 : 0, (unsigned long)f.size(), (long)(t > 1600000000 ? t : 0), rec ? 1 : 0);
        httpd_resp_send_chunk(req, item, HTTPD_RESP_USE_STRLEN);
        n++;
      }
      f.close();
      f = d.openNextFile();
    }
    d.close();
  }
  sdUseEnd();
  httpd_resp_send_chunk(req, "]}", 2);
  return httpd_resp_send_chunk(req, nullptr, 0);
}
static esp_err_t delete_handler(httpd_req_t* req) {
  char path[48];
  if (!queryParam(req, "path", path, sizeof(path)) || !pathAllowed(path, true)) return sendResult(req, false, "Not allowed", path);
  RecInfo ri; recorderGetInfo(ri);
  if ((ri.active && !strcmp(ri.file, path)) || fileBusy(path)) return sendResult(req, false, "File is in use (recording / downloading)", path);
  if (!sdUseBegin()) return sendResult(req, false, "SD card not available", path);
  File f = SD_MMC.open(path, FILE_READ);
  size_t sz = f ? f.size() : 0;
  bool isDir = f && f.isDirectory();
  if (f) f.close();
  bool ok = !isDir && SD_MMC.remove(path);
  sdUseEnd();
  if (ok) { storageAddUsed(-(int64_t)sz); storageCountChange(!strncmp(path, "/videos/", 8), -1); }
  return sendResult(req, ok, ok ? "Deleted" : "Delete failed", path);
}

// ---------------- photo / recording API ----------------
static esp_err_t photo_handler(httpd_req_t* req) {
  char name[48], msg[128];
  bool ok = photoCapture(name, sizeof(name), msg, sizeof(msg));
  return sendResult(req, ok, msg, name);
}
static esp_err_t recstart_handler(httpd_req_t* req) {
  char msg[128];
  bool ok = sessionRecStart(msg, sizeof(msg));
  return sendResult(req, ok, msg, "");
}
static esp_err_t recstop_handler(httpd_req_t* req) {
  char msg[160];
  bool ok = sessionRecStop(msg, sizeof(msg));
  RecInfo ri; recorderGetInfo(ri);
  return sendResult(req, ok, msg, ri.lastFile);
}
// ---------------- stream session ----------------
static esp_err_t streamstart_handler(httpd_req_t* req) {
  char msg[160];
  bool ok = streamStart(msg, sizeof(msg));
  RecInfo ri; recorderGetInfo(ri);
  return sendResult(req, ok, msg, ri.active ? ri.file : "");
}
static esp_err_t streamstop_handler(httpd_req_t* req) {
  char msg[160];
  bool ok = streamStop(msg, sizeof(msg));
  RecInfo ri; recorderGetInfo(ri);
  return sendResult(req, ok, msg, ri.lastFile);
}
static esp_err_t autorec_handler(httpd_req_t* req) {
  char v[4] = "";
  if (!queryParam(req, "on", v, sizeof(v))) return sendResult(req, false, "bad request", "");
  autoRecSet(v[0] == '1');
  return sendResult(req, true, autoRecGet() ? "Auto-record ON: START STREAM also records" : "Auto-record OFF: START STREAM = live view only", "");
}
static esp_err_t time_handler(httpd_req_t* req) {
  char e[16], tz[8] = "0";
  if (queryParam(req, "epoch", e, sizeof(e))) { queryParam(req, "tz", tz, sizeof(tz)); clockSet(strtoul(e, nullptr, 10), atoi(tz)); }
  return sendResult(req, clockValid(), clockValid() ? "clock set" : "clock not set", "");
}
static esp_err_t status_handler(httpd_req_t* req) {
  char js[2300];
  size_t n = statusJson(js, sizeof(js));
  if (!n) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, js, n);
}

// ---------------- camera settings (only what the sensor supports) ----------------
static esp_err_t control_handler(httpd_req_t* req) {
  char var[24], val[12];
  if (!queryParam(req, "var", var, sizeof(var)) || !queryParam(req, "val", val, sizeof(val))) return sendResult(req, false, "bad request", "");
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return sendResult(req, false, "camera not ready", "");
  int v = atoi(val), res = -1;
  if (!strcmp(var, "framesize") || !strcmp(var, "quality")) {                 // custom size / quality (advanced)
    char msg[112];                                                             // recording -> new file, safely
    bool ok = videoCustomSet(!strcmp(var, "framesize"), v, msg, sizeof(msg));
    return sendResult(req, ok, msg, "");
  }
  else if (!strcmp(var, "brightness")) { if (v >= -2 && v <= 2) res = s->set_brightness(s, v); }
  else if (!strcmp(var, "contrast"))   { if (v >= -2 && v <= 2) res = s->set_contrast(s, v); }
  else if (!strcmp(var, "saturation")) { if (v >= -2 && v <= 2) res = s->set_saturation(s, v); }
  else if (!strcmp(var, "hmirror"))    res = s->set_hmirror(s, v ? 1 : 0);
  else if (!strcmp(var, "vflip"))      res = s->set_vflip(s, v ? 1 : 0);
  return sendResult(req, res == 0, res == 0 ? "ok" : "not supported / out of range", "");
}
// LOW / MEDIUM / HIGH preset and/or frame-rate mode:  /api/video?preset=0..2&fps=0..2
static esp_err_t video_handler(httpd_req_t* req) {
  char v[4];
  bool ok = true;
  char msg[140] = "ok";
  if (queryParam(req, "fps", v, sizeof(v))) {
    int m = atoi(v);
    videoFpsModeSet(m);
    int lim = streamFpsLimit();
    if (lim) snprintf(msg, sizeof(msg), "Frame rate: %s (max %d fps)", m == FPSM_SAVER ? "Data saver" : "Normal", lim);
    else strlcpy(msg, "Frame rate: Max (as fast as the network allows)", sizeof(msg));
  }
  if (queryParam(req, "preset", v, sizeof(v))) ok = videoPresetSet(atoi(v), msg, sizeof(msg));
  return sendResult(req, ok, msg, "");
}

// ---------------- WiFi setup (scan / save / forget) ----------------
static void urlDecode(char* s) {
  char* w = s;
  for (char* r = s; *r; r++) {
    if (*r == '%' && isxdigit((unsigned char)r[1]) && isxdigit((unsigned char)r[2])) { char hx[3] = {r[1], r[2], 0}; *w++ = (char)strtol(hx, nullptr, 16); r += 2; }
    else if (*r == '+') *w++ = ' ';
    else *w++ = *r;
  }
  *w = 0;
}
static esp_err_t wifiscan_handler(httpd_req_t* req) {
  char* js = (char*)malloc(3072);
  if (!js) return httpd_resp_send_500(req);
  size_t n = wifiScanJson(js, 3072);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t r = n ? httpd_resp_send(req, js, n) : httpd_resp_send(req, "{\"ok\":false,\"nets\":[]}", HTTPD_RESP_USE_STRLEN);
  free(js);
  return r;
}
// body: ssid=<urlencoded>&pass=<urlencoded>   (sent as POST body, not in the URL)
static esp_err_t wifisave_handler(httpd_req_t* req) {
  char body[400];
  int len = req->content_len;
  if (len <= 0 || len >= (int)sizeof(body)) return sendResult(req, false, "bad request", "");
  int got = 0;
  while (got < len) { int r = httpd_req_recv(req, body + got, len - got); if (r <= 0) return sendResult(req, false, "receive failed", ""); got += r; }
  body[got] = 0;
  char ssid[100] = "", pass[200] = "";
  if (httpd_query_key_value(body, "ssid", ssid, sizeof(ssid)) != ESP_OK) return sendResult(req, false, "WiFi name missing", "");
  httpd_query_key_value(body, "pass", pass, sizeof(pass));
  urlDecode(ssid); urlDecode(pass);
  if (!ssid[0] || strlen(ssid) > 32) return sendResult(req, false, "WiFi name must be 1-32 characters", "");
  if (strlen(pass) > 64 || (pass[0] && strlen(pass) < 8)) return sendResult(req, false, "Password must be 8-64 characters (or empty for an open network)", "");
  if (recorderActive()) return sendResult(req, false, "Stop the stream first (recording must be saved - the camera restarts)", "");
  int r = wifiSaveNet(ssid, pass);
  if (r == 1) return sendResult(req, false, "3 networks already saved - remove one first", ssid);
  return sendResult(req, r == 0, r == 0 ? "Saved. Camera restarts and joins this WiFi" : "Could not save", ssid);
}
static esp_err_t wififorget_handler(httpd_req_t* req) {
  char v[4] = "";
  if (!queryParam(req, "i", v, sizeof(v))) return sendResult(req, false, "bad request", "");
  bool ok = wifiForgetNet(atoi(v));
  return sendResult(req, ok, ok ? "Removed (used from the next restart)" : "No such network", "");
}

// ---------------- optional password (HTTP Basic auth, WEB_PASS in cam_system.h) ----------------
static char authExpect[160] = "";                      // "Basic base64(user:pass)", empty = no password
static void authInit() {
  if (!WEB_PASS[0]) return;
  char up[100]; unsigned char b64[140]; size_t ol = 0;
  snprintf(up, sizeof(up), "%s:%s", WEB_USER, WEB_PASS);
  if (mbedtls_base64_encode(b64, sizeof(b64), &ol, (const unsigned char*)up, strlen(up)) == 0) {
    b64[ol] = 0;
    snprintf(authExpect, sizeof(authExpect), "Basic %s", (const char*)b64);
  }
}
static esp_err_t guarded(httpd_req_t* req) {           // every route except the phone probes
  if (authExpect[0]) {
    char got[160] = "";
    size_t l = httpd_req_get_hdr_value_len(req, "Authorization");
    if (l == 0 || l >= sizeof(got) || httpd_req_get_hdr_value_str(req, "Authorization", got, sizeof(got)) != ESP_OK || strcmp(got, authExpect)) {
      httpd_resp_set_status(req, "401 Unauthorized");
      httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"RescueCam\"");
      return httpd_resp_send(req, "login required", HTTPD_RESP_USE_STRLEN);
    }
  }
  return ((esp_err_t (*)(httpd_req_t*))req->user_ctx)(req);
}

// ---------------- dashboard + phone connectivity checks ----------------
static esp_err_t index_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, DASHBOARD_HTML, sizeof(DASHBOARD_HTML) - 1);
}
static esp_err_t probe204_handler(httpd_req_t* req) { httpd_resp_set_status(req, "204 No Content"); return httpd_resp_send(req, nullptr, 0); }
static esp_err_t probeApple_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>", HTTPD_RESP_USE_STRLEN);
}

void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 28;
  config.max_open_sockets = 10;          // LWIP has 16; httpd keeps 3 for itself
  config.stack_size = 8192;
  config.lru_purge_enable = true;
  config.send_wait_timeout = 5;
  config.recv_wait_timeout = 5;
  struct U { const char* uri; httpd_method_t m; esp_err_t (*h)(httpd_req_t*); };
  static const U uris[] = {
    {"/", HTTP_GET, index_handler},
    {"/stream", HTTP_GET, stream_handler},
    {"/capture", HTTP_GET, capture_handler},
    {"/api/status", HTTP_GET, status_handler},
    {"/api/photo", HTTP_POST, photo_handler},
    {"/api/rec/start", HTTP_POST, recstart_handler},
    {"/api/rec/stop", HTTP_POST, recstop_handler},
    {"/api/files", HTTP_GET, files_handler},
    {"/api/file", HTTP_GET, file_handler},
    {"/api/play", HTTP_GET, play_handler},
    {"/api/delete", HTTP_POST, delete_handler},
    {"/api/time", HTTP_GET, time_handler},
    {"/control", HTTP_GET, control_handler},
    {"/generate_204", HTTP_GET, probe204_handler},
    {"/gen_204", HTTP_GET, probe204_handler},
    {"/hotspot-detect.html", HTTP_GET, probeApple_handler},
    {"/api/wifi/scan", HTTP_GET, wifiscan_handler},
    {"/api/wifi/save", HTTP_POST, wifisave_handler},
    {"/api/wifi/forget", HTTP_POST, wififorget_handler},
    {"/api/stream/start", HTTP_POST, streamstart_handler},
    {"/api/stream/stop", HTTP_POST, streamstop_handler},
    {"/api/autorec", HTTP_POST, autorec_handler},
    {"/api/video", HTTP_POST, video_handler},
  };
  authInit();
  if (httpd_start(&server, &config) != ESP_OK) { Serial.println("[WEB] server start FAILED"); return; }
  for (const U& u : uris) {
    httpd_uri_t x = {};
    bool probe = u.h == probe204_handler || u.h == probeApple_handler;
    x.uri = u.uri; x.method = u.m;
    x.handler = probe ? u.h : guarded;                 // probes stay open (phone "internet check")
    x.user_ctx = probe ? nullptr : (void*)u.h;
    httpd_register_uri_handler(server, &x);
  }
  Serial.printf("[WEB] server started on port 80%s\n", authExpect[0] ? " (password protected)" : "");
}
