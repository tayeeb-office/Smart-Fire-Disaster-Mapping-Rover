// ============================================================================
//  Minimal MJPEG-in-AVI writer (AVI 1.0 / RIFF, one video stream 'MJPG').
//  Each camera JPEG becomes one '00dc' chunk; an idx1 index is written on
//  finish(). The header is re-written every checkpoint so a file cut by a
//  power loss is still readable by VLC/ffmpeg (they rebuild the index).
//
//  File layout (fixed 224-byte header):
//    0   RIFF <size> 'AVI '
//    12  LIST 192 'hdrl'
//    24    avih 56  MainAVIHeader
//    88    LIST 116 'strl'
//    100     strh 56  AVIStreamHeader ('vids' / 'MJPG')
//    164     strf 40  BITMAPINFOHEADER ('MJPG')
//    212 LIST <size> 'movi'
//    224   00dc <len> <jpeg> [pad] ...
//        idx1 <n*16> { '00dc', AVIIF_KEYFRAME, offset (from 'movi'), len } ...
//
//  F must provide: size_t write(const uint8_t*, size_t); bool seek(uint32_t);
//  size_t position(); void flush();
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>

template <class F>
class AviWriter {
 public:
  static const uint32_t HEADER_SIZE = 224;
  static const uint32_t MOVI_FOURCC_POS = 220;       // position of the 'movi' fourcc

  // idx: caller-provided buffer of 2*idxCap uint32 (offset,len pairs), e.g. in PSRAM
  bool begin(F* file, uint16_t w, uint16_t h, uint32_t* idx, uint32_t idxCap) {
    f_ = file; w_ = w; h_ = h; idx_ = idx; cap_ = idxCap;
    n_ = 0; moviBytes_ = 4; maxFrame_ = 0; ok_ = true;
    uint8_t hdr[HEADER_SIZE];
    buildHeader(hdr, 0, 0, false);
    return put(hdr, HEADER_SIZE);
  }
  // false = write error or index full (caller rolls to a new file)
  bool addFrame(const uint8_t* jpg, uint32_t len) {
    if (!ok_ || n_ >= cap_) return false;
    uint32_t chunkPos = HEADER_SIZE + (moviBytes_ - 4);
    uint8_t ch[8];
    memcpy(ch, "00dc", 4); le32(ch + 4, len);
    if (!put(ch, 8) || !put(jpg, len)) return false;
    if (len & 1) { uint8_t z = 0; if (!put(&z, 1)) return false; }
    idx_[2 * n_]     = chunkPos - MOVI_FOURCC_POS;     // offset relative to 'movi' fourcc
    idx_[2 * n_ + 1] = len;
    n_++;
    moviBytes_ += 8 + len + (len & 1);
    if (len > maxFrame_) maxFrame_ = len;
    return true;
  }
  // Rewrite the header with the current counts (no index yet) and flush.
  bool checkpoint(uint32_t durMs) {
    if (!ok_) return false;
    uint32_t end = HEADER_SIZE + (moviBytes_ - 4);
    uint8_t hdr[HEADER_SIZE];
    buildHeader(hdr, durMs, end, false);
    if (!f_->seek(0) || !put(hdr, HEADER_SIZE) || !f_->seek(end)) { ok_ = false; return false; }
    f_->flush();
    return true;
  }
  // Append idx1 and write the final header. File stays open (caller closes).
  bool finish(uint32_t durMs) {
    if (!ok_) return false;
    uint32_t end = HEADER_SIZE + (moviBytes_ - 4);
    if (!f_->seek(end)) { ok_ = false; return false; }
    uint8_t ch[8];
    memcpy(ch, "idx1", 4); le32(ch + 4, n_ * 16);
    if (!put(ch, 8)) return false;
    uint8_t blk[64 * 16];
    uint32_t i = 0;
    while (i < n_) {
      uint32_t k = 0;
      for (; k < 64 && i < n_; k++, i++) {
        uint8_t* e = blk + k * 16;
        memcpy(e, "00dc", 4); le32(e + 4, 0x10);        // AVIIF_KEYFRAME (every JPEG is a key frame)
        le32(e + 8, idx_[2 * i]); le32(e + 12, idx_[2 * i + 1]);
      }
      if (!put(blk, k * 16)) return false;
    }
    uint32_t fileEnd = end + 8 + n_ * 16;
    uint8_t hdr[HEADER_SIZE];
    buildHeader(hdr, durMs, fileEnd, true);
    if (!f_->seek(0) || !put(hdr, HEADER_SIZE) || !f_->seek(fileEnd)) { ok_ = false; return false; }
    f_->flush();
    return true;
  }
  uint32_t frames() const { return n_; }
  uint64_t bytes() const { return (uint64_t)HEADER_SIZE + (moviBytes_ - 4) + 8 + (uint64_t)n_ * 16; }
  bool ok() const { return ok_; }

 private:
  F* f_ = nullptr;
  uint16_t w_ = 0, h_ = 0;
  uint32_t* idx_ = nullptr;
  uint32_t cap_ = 0, n_ = 0, moviBytes_ = 4, maxFrame_ = 0;
  bool ok_ = false;

  static void le32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
  static void le16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
  bool put(const uint8_t* d, size_t len) {
    if (f_->write(d, len) != len) { ok_ = false; return false; }
    return true;
  }
  // fileEnd = total file size known so far (0 = unknown / placeholder)
  void buildHeader(uint8_t* p, uint32_t durMs, uint32_t fileEnd, bool hasIndex) {
    memset(p, 0, HEADER_SIZE);
    // real frame rate from the measured duration (constant-rate AVI)
    uint32_t usPerFrame = 100000, rate = 10000;           // defaults: 10 fps (scale 1000)
    if (n_ > 1 && durMs > 0) {
      usPerFrame = (uint32_t)(((uint64_t)durMs * 1000ULL) / n_);
      if (usPerFrame == 0) usPerFrame = 1;
      rate = (uint32_t)(((uint64_t)n_ * 1000000ULL) / durMs);   // fps * 1000
      if (rate == 0) rate = 1;
    }
    uint32_t riffSize = fileEnd ? fileEnd - 8 : 0;
    memcpy(p + 0, "RIFF", 4); le32(p + 4, riffSize); memcpy(p + 8, "AVI ", 4);
    memcpy(p + 12, "LIST", 4); le32(p + 16, 192); memcpy(p + 20, "hdrl", 4);
    memcpy(p + 24, "avih", 4); le32(p + 28, 56);
    uint8_t* a = p + 32;
    le32(a + 0, usPerFrame);                                // dwMicroSecPerFrame
    le32(a + 4, usPerFrame ? (uint32_t)((uint64_t)maxFrame_ * 1000000ULL / usPerFrame) : 0);  // dwMaxBytesPerSec
    le32(a + 8, 0);                                         // dwPaddingGranularity
    le32(a + 12, hasIndex ? 0x10 : 0);                      // dwFlags: AVIF_HASINDEX
    le32(a + 16, n_);                                       // dwTotalFrames
    le32(a + 20, 0);                                        // dwInitialFrames
    le32(a + 24, 1);                                        // dwStreams
    le32(a + 28, maxFrame_ + 8);                            // dwSuggestedBufferSize
    le32(a + 32, w_); le32(a + 36, h_);                     // dwWidth, dwHeight
    memcpy(p + 88, "LIST", 4); le32(p + 92, 116); memcpy(p + 96, "strl", 4);
    memcpy(p + 100, "strh", 4); le32(p + 104, 56);
    uint8_t* s = p + 108;
    memcpy(s + 0, "vids", 4); memcpy(s + 4, "MJPG", 4);     // fccType, fccHandler
    le32(s + 8, 0); le16(s + 12, 0); le16(s + 14, 0);       // dwFlags, wPriority, wLanguage
    le32(s + 16, 0);                                        // dwInitialFrames
    le32(s + 20, 1000);                                     // dwScale
    le32(s + 24, rate);                                     // dwRate  (fps = rate/scale)
    le32(s + 28, 0);                                        // dwStart
    le32(s + 32, n_);                                       // dwLength (frames)
    le32(s + 36, maxFrame_ + 8);                            // dwSuggestedBufferSize
    le32(s + 40, 0xFFFFFFFF);                               // dwQuality (-1 = default)
    le32(s + 44, 0);                                        // dwSampleSize (0 = variable)
    le16(s + 48, 0); le16(s + 50, 0); le16(s + 52, w_); le16(s + 54, h_);   // rcFrame
    memcpy(p + 164, "strf", 4); le32(p + 168, 40);
    uint8_t* b = p + 172;
    le32(b + 0, 40); le32(b + 4, w_); le32(b + 8, h_);      // biSize, biWidth, biHeight
    le16(b + 12, 1); le16(b + 14, 24);                      // biPlanes, biBitCount
    memcpy(b + 16, "MJPG", 4);                              // biCompression
    le32(b + 20, (uint32_t)w_ * h_ * 3);                    // biSizeImage
    memcpy(p + 212, "LIST", 4); le32(p + 216, moviBytes_); memcpy(p + 220, "movi", 4);
  }
};
