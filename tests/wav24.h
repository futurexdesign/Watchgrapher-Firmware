/**
 * @file wav24.h
 * @brief Minimal streaming reader for stereo 24-bit PCM WAV files (host
 *        tests only).
 *
 * Reads the layout of the handoff test vectors ({@code docs/handoff/vectors})
 * and of {@code channel.read_wav24()}: RIFF/WAVE, a {@code fmt } chunk with
 * PCM (1) or WAVE_FORMAT_EXTENSIBLE (0xFFFE), 2 channels, 24 bits, then a
 * {@code data} chunk of interleaved little-endian 3-byte samples (L, R).
 * Unknown chunks are skipped. Samples are sign-extended to int32, exactly as
 * the firmware's {@code pcm1808_slot_to_pcm24()} delivers them.
 *
 * Frames are read in caller-sized blocks, so the tests feed the library the
 * way the firmware does (240-frame DMA blocks) without loading a whole
 * 37 MB vector into memory.
 *
 * Host only: uses stdio. Header-only.
 *
 * @see tests/test_quartz_vectors.cpp
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

namespace wav24 {

/** Closes a {@code FILE*} for {@code std::unique_ptr}. */
struct FileCloser {
  /** @param f File to close (may be null). */
  void operator()(std::FILE *f) const noexcept {
    if (f != nullptr) {
      std::fclose(f);
    }
  }
};

/**
 * Stereo 24-bit WAV reader. Owns the open file (RAII).
 */
class Reader {
public:
  /**
   * Opens a file and parses its header.
   *
   * @param path File path.
   * @param error Receives a message on failure.
   * @return {@code true} if the file is stereo 24-bit PCM.
   */
  bool open(const std::string &path, std::string &error) {
    file_.reset(std::fopen(path.c_str(), "rb"));
    if (!file_) {
      error = "cannot open " + path;
      return false;
    }
    std::array<uint8_t, 12> riff{};
    if (!read_bytes(riff.data(), riff.size()) || !tag_is(riff.data(), "RIFF") ||
        !tag_is(riff.data() + 8, "WAVE")) {
      error = "not a RIFF/WAVE file";
      return false;
    }
    bool have_fmt = false;
    for (;;) {
      std::array<uint8_t, 8> hdr{};
      if (!read_bytes(hdr.data(), hdr.size())) {
        error = "no data chunk";
        return false;
      }
      const uint32_t size = le32(hdr.data() + 4);
      if (tag_is(hdr.data(), "fmt ")) {
        std::array<uint8_t, 16> fmt{};
        if (size < fmt.size() || !read_bytes(fmt.data(), fmt.size())) {
          error = "short fmt chunk";
          return false;
        }
        const uint16_t tag = le16(fmt.data());
        channels_ = le16(fmt.data() + 2);
        rate_ = le32(fmt.data() + 4);
        bits_ = le16(fmt.data() + 14);
        if ((tag != 1u && tag != 0xFFFEu) || channels_ != 2u || bits_ != 24u) {
          error = "not stereo 24-bit PCM";
          return false;
        }
        if (!skip(size - static_cast<uint32_t>(fmt.size()) + (size & 1u))) {
          error = "truncated fmt chunk";
          return false;
        }
        have_fmt = true;
      } else if (tag_is(hdr.data(), "data")) {
        if (!have_fmt) {
          error = "data before fmt";
          return false;
        }
        frames_left_ = size / 6u;
        frames_ = frames_left_;
        return true;
      } else if (!skip(size + (size & 1u))) {
        error = "truncated chunk";
        return false;
      }
    }
  }

  /** @return Sample rate (Hz). */
  [[nodiscard]] uint32_t rate() const noexcept { return rate_; }

  /** @return Total frames in the data chunk. */
  [[nodiscard]] std::size_t frames() const noexcept { return frames_; }

  /**
   * Reads up to {@code n} frames.
   *
   * @param left Receives left samples (sign-extended 24-bit).
   * @param right Receives right samples.
   * @param n Frames wanted, at most {@link kMaxBlock}.
   * @return Frames read (0 at end of data).
   */
  std::size_t read(int32_t *left, int32_t *right, std::size_t n) {
    if (n > kMaxBlock) {
      n = kMaxBlock;
    }
    if (n > frames_left_) {
      n = frames_left_;
    }
    if (n == 0u || !read_bytes(raw_.data(), n * 6u)) {
      return 0u;
    }
    for (std::size_t i = 0; i < n; i++) {
      left[i] = s24(raw_.data() + 6u * i);
      right[i] = s24(raw_.data() + 6u * i + 3u);
    }
    frames_left_ -= n;
    return n;
  }

  /** Largest block {@link read} returns. */
  static constexpr std::size_t kMaxBlock = 4096u;

private:
  /** @return {@code true} if the 4 bytes at {@code p} spell {@code tag}. */
  static bool tag_is(const uint8_t *p, const char *tag) noexcept {
    return p[0] == static_cast<uint8_t>(tag[0]) &&
           p[1] == static_cast<uint8_t>(tag[1]) &&
           p[2] == static_cast<uint8_t>(tag[2]) &&
           p[3] == static_cast<uint8_t>(tag[3]);
  }

  /** @return Little-endian uint16 at {@code p}. */
  static uint16_t le16(const uint8_t *p) noexcept {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
  }

  /** @return Little-endian uint32 at {@code p}. */
  static uint32_t le32(const uint8_t *p) noexcept {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  }

  /** @return Sign-extended little-endian 24-bit sample at {@code p}. */
  static int32_t s24(const uint8_t *p) noexcept {
    const uint32_t u = static_cast<uint32_t>(p[0]) |
                       (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16);
    // Two's-complement sign extension without implementation-defined shifts.
    return static_cast<int32_t>(u) - (((u & 0x800000u) != 0u) ? 0x1000000 : 0);
  }

  /** @return {@code true} if exactly {@code n} bytes were read. */
  bool read_bytes(uint8_t *dst, std::size_t n) {
    return std::fread(dst, 1u, n, file_.get()) == n;
  }

  /** @return {@code true} if {@code n} bytes were skipped. */
  bool skip(uint32_t n) {
    return std::fseek(file_.get(), static_cast<long>(n), SEEK_CUR) == 0;
  }

  std::unique_ptr<std::FILE, FileCloser> file_; /**< Open file. */
  uint32_t rate_ = 0;                           /**< Sample rate (Hz). */
  uint16_t channels_ = 0;                       /**< Channels (must be 2). */
  uint16_t bits_ = 0;           /**< Bits per sample (must be 24). */
  std::size_t frames_ = 0;      /**< Frames in the data chunk. */
  std::size_t frames_left_ = 0; /**< Frames not yet read. */
  std::array<uint8_t, kMaxBlock * 6u> raw_{}; /**< Read buffer. */
};

} // namespace wav24
