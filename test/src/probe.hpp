// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_TEST_PROBE_HPP
#define QRB_ROS_ADAPTER_TEST_PROBE_HPP

//
// What both sides of the stream probe share: the probe every frame carries,
// the statistics of a stream, and the parameters. Neither ROS nor QCNode is
// used here.
//
// A byte payload (DmaBuf Contracts) is laid out as
//
//   [ ProbeHeader | pattern ... | sentinel ]
//
// A sender writes the pattern once per buffer and per frame rewrites only the
// header and the sentinel; a receiver checks both and one pattern word every
// kSampleStride bytes. An Inline Contract carries the same values as exact
// integers in an array of doubles.
//
// The ROS side takes its parameters from the built-in defaults, a JSON
// parameter file, then ROS parameters; the HCM side from its sample attributes.
// The Flow is not a parameter: both sides read it from the Adapter's static
// config.
//

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <bitset>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace qrb::adapter::stream_probe
{

constexpr std::uint32_t kProbeMagic = 0x50535251u;  // "QRSP" in little-endian bytes

/** @brief The start of every frame payload. */
struct ProbeHeader
{
  std::uint32_t magic;
  std::uint32_t streamTag;
  std::uint64_t sequence;
  std::uint64_t sendTimeNs;  // CLOCK_MONOTONIC of the sender
};
static_assert(sizeof(ProbeHeader) == 24, "ProbeHeader must have no padding");

constexpr std::size_t kProbeBytes = sizeof(ProbeHeader);
constexpr std::size_t kSentinelBytes = sizeof(std::uint64_t);
/** @brief The smallest byte payload that holds a probe and a sentinel. */
constexpr std::size_t kMinPayloadBytes = kProbeBytes + kSentinelBytes;
/** @brief Distance between two pattern words a receiver checks. */
constexpr std::size_t kSampleStride = 4096;

/** @brief CLOCK_MONOTONIC in nanoseconds, comparable across processes of one machine. */
inline std::uint64_t MonotonicNowNs()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ull +
         static_cast<std::uint64_t>(now.tv_nsec);
}

/** @brief The tag of the stream on `topic` (FNV-1a), so streams need no configured tag. */
inline std::uint32_t StreamTagOf(std::string_view topic)
{
  std::uint32_t hash = 2166136261u;
  for (char c : topic) {
    hash ^= static_cast<std::uint8_t>(c);
    hash *= 16777619u;
  }
  return hash;
}

inline std::uint64_t Mix(std::uint64_t value)
{
  // splitmix64 finalizer
  value += 0x9E3779B97F4A7C15ull;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
  return value ^ (value >> 31);
}

inline std::uint64_t PatternWord(std::uint32_t streamTag, std::uint64_t offset)
{
  return Mix((static_cast<std::uint64_t>(streamTag) << 32) ^ offset);
}

inline std::uint64_t SentinelWord(std::uint32_t streamTag, std::uint64_t sequence)
{
  return Mix(~sequence ^ (static_cast<std::uint64_t>(streamTag) * 0xD6E8FEB86659FD93ull));
}

/** @brief What a receiver found in one frame. */
enum class FrameCheck
{
  kOk,
  kBadPayload,  // missing, unreadable, or too small for a probe
  kBadLayout,   // message fields other than the payload are not the sender's
  kBadMagic,
  kForeignStream,  // a valid probe of another stream
  kBadSentinel,
  kBadPattern,
};

inline const char * FrameCheckName(FrameCheck check)
{
  switch (check) {
    case FrameCheck::kOk:
      return "ok";
    case FrameCheck::kBadPayload:
      return "bad_payload";
    case FrameCheck::kBadLayout:
      return "bad_layout";
    case FrameCheck::kBadMagic:
      return "bad_magic";
    case FrameCheck::kForeignStream:
      return "foreign_stream";
    case FrameCheck::kBadSentinel:
      return "bad_sentinel";
    case FrameCheck::kBadPattern:
      return "bad_pattern";
  }
  return "unknown";
}

struct FrameReading
{
  FrameCheck check{FrameCheck::kBadPayload};
  ProbeHeader header{};
  /** @brief Where the check failed, for kBadSentinel and kBadPattern. */
  std::uint64_t badOffset{0};
};

// --- byte payloads -------------------------------------------------------------

/** @brief Write the pattern of `streamTag` over a whole buffer, once per allocation. */
inline void FillPattern(std::uint8_t * data, std::size_t size, std::uint32_t streamTag)
{
  std::size_t offset = 0;
  for (; offset + sizeof(std::uint64_t) <= size; offset += sizeof(std::uint64_t)) {
    const std::uint64_t word = PatternWord(streamTag, offset);
    std::memcpy(data + offset, &word, sizeof(word));
  }
  if (offset < size) {
    const std::uint64_t word = PatternWord(streamTag, offset);
    std::memcpy(data + offset, &word, size - offset);
  }
}

/** @brief Stamp frame `sequence` into a payload of `size` bytes holding the pattern. */
inline void StampFrame(std::uint8_t * data,
    std::size_t size,
    std::uint32_t streamTag,
    std::uint64_t sequence,
    std::uint64_t sendTimeNs)
{
  const ProbeHeader header{kProbeMagic, streamTag, sequence, sendTimeNs};
  std::memcpy(data, &header, sizeof(header));
  const std::uint64_t sentinel = SentinelWord(streamTag, sequence);
  std::memcpy(data + size - kSentinelBytes, &sentinel, sizeof(sentinel));
}

/** @brief Check the probe, the sentinel and one pattern word per kSampleStride. */
inline FrameReading CheckFrame(const std::uint8_t * data, std::size_t size, std::uint32_t streamTag)
{
  FrameReading reading;
  if (data == nullptr || size < kMinPayloadBytes)
    return reading;
  std::memcpy(&reading.header, data, sizeof(reading.header));
  if (reading.header.magic != kProbeMagic) {
    reading.check = FrameCheck::kBadMagic;
    return reading;
  }
  if (reading.header.streamTag != streamTag) {
    reading.check = FrameCheck::kForeignStream;
    return reading;
  }
  const std::size_t sentinelOffset = size - kSentinelBytes;
  std::uint64_t sentinel = 0;
  std::memcpy(&sentinel, data + sentinelOffset, sizeof(sentinel));
  if (sentinel != SentinelWord(streamTag, reading.header.sequence)) {
    reading.check = FrameCheck::kBadSentinel;
    reading.badOffset = sentinelOffset;
    return reading;
  }
  for (std::size_t offset = kSampleStride; offset + sizeof(std::uint64_t) <= sentinelOffset;
      offset += kSampleStride) {
    std::uint64_t word = 0;
    std::memcpy(&word, data + offset, sizeof(word));
    if (word != PatternWord(streamTag, offset)) {
      reading.check = FrameCheck::kBadPattern;
      reading.badOffset = offset;
      return reading;
    }
  }
  reading.check = FrameCheck::kOk;
  return reading;
}

// --- Image payloads ------------------------------------------------------------

//
// The payload of an image is its valid bytes, row after row and plane after
// plane, as a byte payload above. Each side lays those rows out at its own
// stride: the HCM probe as the QCNode Camera sample allocates rgb8, the ROS
// probe as qrb_ros_transport does. So the check covers the repack between the
// two as well as the bytes.
//

/** @brief Width of a probe image: not a multiple of 128, so the two sides' strides differ. */
constexpr std::uint32_t kProbeImageWidth = 1928;
constexpr std::uint64_t kProbeImageRowBytes = kProbeImageWidth * 3;  // rgb8
/** @brief The HCM row stride, ALIGN(width * 3, 16) as the QCNode Camera sample uses for rgb8. */
constexpr std::uint64_t kProbeImageHcmStride = (kProbeImageRowBytes + 15) / 16 * 16;

/** @brief Rows of a probe image whose HCM storage fits `payloadBytes`; 0 if none fits. */
inline std::uint32_t ProbeImageHeight(std::uint64_t payloadBytes)
{
  return static_cast<std::uint32_t>(payloadBytes / kProbeImageHcmStride);
}

/** @brief Where the valid rows of one plane lie in a buffer. */
struct ProbeImagePlane
{
  std::uint64_t offset{0};
  std::uint64_t stride{0};
  std::uint64_t rowBytes{0};
  std::uint64_t rows{0};
};

struct ProbeImageLayout
{
  std::array<ProbeImagePlane, 2> planes{};
  std::size_t planeCount{0};

  std::uint64_t LogicalBytes() const
  {
    std::uint64_t bytes = 0;
    for (std::size_t i = 0; i < planeCount; ++i)
      bytes += planes[i].rowBytes * planes[i].rows;
    return bytes;
  }
};

/**
 * @brief Call `span( at, logical, count )` for each run of the `size` payload
 * bytes at logical `offset`: `count` bytes at `at` in `data`, starting at
 * payload offset `logical`.
 */
template <typename Byte>
inline void ForEachImageSpan(Byte * data,
    const ProbeImageLayout & layout,
    std::uint64_t offset,
    std::uint64_t size,
    const std::function<void(Byte *, std::uint64_t, std::uint64_t)> & span)
{
  std::uint64_t logical = 0;
  for (std::size_t p = 0; p < layout.planeCount && size > 0; ++p) {
    const ProbeImagePlane & plane = layout.planes[p];
    for (std::uint64_t row = 0; row < plane.rows && size > 0; ++row, logical += plane.rowBytes) {
      if (offset >= logical + plane.rowBytes)
        continue;
      const std::uint64_t inRow = offset - logical;
      const std::uint64_t count = std::min(size, plane.rowBytes - inRow);
      span(data + plane.offset + row * plane.stride + inRow, offset, count);
      offset += count;
      size -= count;
    }
  }
}

/** @brief Write the pattern of `streamTag` into the valid rows, once per allocation. */
inline void FillImagePattern(std::uint8_t * data,
    const ProbeImageLayout & layout,
    std::uint32_t streamTag)
{
  std::vector<std::uint8_t> payload(static_cast<std::size_t>(layout.LogicalBytes()));
  FillPattern(payload.data(), payload.size(), streamTag);
  ForEachImageSpan<std::uint8_t>(data, layout, 0, payload.size(),
      [&payload](std::uint8_t * at, std::uint64_t logical, std::uint64_t count) {
        std::memcpy(at, payload.data() + logical, static_cast<std::size_t>(count));
      });
}

/** @brief Stamp frame `sequence` into an image whose rows hold the pattern. */
inline void StampImage(std::uint8_t * data,
    const ProbeImageLayout & layout,
    std::uint32_t streamTag,
    std::uint64_t sequence,
    std::uint64_t sendTimeNs)
{
  const ProbeHeader header{kProbeMagic, streamTag, sequence, sendTimeNs};
  const std::uint64_t sentinel = SentinelWord(streamTag, sequence);
  const auto * headerBytes = reinterpret_cast<const std::uint8_t *>(&header);
  const auto * sentinelBytes = reinterpret_cast<const std::uint8_t *>(&sentinel);
  const std::uint64_t sentinelOffset = layout.LogicalBytes() - kSentinelBytes;
  ForEachImageSpan<std::uint8_t>(data, layout, 0, sizeof(header),
      [&](std::uint8_t * at, std::uint64_t logical, std::uint64_t count) {
        std::memcpy(at, headerBytes + logical, static_cast<std::size_t>(count));
      });
  ForEachImageSpan<std::uint8_t>(data, layout, sentinelOffset, kSentinelBytes,
      [&](std::uint8_t * at, std::uint64_t logical, std::uint64_t count) {
        std::memcpy(
            at, sentinelBytes + (logical - sentinelOffset), static_cast<std::size_t>(count));
      });
}

/** @brief Gather the valid rows and check them as a byte payload. */
inline FrameReading CheckImage(const std::uint8_t * data,
    const ProbeImageLayout & layout,
    std::uint32_t streamTag)
{
  std::vector<std::uint8_t> payload(static_cast<std::size_t>(layout.LogicalBytes()));
  ForEachImageSpan<const std::uint8_t>(data, layout, 0, payload.size(),
      [&payload](const std::uint8_t * at, std::uint64_t logical, std::uint64_t count) {
        std::memcpy(payload.data() + logical, at, static_cast<std::size_t>(count));
      });
  return CheckFrame(payload.data(), payload.size(), streamTag);
}

/** @brief Whether two clock readings in nanoseconds are within `toleranceNs`. */
inline bool WithinNs(std::uint64_t a, std::uint64_t b, std::uint64_t toleranceNs)
{
  return (a > b ? a - b : b - a) <= toleranceNs;
}

// --- Inline payloads -----------------------------------------------------------

//
// An array of doubles, e.g. a pose covariance, as
//   [ magic, streamTag, sequence, sendTimeNs >> 32, sendTimeNs & 0xFFFFFFFF,
//     pattern ..., sentinel ]
// Every value is an integer below 2^53, so it is exact in a double; pattern
// and sentinel keep the low 32 bits of their words.
//

constexpr std::size_t kProbeDoubles = 5;
constexpr std::size_t kMinProbeDoubles = kProbeDoubles + 1;

inline double LowWord(std::uint64_t value)
{
  return static_cast<double>(value & 0xFFFFFFFFull);
}

inline void StampDoubles(double * values,
    std::size_t count,
    std::uint32_t streamTag,
    std::uint64_t sequence,
    std::uint64_t sendTimeNs)
{
  values[0] = static_cast<double>(kProbeMagic);
  values[1] = static_cast<double>(streamTag);
  values[2] = static_cast<double>(sequence);
  values[3] = static_cast<double>(sendTimeNs >> 32);
  values[4] = LowWord(sendTimeNs);
  for (std::size_t i = kProbeDoubles; i + 1 < count; ++i) {
    values[i] = LowWord(PatternWord(streamTag, i));
  }
  values[count - 1] = LowWord(SentinelWord(streamTag, sequence));
}

inline FrameReading CheckDoubles(const double * values, std::size_t count, std::uint32_t streamTag)
{
  FrameReading reading;
  if (values == nullptr || count < kMinProbeDoubles)
    return reading;
  const auto word = [](double value) {
    return value >= 0.0 && value < 18446744073709551616.0 ? static_cast<std::uint64_t>(value) :
                                                            ~std::uint64_t{0};
  };
  reading.header.magic = static_cast<std::uint32_t>(word(values[0]));
  reading.header.streamTag = static_cast<std::uint32_t>(word(values[1]));
  reading.header.sequence = word(values[2]);
  reading.header.sendTimeNs = (word(values[3]) << 32) | word(values[4]);
  if (values[0] != static_cast<double>(kProbeMagic)) {
    reading.check = FrameCheck::kBadMagic;
    return reading;
  }
  if (values[1] != static_cast<double>(streamTag)) {
    reading.check = FrameCheck::kForeignStream;
    return reading;
  }
  if (values[count - 1] != LowWord(SentinelWord(streamTag, reading.header.sequence))) {
    reading.check = FrameCheck::kBadSentinel;
    reading.badOffset = count - 1;
    return reading;
  }
  for (std::size_t i = kProbeDoubles; i + 1 < count; ++i) {
    if (values[i] != LowWord(PatternWord(streamTag, i))) {
      reading.check = FrameCheck::kBadPattern;
      reading.badOffset = i;
      return reading;
    }
  }
  reading.check = FrameCheck::kOk;
  return reading;
}

//
// A message with a header stamp but too few doubles for StampDoubles(), e.g. a Vector3Stamped
// with its three, as
//   [ streamTag, sequence, sentinel ]
// and the send time as the header stamp itself (sec and nanosec of the CLOCK_MONOTONIC time, so
// exact to the nanosecond), which the caller passes to the checker. There is no magic and no
// pattern: the stream tag and the sentinel tell a foreign or a damaged frame apart.
//

constexpr std::size_t kTripleDoubles = 3;

inline void StampTriple(double * values, std::uint32_t streamTag, std::uint64_t sequence)
{
  values[0] = static_cast<double>(streamTag);
  values[1] = static_cast<double>(sequence);
  values[2] = LowWord(SentinelWord(streamTag, sequence));
}

/** @param sendTimeNs the header stamp of the message, in nanoseconds */
inline FrameReading CheckTriple(const double * values,
    std::uint64_t sendTimeNs,
    std::uint32_t streamTag)
{
  FrameReading reading;
  if (values == nullptr)
    return reading;
  const auto word = [](double value) {
    return value >= 0.0 && value < 18446744073709551616.0 ? static_cast<std::uint64_t>(value) :
                                                            ~std::uint64_t{0};
  };
  reading.header.magic = kProbeMagic;
  reading.header.streamTag = static_cast<std::uint32_t>(word(values[0]));
  reading.header.sequence = word(values[1]);
  reading.header.sendTimeNs = sendTimeNs;
  if (values[0] != static_cast<double>(streamTag)) {
    reading.check = FrameCheck::kForeignStream;
    return reading;
  }
  if (values[2] != LowWord(SentinelWord(streamTag, reading.header.sequence))) {
    reading.check = FrameCheck::kBadSentinel;
    reading.badOffset = 2;
    return reading;
  }
  reading.check = FrameCheck::kOk;
  return reading;
}

// --- detection frames ----------------------------------------------------------
//
// The frame both sides build and check for the detection Contracts: two detections, the first
// with two hypotheses and the second with one. The probe is the array of 36 doubles of the
// covariance of the first hypothesis of the first detection; the Contract keeps a covariance that
// is not all zero. Every other value is exact in a double, so it must arrive unchanged.
//

namespace detection_probe
{

constexpr const char * kFrameId = "stream_probe";
constexpr std::size_t kDetections = 2;
constexpr std::size_t kCovarianceDoubles = 36;

inline const char * Id(std::size_t detection)
{
  return detection == 0 ? "probe-0" : "probe-1";
}

inline std::size_t HypothesisCount(std::size_t detection)
{
  return detection == 0 ? 2 : 1;
}

inline const char * ClassId(std::size_t detection, std::size_t hypothesis)
{
  return detection == 0 ? (hypothesis == 0 ? "probe-a" : "probe-b") : "probe-c";
}

inline double Score(std::size_t detection, std::size_t hypothesis)
{
  return detection == 0 ? (hypothesis == 0 ? 0.75 : 0.25) : 0.5;
}

/** @brief Number `component` of the bounding box of `detection`, in the order of its fields. */
inline double BoxValue(std::size_t detection, std::size_t component)
{
  return 10.0 * static_cast<double>(detection + 1) + 0.5 * static_cast<double>(component);
}

}  // namespace detection_probe

// --- DMA-BUF mapping -------------------------------------------------------------

/**
 * @brief A probe's own mapping of a DMA-BUF fd.
 *
 * The probes never map through lib_mem_dmabuf::DmaBuffer: its single mapping
 * is shared with whoever else holds the buffer (the ROS IPC Bridge, the
 * qrb_ros_transport type adapter), which map and unmap it on their own.
 */
class DmaMapping
{
public:
  DmaMapping(int fd, std::size_t size, bool writable) : fd_(fd), size_(size), writable_(writable)
  {
    if (fd < 0 || size == 0)
      return;
    void * addr =
        mmap(nullptr, size, writable ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, fd, 0);
    if (addr != MAP_FAILED)
      data_ = static_cast<std::uint8_t *>(addr);
  }
  ~DmaMapping()
  {
    if (data_ != nullptr)
      munmap(data_, size_);
  }
  DmaMapping(const DmaMapping &) = delete;
  DmaMapping & operator=(const DmaMapping &) = delete;

  bool Valid() const { return data_ != nullptr; }
  std::uint8_t * Data() const { return data_; }
  std::size_t Size() const { return size_; }

  /** @brief Begin CPU access; pair with EndAccess(). */
  bool BeginAccess() { return Sync(DMA_BUF_SYNC_START); }
  bool EndAccess() { return Sync(DMA_BUF_SYNC_END); }

private:
  bool Sync(std::uint64_t phase)
  {
    dma_buf_sync sync{};
    sync.flags = phase | (writable_ ? DMA_BUF_SYNC_RW : DMA_BUF_SYNC_READ);
    int result = 0;
    do {
      result = ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync);
    } while (result != 0 && (errno == EINTR || errno == EAGAIN));
    return result == 0;
  }

  int fd_;
  std::size_t size_;
  bool writable_;
  std::uint8_t * data_{nullptr};
};

// --- Statistics ------------------------------------------------------------------

/** @brief printf into a std::string. */
[[gnu::format(printf, 1, 2)]] inline std::string FormatText(const char * format, ...)
{
  va_list args;
  va_start(args, format);
  va_list measure;
  va_copy(measure, args);
  const int n = std::vsnprintf(nullptr, 0, format, measure);
  va_end(measure);
  std::string out(n > 0 ? static_cast<std::size_t>(n) : 0, '\0');
  if (n > 0)
    std::vsnprintf(out.data(), out.size() + 1, format, args);
  va_end(args);
  return out;
}

inline unsigned long long U64(std::uint64_t value)
{
  return static_cast<unsigned long long>(value);
}

inline double NsToMs(std::uint64_t ns)
{
  return static_cast<double>(ns) / 1e6;
}

/**
 * @brief The start of a --log line: the CLOCK_MONOTONIC time in seconds, so
 * the logs of both sides line up, and the line type.
 */
inline std::string LogPrefix(std::uint64_t nowNs, const char * type)
{
  return FormatText("%.6f %-7s ", static_cast<double>(nowNs) / 1e9, type);
}

/**
 * @brief Received, lost, duplicate and reordered frames from the sequence
 * numbers of one stream.
 *
 * Lost frames are the gaps between the lowest and the highest sequence
 * received, so it does not matter which side stops first. Duplicates are
 * recognized within the last kWindow sequences; an older sequence counts as
 * reordered.
 */
class SequenceTracker
{
public:
  static constexpr std::uint64_t kWindow = 4096;

  enum class Arrival
  {
    kInOrder,
    kReordered,
    kDuplicate,
  };

  /** @param[out] gap sequences skipped since the highest one so far, 0 unless in order */
  Arrival Record(std::uint64_t sequence, std::uint64_t & gap)
  {
    gap = 0;
    if (unique_ == 0) {
      lowest_ = highest_ = sequence;
      seen_.reset();
      seen_.set(Slot(sequence));
      ++unique_;
      return Arrival::kInOrder;
    }
    if (sequence > highest_) {
      // Slots between the old and the new highest are no longer seen.
      const std::uint64_t advance = sequence - highest_;
      gap = advance - 1;
      if (advance >= kWindow)
        seen_.reset();
      else {
        for (std::uint64_t s = highest_ + 1; s <= sequence; ++s)
          seen_.reset(Slot(s));
      }
      highest_ = sequence;
      seen_.set(Slot(sequence));
      ++unique_;
      return Arrival::kInOrder;
    }
    if (highest_ - sequence < kWindow) {
      if (seen_.test(Slot(sequence))) {
        ++duplicates_;
        return Arrival::kDuplicate;
      }
      seen_.set(Slot(sequence));
    }
    lowest_ = std::min(lowest_, sequence);
    ++unique_;
    ++reordered_;
    return Arrival::kReordered;
  }

  std::uint64_t Unique() const { return unique_; }
  std::uint64_t Duplicates() const { return duplicates_; }
  std::uint64_t Reordered() const { return reordered_; }

  static const char * Name(Arrival arrival)
  {
    switch (arrival) {
      case Arrival::kInOrder:
        return "in_order";
      case Arrival::kReordered:
        return "reordered";
      case Arrival::kDuplicate:
        return "duplicate";
    }
    return "unknown";
  }

  std::uint64_t Lost() const
  {
    if (unique_ == 0)
      return 0;
    const std::uint64_t span = highest_ - lowest_ + 1;
    return span > unique_ ? span - unique_ : 0;
  }

private:
  static std::size_t Slot(std::uint64_t sequence)
  {
    return static_cast<std::size_t>(sequence % kWindow);
  }

  std::bitset<kWindow> seen_;
  std::uint64_t lowest_{0};
  std::uint64_t highest_{0};
  std::uint64_t unique_{0};
  std::uint64_t duplicates_{0};
  std::uint64_t reordered_{0};
};

/**
 * @brief Latency distribution in nanoseconds with a fixed memory footprint,
 * for runs of any length: 64 buckets per power of two, so a percentile is
 * within 1/64 of the true value. Minimum, maximum and mean are exact.
 */
class LatencyHistogram
{
public:
  static constexpr unsigned kSubBits = 6;
  static constexpr std::size_t kSub = std::size_t{1} << kSubBits;
  static constexpr std::size_t kBuckets = (64 - kSubBits + 1) * kSub;

  void Record(std::uint64_t ns)
  {
    ++buckets_[Index(ns)];
    ++count_;
    sum_ += static_cast<long double>(ns);
    min_ = std::min(min_, ns);
    max_ = std::max(max_, ns);
  }

  std::uint64_t Count() const { return count_; }

  /** @brief The value below which a fraction `quantile` of the samples lies. */
  std::uint64_t Percentile(double quantile) const
  {
    if (count_ == 0)
      return 0;
    const auto rank = static_cast<std::uint64_t>(std::ceil(quantile * static_cast<double>(count_)));
    const std::uint64_t target = std::max<std::uint64_t>(rank, 1);
    std::uint64_t seen = 0;
    for (std::size_t i = 0; i < kBuckets; ++i) {
      seen += buckets_[i];
      if (seen >= target)
        return std::min(std::max(UpperBound(i), min_), max_);
    }
    return max_;
  }

  /**
   * @brief The samples recorded since this histogram was `earlier`. Minimum
   * and maximum are those of the outermost buckets, so within 1/64.
   */
  LatencyHistogram Since(const LatencyHistogram & earlier) const
  {
    LatencyHistogram interval;
    if (earlier.count_ > count_)
      return interval;
    std::size_t lowest = kBuckets;
    std::size_t highest = 0;
    for (std::size_t i = 0; i < kBuckets; ++i) {
      interval.buckets_[i] = buckets_[i] - earlier.buckets_[i];
      if (interval.buckets_[i] == 0)
        continue;
      lowest = std::min(lowest, i);
      highest = i;
    }
    interval.count_ = count_ - earlier.count_;
    interval.sum_ = sum_ - earlier.sum_;
    if (interval.count_ == 0 || lowest == kBuckets)
      return LatencyHistogram();
    interval.min_ = std::max(LowerBound(lowest), min_);
    interval.max_ = std::min(UpperBound(highest), max_);
    return interval;
  }

  std::uint64_t Mean() const { return count_ == 0 ? 0 : static_cast<std::uint64_t>(sum_ / count_); }
  std::uint64_t Min() const { return count_ == 0 ? 0 : min_; }
  std::uint64_t Max() const { return max_; }

private:
  static std::size_t Index(std::uint64_t ns)
  {
    if (ns < kSub)
      return static_cast<std::size_t>(ns);
    unsigned msb = 63;
    while ((ns >> msb) == 0)
      --msb;
    const unsigned group = msb - kSubBits + 1;
    const std::uint64_t top = ns >> (msb - kSubBits);  // in [kSub, 2 * kSub)
    return group * kSub + static_cast<std::size_t>(top - kSub);
  }

  static std::uint64_t LowerBound(std::size_t index)
  {
    if (index < kSub)
      return index;
    const std::size_t group = index / kSub;
    return static_cast<std::uint64_t>(kSub + index % kSub) << (group - 1);
  }

  static std::uint64_t UpperBound(std::size_t index)
  {
    if (index < kSub)
      return index;
    const std::size_t group = index / kSub;
    return LowerBound(index) + ((std::uint64_t{1} << (group - 1)) - 1);
  }

  std::array<std::uint64_t, kBuckets> buckets_{};
  std::uint64_t count_{0};
  long double sum_{0};
  std::uint64_t min_{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t max_{0};
};

/** @brief Counters of a sender. */
struct SenderCounters
{
  std::uint64_t sent{0};
  std::uint64_t bytes{0};
  /** @brief Frames skipped because every buffer was still held downstream. */
  std::uint64_t poolExhausted{0};
  std::uint64_t sendErrors{0};
};

/** @brief Counters of a receiver. */
struct ReceiverCounters
{
  std::uint64_t received{0};
  std::uint64_t bytes{0};
  std::uint64_t lost{0};
  std::uint64_t duplicates{0};
  std::uint64_t reordered{0};
  std::uint64_t verifyErrors{0};
  std::uint64_t foreignStream{0};
};

// --- Text of the --log lines: counters that are 0 are left out, latencies in ms ---

inline std::string SenderText(const SenderCounters & c)
{
  std::string text = FormatText("sent %llu", U64(c.sent));
  if (c.poolExhausted != 0)
    text += FormatText("  pool_exhausted %llu", U64(c.poolExhausted));
  if (c.sendErrors != 0)
    text += FormatText("  send_errors %llu", U64(c.sendErrors));
  return text;
}

inline std::string ReceiverText(const ReceiverCounters & c)
{
  std::string text = FormatText("received %llu  lost %llu", U64(c.received), U64(c.lost));
  if (c.duplicates != 0)
    text += FormatText("  duplicates %llu", U64(c.duplicates));
  if (c.reordered != 0)
    text += FormatText("  reordered %llu", U64(c.reordered));
  if (c.verifyErrors != 0)
    text += FormatText("  verify_errors %llu", U64(c.verifyErrors));
  if (c.foreignStream != 0)
    text += FormatText("  foreign_stream %llu", U64(c.foreignStream));
  return text;
}

/** @brief Short end-to-end latency of `h`, for a monitor line. */
inline std::string LatencyText(const LatencyHistogram & h)
{
  if (h.Count() == 0)
    return "e2e -";
  return FormatText("e2e p50 %.3f p99 %.3f max %.3f ms", NsToMs(h.Percentile(0.50)),
      NsToMs(h.Percentile(0.99)), NsToMs(h.Max()));
}

/** @brief Full end-to-end latency of `h`, for the summary. */
inline std::string LatencySummaryText(const LatencyHistogram & h)
{
  if (h.Count() == 0)
    return "e2e latency: no frame measured";
  return FormatText(
      "e2e latency of %llu frames: avg %.3f min %.3f p50 %.3f p95 %.3f p99 %.3f max %.3f ms",
      U64(h.Count()), NsToMs(h.Mean()), NsToMs(h.Min()), NsToMs(h.Percentile(0.50)),
      NsToMs(h.Percentile(0.95)), NsToMs(h.Percentile(0.99)), NsToMs(h.Max()));
}

/**
 * @brief One frame a probe sent or received: a `frame` line of the --log
 * file. Times are CLOCK_MONOTONIC, so the logs of both sides line up.
 */
struct FrameRecord
{
  /** Frames recorded so far, this one included; 0: none yet. */
  std::uint64_t count{0};
  bool received{false};
  std::uint64_t sequence{0};
  std::uint64_t sendNs{0};
  std::uint64_t receiveNs{0};
  std::uint64_t bytes{0};
  /** For a received frame: what its probe showed. */
  FrameCheck check{FrameCheck::kOk};
  std::uint64_t badOffset{0};
  std::uint32_t streamTag{0};
  /** Past the warm-up. A measured frame that checked ok has `arrival` and `lostBefore`. */
  bool measured{false};
  SequenceTracker::Arrival arrival{SequenceTracker::Arrival::kInOrder};
  /** Sequences missing right before this one; a later reordered frame may fill some. */
  std::uint64_t lostBefore{0};
};

/**
 * @brief The --log line of `frame`, without the newline: its sequence and, if
 * received, the end-to-end latency and anything unusual about it. `extra` is
 * appended, starting with two spaces.
 */
inline std::string FrameLogLine(const FrameRecord & frame, const std::string & extra = {})
{
  std::string rest;
  if (frame.received) {
    const long long latencyNs =
        static_cast<long long>(frame.receiveNs) - static_cast<long long>(frame.sendNs);
    rest = FormatText("  e2e %.3f ms", static_cast<double>(latencyNs) / 1e6);
    if (frame.check == FrameCheck::kForeignStream)
      rest +=
          FormatText("  BAD %s, stream tag 0x%08x", FrameCheckName(frame.check), frame.streamTag);
    else if (frame.check != FrameCheck::kOk)
      rest +=
          FormatText("  BAD %s at offset %llu", FrameCheckName(frame.check), U64(frame.badOffset));
    else if (!frame.measured)
      rest += "  warm-up";
    else {
      if (frame.arrival != SequenceTracker::Arrival::kInOrder)
        rest += std::string("  ") + SequenceTracker::Name(frame.arrival);
      if (frame.lostBefore != 0)
        rest += FormatText("  %llu lost before", U64(frame.lostBefore));
    }
  }
  rest += extra;
  std::string line = LogPrefix(frame.received ? frame.receiveNs : frame.sendNs, "frame");
  line += rest.empty() ? FormatText("seq %llu", U64(frame.sequence)) :
                         FormatText("seq %-8llu", U64(frame.sequence)) + rest;
  return line;
}

/**
 * @brief What a receiver made of the frames it read. Verification errors and
 * frames of another stream always count; the rest only past the warm-up.
 */
class ReceiverStats
{
public:
  /** @brief Account frame `reading` received at `receiveNs`, of `bytes` payload bytes. */
  FrameRecord
  Record(const FrameReading & reading, std::uint64_t receiveNs, std::uint64_t bytes, bool measured)
  {
    FrameRecord frame;
    frame.count = ++frames_;
    frame.received = true;
    frame.sequence = reading.header.sequence;
    frame.sendNs = reading.header.sendTimeNs;
    frame.receiveNs = receiveNs;
    frame.bytes = bytes;
    frame.check = reading.check;
    frame.badOffset = reading.badOffset;
    frame.streamTag = reading.header.streamTag;
    frame.measured = measured;
    switch (reading.check) {
      case FrameCheck::kOk:
        break;
      case FrameCheck::kForeignStream:
        ++foreignStream_;
        return frame;
      default:
        ++verifyErrors_;
        return frame;
    }
    if (!measured)
      return frame;
    ++received_;
    bytes_ += bytes;
    frame.arrival = sequence_.Record(reading.header.sequence, frame.lostBefore);
    if (frame.arrival == SequenceTracker::Arrival::kDuplicate)
      return frame;
    const std::uint64_t latency =
        receiveNs > reading.header.sendTimeNs ? receiveNs - reading.header.sendTimeNs : 0;
    total_.Record(latency);
    return frame;
  }

  std::uint64_t VerifyErrors() const { return verifyErrors_; }
  std::uint64_t ForeignStream() const { return foreignStream_; }
  std::uint64_t Received() const { return received_; }

  ReceiverCounters Counters() const
  {
    return {received_, bytes_, sequence_.Lost(), sequence_.Duplicates(), sequence_.Reordered(),
        verifyErrors_, foreignStream_};
  }

  /** @brief Latency of every measured frame so far. */
  const LatencyHistogram & Latency() const { return total_; }

private:
  SequenceTracker sequence_;
  LatencyHistogram total_;
  std::uint64_t frames_{0};
  std::uint64_t received_{0};
  std::uint64_t bytes_{0};
  std::uint64_t verifyErrors_{0};
  std::uint64_t foreignStream_{0};
};

static_assert(std::is_trivially_copyable_v<LatencyHistogram> &&
                  std::is_trivially_copyable_v<SenderCounters> &&
                  std::is_trivially_copyable_v<ReceiverCounters> &&
                  std::is_trivially_copyable_v<FrameRecord>,
    "monitoring data of the HCM probe node is copied with memcpy");

// --- Parameters ------------------------------------------------------------------

/** @brief Which side sends: 0 HCM to ROS, 1 ROS to HCM. */
enum class ProbeDirection : int
{
  kHcmToRos = 0,
  kRosToHcm = 1,
};

constexpr std::uint32_t kMaxDepth = 32;  // the product's kMaxQosDepth
constexpr const char * kDefaultAdapterConfig = "adapter_point_cloud_h2r.json";

struct ProbeParams
{
  std::string adapterConfig;             // the Adapter's static config
  double fps{30.0};                      // frames per second, 0: unlimited
  std::uint64_t payloadBytes{1u << 20};  // what a sender sends per frame
  std::uint64_t bufferBytes{32u << 20};  // DMA buffer size of the HCM receiver (R2H)
  double duration{0.0};                  // seconds, 0: until stopped
  double warmup{0.0};                    // seconds
  std::string log;                       // text log file, empty: none
};

/** @brief The built-in defaults; `configDir` holds the default configs. */
inline ProbeParams DefaultParams(const std::string & configDir)
{
  ProbeParams params;
  params.adapterConfig = configDir + "/" + kDefaultAdapterConfig;
  return params;
}

/** @brief The directory of `path`, or "." when it has none. */
inline std::string DirectoryOf(const std::string & path)
{
  const std::size_t slash = path.rfind('/');
  if (slash == std::string::npos)
    return ".";
  return slash == 0 ? "/" : path.substr(0, slash);
}

/**
 * @brief Apply the keys of JSON object `object` from `source` to `params`. A
 * relative path is taken relative to `baseDir`, or as given when it is empty.
 * @throws std::invalid_argument on an unknown key or a value of the wrong type.
 */
inline void ApplyJson(ProbeParams & params,
    const nlohmann::json & object,
    const std::string & source,
    const std::string & baseDir = {})
{
  if (!object.is_object())
    throw std::invalid_argument(source + ": not a JSON object");
  const auto fail = [&source](const std::string & key, const char * expected) {
    throw std::invalid_argument(source + ": '" + key + "' must be " + expected);
  };
  const auto text = [&](const std::string & key, const nlohmann::json & value, std::string & out) {
    if (!value.is_string())
      fail(key, "a string");
    out = value.get<std::string>();
  };
  const auto path = [&](const std::string & key, const nlohmann::json & value, std::string & out) {
    text(key, value, out);
    if (!out.empty() && out[0] != '/' && !baseDir.empty())
      out = baseDir + "/" + out;
  };
  const auto unsignedValue = [&](const std::string & key, const nlohmann::json & value,
                                 std::uint64_t maximum) -> std::uint64_t {
    if (!value.is_number_integer() ||
        (!value.is_number_unsigned() && value.get<std::int64_t>() < 0))
      fail(key, "a non-negative integer");
    const std::uint64_t v = value.get<std::uint64_t>();
    if (v > maximum)
      fail(key, ("at most " + std::to_string(maximum)).c_str());
    return v;
  };
  const auto nonNegative = [&](const std::string & key, const nlohmann::json & value) {
    if (!value.is_number())
      fail(key, "a number");
    const double v = value.get<double>();
    if (!std::isfinite(v) || v < 0.0)
      fail(key, "a non-negative number");
    return v;
  };

  for (const auto & [key, value] : object.items()) {
    if (key == "adapter_config")
      path(key, value, params.adapterConfig);
    else if (key == "fps")
      params.fps = nonNegative(key, value);
    else if (key == "payload_bytes")
      params.payloadBytes = unsignedValue(key, value, std::numeric_limits<std::uint32_t>::max());
    else if (key == "buffer_bytes")
      params.bufferBytes = unsignedValue(key, value, std::numeric_limits<std::uint32_t>::max());
    else if (key == "duration")
      params.duration = nonNegative(key, value);
    else if (key == "warmup")
      params.warmup = nonNegative(key, value);
    else if (key == "log")
      path(key, value, params.log);
    else
      throw std::invalid_argument(source + ": unknown key '" + key + "'");
  }
}

/**
 * @brief Apply the parameter file `path`; its relative paths are relative to
 * its directory. @throws std::invalid_argument
 */
inline void ApplyJsonFile(ProbeParams & params, const std::string & path)
{
  std::ifstream in(path);
  if (!in)
    throw std::invalid_argument("cannot open parameter file " + path);
  nlohmann::json object;
  try {
    in >> object;
  } catch (const nlohmann::json::exception & error) {
    throw std::invalid_argument(path + ": " + error.what());
  }
  ApplyJson(params, object, path, DirectoryOf(path));
}

/**
 * @brief Check what one key cannot: combinations. Contract-specific payload
 * limits are checked by the Contract's sample.
 * @throws std::invalid_argument
 */
inline void Validate(const ProbeParams & params)
{
  const auto fail = [](const std::string & message) { throw std::invalid_argument(message); };
  if (params.adapterConfig.empty())
    fail("'adapter_config' must not be empty");
  if (params.duration > 0.0 && params.warmup >= params.duration)
    fail("'warmup' must be shorter than 'duration'");
}

inline nlohmann::json ToJson(const ProbeParams & params)
{
  return nlohmann::json{
      {"adapter_config", params.adapterConfig},
      {"fps", params.fps},
      {"payload_bytes", params.payloadBytes},
      {"buffer_bytes", params.bufferBytes},
      {"duration", params.duration},
      {"warmup", params.warmup},
      {"log", params.log},
  };
}

/** @brief Read file `path` into `out`. @return false if it cannot be read */
inline bool ReadTextFile(const std::string & path, std::string & out)
{
  std::ifstream in(path);
  if (!in)
    return false;
  std::ostringstream text;
  text << in.rdbuf();
  out = text.str();
  return true;
}

// --- The Flow, from the Adapter's static config ------------------------------------

/** @brief The fields of the Adapter's static config the probes follow. */
struct ProbeFlow
{
  ProbeDirection direction{ProbeDirection::kHcmToRos};
  std::string contract{"qrb.contract.sensor.v1.PointCloud2"};
  std::string topic{"/qrb_ros_adapter/stream_probe/points_h2r"};
  std::string reliability{"reliable"};  // the Adapter's QoS defaults, rclcpp::QoS( 10 )
  std::string durability{"volatile"};
  std::uint32_t depth{10};
  std::uint32_t globalBufferId{0};
};

/** @brief The short name of Contract `typeName`: its last dot-separated segment. */
inline std::string ShortContractName(const std::string & typeName)
{
  const std::size_t dot = typeName.rfind('.');
  return dot == std::string::npos ? typeName : typeName.substr(dot + 1);
}

/** @brief Whether `contract` names Contract `typeName`, by full or short name. */
inline bool MatchesContract(const std::string & contract, const std::string & typeName)
{
  return contract == typeName || contract == ShortContractName(typeName);
}

/**
 * @brief Read the fields of `base` from Adapter static config `text`. A field
 * that is missing where the Adapter requires it, or that holds a value the
 * Adapter rejects, keeps its value from `base` and adds a line to `problems`.
 * Fields the probes do not follow are left to the Adapter.
 */
inline ProbeFlow ReadProbeFlow(const std::string & text,
    const ProbeFlow & base,
    std::vector<std::string> & problems)
{
  ProbeFlow flow = base;
  const nlohmann::json root = nlohmann::json::parse(text, nullptr, false);
  if (root.is_discarded()) {
    problems.push_back("not JSON");
    return flow;
  }
  if (!root.is_object() || !root.contains("static") || !root["static"].is_object()) {
    problems.push_back("no \"static\" object");
    return flow;
  }
  const nlohmann::json & node = root["static"];

  const auto required = [&](const char * key) -> const nlohmann::json * {
    auto it = node.find(key);
    if (it != node.end())
      return &*it;
    problems.push_back(std::string("\"") + key + "\" is missing");
    return nullptr;
  };
  const auto oneOf = [&](const nlohmann::json & value, const char * key, const char * a,
                         const char * b, std::string & out) {
    if (value.is_string() && (value == a || value == b))
      out = value.get<std::string>();
    else
      problems.push_back(std::string("\"") + key + "\" must be " + a + " or " + b);
  };
  const auto nonEmpty = [&](const nlohmann::json & value, const char * key, std::string & out) {
    if (value.is_string() && !value.get<std::string>().empty())
      out = value.get<std::string>();
    else
      problems.push_back(std::string("\"") + key + "\" must be a non-empty string");
  };

  if (const nlohmann::json * value = required("contract_type"))
    nonEmpty(*value, "contract_type", flow.contract);
  if (const nlohmann::json * value = required("direction")) {
    if (value->is_number_unsigned() && value->get<std::uint64_t>() <= 1)
      flow.direction = static_cast<ProbeDirection>(value->get<std::uint64_t>());
    else
      problems.push_back("\"direction\" must be 0 (HCM to ROS) or 1 (ROS to HCM)");
  }
  if (const nlohmann::json * value = required("ros_topic"))
    nonEmpty(*value, "ros_topic", flow.topic);

  auto qos = node.find("qos_profile");
  if (qos != node.end()) {
    if (!qos->is_object())
      problems.push_back("\"qos_profile\" must be an object");
    else {
      auto it = qos->find("reliability");
      if (it != qos->end())
        oneOf(*it, "reliability", "reliable", "best_effort", flow.reliability);
      it = qos->find("durability");
      if (it != qos->end())
        oneOf(*it, "durability", "volatile", "transient_local", flow.durability);
      it = qos->find("depth");
      if (it != qos->end()) {
        if (it->is_number_unsigned() && it->get<std::uint64_t>() >= 1 &&
            it->get<std::uint64_t>() <= kMaxDepth)
          flow.depth = static_cast<std::uint32_t>(it->get<std::uint64_t>());
        else
          problems.push_back("\"depth\" must be in [1, " + std::to_string(kMaxDepth) + "]");
      }
    }
  }

  auto map = node.find("globalBufferIdMap");
  if (map != node.end()) {
    const nlohmann::json * id = nullptr;
    if (map->is_array() && map->size() == 1 && (*map)[0].is_object() && (*map)[0].contains("id"))
      id = &(*map)[0]["id"];
    if (id != nullptr && id->is_number_unsigned() &&
        id->get<std::uint64_t>() < std::numeric_limits<std::uint32_t>::max())
      flow.globalBufferId = static_cast<std::uint32_t>(id->get<std::uint64_t>());
    else
      problems.push_back("\"globalBufferIdMap\" must hold one entry with a valid \"id\"");
  }
  return flow;
}

/** @brief `flow` in one line, as printed by the HCM side or the ROS side. */
inline std::string FlowText(const ProbeFlow & flow, bool hcmSide)
{
  const bool hcmToRos = flow.direction == ProbeDirection::kHcmToRos;
  std::string text = "direction " + std::to_string(static_cast<int>(flow.direction)) +
                     (hcmToRos ? " (HCM to ROS" : " (ROS to HCM") +
                     (hcmToRos == hcmSide ? ", this side sends), " : ", this side receives), ") +
                     ShortContractName(flow.contract) + " on " + flow.topic + ", QoS " +
                     flow.reliability + "/" + flow.durability + "/" + std::to_string(flow.depth);
  if (hcmSide)
    text += ", descriptor at buffer " + std::to_string(flow.globalBufferId);
  return text;
}

}  // namespace qrb::adapter::stream_probe

#endif  // QRB_ROS_ADAPTER_TEST_PROBE_HPP
