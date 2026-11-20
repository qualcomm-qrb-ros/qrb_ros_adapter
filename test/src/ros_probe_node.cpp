// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "ros_probe_node.hpp"

#define QRB_CONTRACT_MAPPING_INCLUDE_ROS_HEADERS
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING_INCLUDE_ROS_HEADERS

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace qrb::adapter::stream_probe
{

// --- The probe in each Contract's ROS message ------------------------------------
//
// A new Contract needs a RosProbeSample<> for its ROS message type; without it
// this package does not compile.

/** @brief What became of one frame a sender tried to send. */
enum class SendOutcome
{
  kSent,
  kPoolExhausted,  // every buffer is still held downstream; nothing was sent
  kError,
};

template <typename>
inline constexpr bool kAlwaysFalse = false;

/**
 * @brief The probe in ROS message `RosMessageT`. Specialize it for the ROS
 * message of every QRB_CONTRACT_MAPPING entry, with:
 *
 *   class Source            // a sender's frames
 *   {
 *       // throws on a bad payload size
 *       Source( std::uint64_t payloadBytes, std::uint32_t depth, std::uint32_t streamTag );
 *       SendOutcome Next( std::uint64_t sequence, std::unique_ptr<RosMessageT> &out,
 *           std::uint64_t &sendTimeNs );
 *       std::uint64_t PayloadBytes() const;
 *   };
 *   static FrameReading Check( const RosMessageT &, std::uint32_t streamTag,
 *       std::uint64_t &payloadBytes );
 *
 * Next() stamps the send time itself, as late as it can, and gives it back in
 * `sendTimeNs`.
 */
template <typename RosMessageT>
struct RosProbeSample
{
  static_assert(kAlwaysFalse<RosMessageT>,
      "add a RosProbeSample<> for the new Contract in test/src/ros_probe_node.cpp");
};

// --- geometry.v1.PoseWithCovarianceStamped (inline) ---------------------------

/** The probe fills the covariance; the pose is the identity. */
template <>
struct RosProbeSample<adapter::PoseRosMessage>
{
  using Message = adapter::PoseRosMessage;
  static constexpr const char * kFrameId = "stream_probe";

  class Source
  {
  public:
    Source(std::uint64_t, std::uint32_t, std::uint32_t streamTag) : streamTag_(streamTag) {}

    SendOutcome Next(std::uint64_t sequence,
        std::unique_ptr<Message> & out,
        std::uint64_t & sendTimeNs)
    {
      out = std::make_unique<Message>();
      out->header.frame_id = kFrameId;
      out->pose.pose.orientation.w = 1.0;
      const std::uint64_t now = MonotonicNowNs();
      sendTimeNs = now;
      out->header.stamp.sec = static_cast<std::int32_t>(now / 1000000000ull);
      out->header.stamp.nanosec = static_cast<std::uint32_t>(now % 1000000000ull);
      StampDoubles(
          out->pose.covariance.data(), out->pose.covariance.size(), streamTag_, sequence, now);
      return SendOutcome::kSent;
    }

    std::uint64_t PayloadBytes() const { return sizeof(Message{}.pose.covariance); }

  private:
    std::uint32_t streamTag_;
  };

  static FrameReading Check(const Message & pose,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(pose.pose.covariance);
    FrameReading reading =
        CheckDoubles(pose.pose.covariance.data(), pose.pose.covariance.size(), streamTag);
    if (reading.check == FrameCheck::kOk &&
        (pose.header.frame_id != kFrameId || pose.pose.pose.orientation.w != 1.0))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- nav.v1.Odometry (inline) -------------------------------------------------

/**
 * The probe fills the pose covariance; the pose is the identity,
 * the twist is zero and its covariance carries a fixed marker. The layout is
 * the one HcmProbeSample<> uses, so either side checks what the other sends.
 */
template <>
struct RosProbeSample<adapter::OdometryRosMessage>
{
  using Message = adapter::OdometryRosMessage;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr const char * kChildFrameId = "stream_probe_base";
  static constexpr double kTwistCovarianceMarker = 0.5;

  class Source
  {
  public:
    Source(std::uint64_t, std::uint32_t, std::uint32_t streamTag) : streamTag_(streamTag) {}

    SendOutcome Next(std::uint64_t sequence,
        std::unique_ptr<Message> & out,
        std::uint64_t & sendTimeNs)
    {
      out = std::make_unique<Message>();
      out->header.frame_id = kFrameId;
      out->child_frame_id = kChildFrameId;
      out->pose.pose.orientation.w = 1.0;
      out->twist.covariance.fill(kTwistCovarianceMarker);
      const std::uint64_t now = MonotonicNowNs();
      sendTimeNs = now;
      out->header.stamp.sec = static_cast<std::int32_t>(now / 1000000000ull);
      out->header.stamp.nanosec = static_cast<std::uint32_t>(now % 1000000000ull);
      StampDoubles(
          out->pose.covariance.data(), out->pose.covariance.size(), streamTag_, sequence, now);
      return SendOutcome::kSent;
    }

    std::uint64_t PayloadBytes() const { return sizeof(Message{}.pose.covariance); }

  private:
    std::uint32_t streamTag_;
  };

  static FrameReading Check(const Message & odom,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(odom.pose.covariance);
    FrameReading reading =
        CheckDoubles(odom.pose.covariance.data(), odom.pose.covariance.size(), streamTag);
    if (reading.check == FrameCheck::kOk &&
        (odom.header.frame_id != kFrameId || odom.child_frame_id != kChildFrameId ||
            odom.pose.pose.orientation.w != 1.0 ||
            odom.twist.covariance[0] != kTwistCovarianceMarker ||
            odom.twist.covariance[35] != kTwistCovarianceMarker))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- sensor.v1.PointCloud2 (DMA-BUF) ------------------------------------------

/**
 * @brief DMA-BUFs of one sender, each mapped once and filled with the pattern
 * once. A buffer comes back when the last holder of its message lets go,
 * possibly after the pool is gone; then it is freed instead.
 */
class DmaPayloadPool
{
public:
  static constexpr const char * kHeap = "/dev/dma_heap/system";

  struct Slot
  {
    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer;
    std::unique_ptr<DmaMapping> mapping;
  };

  DmaPayloadPool(std::size_t bytes, std::size_t capacity, std::uint32_t streamTag)
    : bytes_(bytes), capacity_(capacity), streamTag_(streamTag), state_(std::make_shared<State>())
  {
  }

  ~DmaPayloadPool()
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->closed = true;
    state_->free.clear();
  }

  DmaPayloadPool(const DmaPayloadPool &) = delete;
  DmaPayloadPool & operator=(const DmaPayloadPool &) = delete;

  /**
   * @brief A free buffer, or a new one below the capacity.
   * @param[out] slot its mapping, valid while `buffer` is held
   */
  SendOutcome Acquire(std::shared_ptr<lib_mem_dmabuf::DmaBuffer> & buffer, Slot *& slot)
  {
    std::shared_ptr<Slot> acquired;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if (!state_->free.empty()) {
        acquired = std::move(state_->free.back());
        state_->free.pop_back();
      } else if (allocated_ >= capacity_) {
        return SendOutcome::kPoolExhausted;
      }
    }
    if (acquired == nullptr) {
      acquired = Allocate();
      if (acquired == nullptr)
        return SendOutcome::kError;
      ++allocated_;
    }
    std::shared_ptr<void> lease(nullptr, [state = state_, acquired](void *) {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->closed)
        state->free.push_back(acquired);
    });
    slot = acquired.get();
    buffer = std::shared_ptr<lib_mem_dmabuf::DmaBuffer>(lease, acquired->buffer.get());
    return SendOutcome::kSent;
  }

private:
  struct State
  {
    std::mutex mutex;
    std::vector<std::shared_ptr<Slot>> free;
    bool closed{false};
  };

  std::shared_ptr<Slot> Allocate() const
  {
    auto slot = std::make_shared<Slot>();
    slot->buffer = lib_mem_dmabuf::DmaBuffer::alloc(bytes_, kHeap);
    if (slot->buffer == nullptr || slot->buffer->fd() < 0)
      return nullptr;
    slot->mapping = std::make_unique<DmaMapping>(slot->buffer->fd(), bytes_, true);
    if (!slot->mapping->Valid() || !slot->mapping->BeginAccess())
      return nullptr;
    FillPattern(slot->mapping->Data(), bytes_, streamTag_);
    if (!slot->mapping->EndAccess())
      return nullptr;
    return slot;
  }

  std::size_t bytes_;
  std::size_t capacity_;
  std::uint32_t streamTag_;
  std::size_t allocated_{0};  // touched by the sending thread only
  std::shared_ptr<State> state_;
};

/** One row of `payload_bytes / 16` points x, y, z, intensity (FLOAT32). */
template <>
struct RosProbeSample<adapter::PointCloudRosMessage>
{
  using Message = adapter::PointCloudRosMessage;
  using PointField = adapter::PointFieldRosMessage;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr std::uint32_t kPointStep = 16;
  static constexpr const char * kFieldNames[] = {"x", "y", "z", "intensity"};
  static constexpr std::size_t kFieldCount = sizeof(kFieldNames) / sizeof(kFieldNames[0]);

  class Source
  {
  public:
    Source(std::uint64_t payloadBytes, std::uint32_t depth, std::uint32_t streamTag)
      : streamTag_(streamTag)
      , bytes_(payloadBytes)
      ,
      // The DDS queue, the Bridge's latest sample, one at HCM, and spares.
      pool_(static_cast<std::size_t>(payloadBytes), depth + 4u, streamTag)
    {
      if (bytes_ < kMinPayloadBytes || bytes_ % kPointStep != 0)
        throw std::invalid_argument("'payload_bytes' of PointCloud2 must be a multiple of " +
                                    std::to_string(kPointStep) + " and at least " +
                                    std::to_string(kMinPayloadBytes));
      // Fail at start-up rather than at the first frame without a heap.
      std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer;
      DmaPayloadPool::Slot * slot = nullptr;
      if (pool_.Acquire(buffer, slot) != SendOutcome::kSent)
        throw std::runtime_error(
            std::string("cannot allocate a DMA-BUF from ") + DmaPayloadPool::kHeap);
    }

    SendOutcome Next(std::uint64_t sequence,
        std::unique_ptr<Message> & out,
        std::uint64_t & sendTimeNs)
    {
      std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer;
      DmaPayloadPool::Slot * slot = nullptr;
      const SendOutcome acquired = pool_.Acquire(buffer, slot);
      if (acquired != SendOutcome::kSent)
        return acquired;

      auto cloud = std::make_unique<Message>(Message());
      cloud->header.frame_id = kFrameId;
      cloud->height = 1;
      cloud->width = static_cast<std::uint32_t>(bytes_ / kPointStep);
      for (std::size_t i = 0; i < kFieldCount; ++i) {
        PointField field;
        field.name = kFieldNames[i];
        field.offset = static_cast<std::uint32_t>(i * 4);
        field.datatype = PointField::FLOAT32;
        field.count = 1;
        cloud->fields.push_back(field);
      }
      cloud->is_bigendian = false;
      cloud->point_step = kPointStep;
      cloud->row_step = static_cast<std::uint32_t>(bytes_);
      cloud->is_dense = true;
      cloud->data_fd = std::move(buffer);

      if (!slot->mapping->BeginAccess())
        return SendOutcome::kError;
      const std::uint64_t now = MonotonicNowNs();
      sendTimeNs = now;
      StampFrame(
          slot->mapping->Data(), static_cast<std::size_t>(bytes_), streamTag_, sequence, now);
      if (!slot->mapping->EndAccess())
        return SendOutcome::kError;
      cloud->header.stamp.sec = static_cast<std::int32_t>(now / 1000000000ull);
      cloud->header.stamp.nanosec = static_cast<std::uint32_t>(now % 1000000000ull);
      out = std::move(cloud);
      return SendOutcome::kSent;
    }

    std::uint64_t PayloadBytes() const { return bytes_; }

  private:
    std::uint32_t streamTag_;
    std::uint64_t bytes_;
    DmaPayloadPool pool_;
  };

  static FrameReading Check(const Message & cloud,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = static_cast<std::uint64_t>(cloud.row_step) * cloud.height;
    FrameReading reading;
    if (cloud.data_fd == nullptr || cloud.data_fd->fd() < 0 || cloud.data_fd->size() <= 0 ||
        payloadBytes > static_cast<std::uint64_t>(cloud.data_fd->size()))
      return reading;

    DmaMapping mapping(cloud.data_fd->fd(), static_cast<std::size_t>(payloadBytes), false);
    if (!mapping.Valid() || !mapping.BeginAccess())
      return reading;
    reading = CheckFrame(mapping.Data(), mapping.Size(), streamTag);
    const bool ended = mapping.EndAccess();
    if (reading.check != FrameCheck::kOk)
      return reading;
    if (!ended) {
      reading.check = FrameCheck::kBadPayload;
      return reading;
    }
    if (!HasSenderLayout(cloud))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }

private:
  static bool HasSenderLayout(const Message & cloud)
  {
    if (cloud.header.frame_id != kFrameId || cloud.height != 1 || cloud.point_step != kPointStep ||
        cloud.row_step != std::uint64_t{cloud.width} * kPointStep || cloud.is_bigendian ||
        !cloud.is_dense || cloud.fields.size() != kFieldCount)
      return false;
    for (std::size_t i = 0; i < kFieldCount; ++i) {
      const PointField & field = cloud.fields[i];
      if (field.name != kFieldNames[i] || field.offset != i * 4 ||
          field.datatype != PointField::FLOAT32 || field.count != 1)
        return false;
    }
    return true;
  }
};

namespace
{

constexpr std::uint64_t kNsPerSecond = 1000000000ull;
/** Frames whose problems are logged one by one; later ones are only counted. */
constexpr std::uint64_t kLoggedProblems = 10;
/** Seconds between the monitor lines of the --log file. */
constexpr std::uint64_t kLogMonitorSeconds = 5;

std::uint64_t SecondsToNs(double seconds)
{
  return static_cast<std::uint64_t>(seconds * static_cast<double>(kNsPerSecond));
}

double NsToSeconds(std::uint64_t ns)
{
  return static_cast<double>(ns) / static_cast<double>(kNsPerSecond);
}

rclcpp::QoS MakeQos(const ProbeFlow & flow)
{
  rclcpp::QoS qos{rclcpp::KeepLast(flow.depth)};
  if (flow.reliability == "best_effort")
    qos.best_effort();
  else
    qos.reliable();
  if (flow.durability == "transient_local")
    qos.transient_local();
  else
    qos.durability_volatile();
  return qos;
}

/** One line per parameter, shown by `ros2 param describe`. */
struct ParamHelp
{
  const char * name;
  const char * description;
};
constexpr ParamHelp kParamHelp[] = {
    {"params_file", "JSON file of the parameters below"},
    {"adapter_config", "the Adapter's static JSON; the Flow comes from it only"},
    {"fps", "frames per second of a sender, 0: unlimited"},
    {"payload_bytes", "payload of each frame this side sends"},
    {"buffer_bytes", "the HCM receiver's buffer; ignored on this side"},
    {"duration", "seconds to run, 0: until unloaded or a signal"},
    {"warmup", "seconds a receiver does not measure"},
    {"log",
        "text log: a line per frame sent or received, a monitor line every 5 s, events and the "
        "summary; empty: none"},
};

const char * DescribeParam(const std::string & name)
{
  for (const ParamHelp & help : kParamHelp) {
    if (name == help.name)
      return help.description;
  }
  return "";
}

/** `ros2 component load` shows only this message of a failed load, so point to the help. */
constexpr const char * kHelpHint =
    "; parameters: ros2 run qrb_ros_adapter_stream_probe ros_stream_probe_node --help";

}  // namespace

// --- ProbeRecorder ---------------------------------------------------------------

/**
 * @brief Statistics, events and the --log file of one probe. Called from the
 * executor (frames, QoS events, the timer) and from the sending thread.
 */
class ProbeRecorder
{
public:
  ProbeRecorder(rclcpp::Logger logger,
      const ProbeParams & params,
      const ProbeFlow & flow,
      bool sender)
    : logger_(std::move(logger)), params_(params), sender_(sender), lastLogNs_(MonotonicNowNs())
  {
    if (params.log.empty())
      return;
    log_.open(params.log, std::ios::out | std::ios::trunc);
    if (!log_)
      throw std::runtime_error("cannot open log file " + params.log);
    WriteLocked(
        "# qrb_ros_adapter_stream_probe, ROS side. Columns: CLOCK_MONOTONIC seconds (the clock of "
        "the\n"
        "# HCM side's log too), line type, details. Latencies in ms; e2e: probe send to receive.\n"
        "# A frame line per frame. A monitor line every 5 s: rates over those 5 s, totals since "
        "the start.\n" +
        FormatText("# run: fps %g, payload %llu bytes, duration %g s, warmup %g s", params.fps,
            U64(params.payloadBytes), params.duration, params.warmup));
    WriteLocked(LogPrefix(MonotonicNowNs(), "flow") + FlowText(flow, false));
  }

  /** @brief Start the clock of `duration` and `warmup`, once. */
  void Begin(const std::string & event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    BeginLocked(MonotonicNowNs(), event);
  }

  bool Expired(std::uint64_t nowNs) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return ExpiredLocked(nowNs);
  }

  /** @brief Account a received frame. @return whether `duration` is now reached */
  bool OnFrame(const FrameReading & reading, std::uint64_t receiveNs, std::uint64_t bytes)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (done_)
      return false;
    if (!started_) {
      BeginLocked(receiveNs, FormatText("first frame: seq %llu, check %s",
                                 U64(reading.header.sequence), FrameCheckName(reading.check)));
    }
    const bool measured = receiveNs - startNs_ >= SecondsToNs(params_.warmup);
    const FrameRecord frame = stats_.Record(reading, receiveNs, bytes, measured);
    if (log_.is_open())
      log_ << FrameLogLine(frame) << '\n';
    if (reading.check != FrameCheck::kOk && ++problems_ <= kLoggedProblems) {
      RCLCPP_WARN(logger_, "frame %llu: %s at offset %llu (stream tag 0x%08x)%s",
          U64(reading.header.sequence), FrameCheckName(reading.check), U64(reading.badOffset),
          reading.header.streamTag,
          problems_ == kLoggedProblems ? "; further problems are only counted" : "");
    }
    return ExpiredLocked(receiveNs);
  }

  /** @param sequence, sendNs those of the frame, if `outcome` is kSent */
  void
  OnSend(SendOutcome outcome, std::uint64_t bytes, std::uint64_t sequence, std::uint64_t sendNs)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    switch (outcome) {
      case SendOutcome::kSent:
        ++counters_.sent;
        counters_.bytes += bytes;
        if (log_.is_open()) {
          FrameRecord frame;
          frame.count = counters_.sent;
          frame.sequence = sequence;
          frame.sendNs = sendNs;
          frame.bytes = bytes;
          log_ << FrameLogLine(frame) << '\n';
        }
        break;
      case SendOutcome::kPoolExhausted:
        ++counters_.poolExhausted;
        break;
      case SendOutcome::kError:
        if (++counters_.sendErrors <= kLoggedProblems)
          RCLCPP_WARN(logger_, "sending frame %llu failed", U64(counters_.sent));
        break;
    }
  }

  void OnIncompatibleQos(const char * endpoint, int totalCount, const std::string & policy)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    incompatibleQos_ = static_cast<std::uint64_t>(totalCount);
    lastIncompatiblePolicy_ = policy;
    const std::string text = FormatText(
        "incompatible QoS %s: policy %s (total %d)", endpoint, policy.c_str(), totalCount);
    RCLCPP_WARN(logger_, "%s", text.c_str());
    EventLocked(text);
  }

  /** @brief Called once a second with the matched `peers`; writes a monitor line every
   * kLogMonitorSeconds. */
  void Tick(std::size_t peers)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (done_ || !log_.is_open())
      return;
    const std::uint64_t now = MonotonicNowNs();
    if (now - lastLogNs_ < kLogMonitorSeconds * kNsPerSecond - kNsPerSecond / 2)
      return;
    const double interval = NsToSeconds(now - lastLogNs_);
    const std::uint64_t frames = sender_ ? counters_.sent : stats_.Received();
    std::string text = FormatText("fps %.1f  peers %zu  ",
        interval > 0.0 ? static_cast<double>(frames - lastLogFrames_) / interval : 0.0, peers);
    if (sender_)
      text += SenderText(counters_);
    else
      text += ReceiverText(stats_.Counters()) + "  " +
              LatencyText(stats_.Latency().Since(lastLogLatency_));
    if (incompatibleQos_ != 0)
      text += FormatText("  incompatible_qos %llu", U64(incompatibleQos_));
    WriteLocked(LogPrefix(now, "monitor") + text);
    lastLogNs_ = now;
    lastLogFrames_ = frames;
    if (!sender_)
      lastLogLatency_ = stats_.Latency();
  }

  /** @brief Write the summary, once. @return whether this call finished the probe */
  bool Finish(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (done_)
      return false;
    done_ = true;
    const std::uint64_t now = MonotonicNowNs();
    std::vector<std::string> lines;
    lines.push_back(FormatText("end: %s, %s, exit code %d", reason.c_str(),
        started_ ? FormatText("%.1f s since the start", NsToSeconds(now - startNs_)).c_str() :
                   "not started",
        ExitCodeLocked()));
    lines.push_back(
        "stream: " + (sender_ ? SenderText(counters_) : ReceiverText(stats_.Counters())));
    if (!sender_)
      lines.push_back(LatencySummaryText(stats_.Latency()));
    if (incompatibleQos_ != 0)
      lines.push_back(FormatText("incompatible QoS: %llu, last policy %s", U64(incompatibleQos_),
          lastIncompatiblePolicy_.c_str()));
    std::string logged;
    for (const std::string & line : lines) {
      logged += (logged.empty() ? "" : "\n") + LogPrefix(now, "summary") + line;
      RCLCPP_INFO(logger_, "summary: %s", line.c_str());
    }
    WriteLocked(logged);
    return true;
  }

  bool Done() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return done_;
  }

  int ExitCode() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return ExitCodeLocked();
  }

private:
  void BeginLocked(std::uint64_t nowNs, const std::string & event)
  {
    if (started_)
      return;
    started_ = true;
    startNs_ = nowNs;
    RCLCPP_INFO(logger_, "%s", event.c_str());
    EventLocked(event, nowNs);
  }

  bool ExpiredLocked(std::uint64_t nowNs) const
  {
    return started_ && params_.duration > 0.0 && nowNs - startNs_ >= SecondsToNs(params_.duration);
  }

  int ExitCodeLocked() const
  {
    return stats_.VerifyErrors() > 0 || stats_.ForeignStream() > 0 ? 1 : 0;
  }

  void EventLocked(const std::string & detail, std::uint64_t nowNs = MonotonicNowNs())
  {
    WriteLocked(LogPrefix(nowNs, "event") + detail);
  }

  /** Frame lines are written as they come and flushed with the next other line. */
  void WriteLocked(const std::string & text)
  {
    if (!log_.is_open())
      return;
    log_ << text << '\n';
    log_.flush();
  }

  rclcpp::Logger logger_;
  const ProbeParams params_;
  const bool sender_;

  mutable std::mutex mutex_;
  std::ofstream log_;
  bool started_{false};
  bool done_{false};
  std::uint64_t startNs_{0};
  ReceiverStats stats_;
  SenderCounters counters_;
  // Of the last monitor line.
  std::uint64_t lastLogNs_;
  std::uint64_t lastLogFrames_{0};
  LatencyHistogram lastLogLatency_;
  std::uint64_t problems_{0};
  std::uint64_t incompatibleQos_{0};
  std::string lastIncompatiblePolicy_;
};

// --- ProbeEndpoint ---------------------------------------------------------------

/** @brief The publisher or the subscription of a probe, whatever its message type. */
class ProbeEndpoint
{
public:
  virtual ~ProbeEndpoint() = default;
  /** @brief Matched subscriptions of a sender, publishers of a receiver. */
  virtual std::size_t PeerCount() const = 0;
  /** @brief Send frame `sequence`, stamped `sendNs`; for a sender only. */
  virtual SendOutcome Send(std::uint64_t sequence, std::uint64_t & sendNs) = 0;
  virtual std::uint64_t PayloadBytes() const = 0;
};

namespace
{

template <typename RosMessageT>
class TypedEndpoint final : public ProbeEndpoint
{
  using Sample = RosProbeSample<RosMessageT>;

public:
  TypedEndpoint(rclcpp::Node & node,
      const ProbeParams & params,
      const ProbeFlow & flow,
      bool sender,
      ProbeRecorder & recorder,
      std::function<void()> onExpired)
    : streamTag_(StreamTagOf(flow.topic)), recorder_(recorder), onExpired_(std::move(onExpired))
  {
    const rclcpp::QoS qos = MakeQos(flow);
    if (sender) {
      source_.emplace(params.payloadBytes, flow.depth, streamTag_);
      rclcpp::PublisherOptions options;
      options.event_callbacks.incompatible_qos_callback =
          [&recorder](rclcpp::QOSOfferedIncompatibleQoSInfo & info) {
            recorder.OnIncompatibleQos("offered", info.total_count,
                rclcpp::qos_policy_name_from_kind(info.last_policy_kind));
          };
      publisher_ = node.create_publisher<RosMessageT>(flow.topic, qos, options);
    } else {
      rclcpp::SubscriptionOptions options;
      options.event_callbacks.incompatible_qos_callback =
          [&recorder](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
            recorder.OnIncompatibleQos("requested", info.total_count,
                rclcpp::qos_policy_name_from_kind(info.last_policy_kind));
          };
      subscription_ = node.create_subscription<RosMessageT>(
          flow.topic, qos,
          [this](std::shared_ptr<const RosMessageT> message) { OnMessage(*message); }, options);
    }
  }

  std::size_t PeerCount() const override
  {
    if (publisher_ != nullptr)
      return publisher_->get_subscription_count() +
             publisher_->get_intra_process_subscription_count();
    return subscription_ != nullptr ? subscription_->get_publisher_count() : 0;
  }

  SendOutcome Send(std::uint64_t sequence, std::uint64_t & sendNs) override
  {
    if (!source_ || publisher_ == nullptr)
      return SendOutcome::kError;
    std::unique_ptr<RosMessageT> message;
    const SendOutcome outcome = source_->Next(sequence, message, sendNs);
    if (outcome != SendOutcome::kSent)
      return outcome;
    try {
      publisher_->publish(std::move(message));
    } catch (const std::exception &) {
      return SendOutcome::kError;
    }
    return SendOutcome::kSent;
  }

  std::uint64_t PayloadBytes() const override { return source_ ? source_->PayloadBytes() : 0; }

private:
  void OnMessage(const RosMessageT & message)
  {
    const std::uint64_t receiveNs = MonotonicNowNs();
    std::uint64_t bytes = 0;
    const FrameReading reading = Sample::Check(message, streamTag_, bytes);
    if (recorder_.OnFrame(reading, receiveNs, bytes))
      onExpired_();
  }

  const std::uint32_t streamTag_;
  ProbeRecorder & recorder_;
  const std::function<void()> onExpired_;
  std::optional<typename Sample::Source> source_;
  typename rclcpp::Publisher<RosMessageT>::SharedPtr publisher_;
  typename rclcpp::Subscription<RosMessageT>::SharedPtr subscription_;
};

/** @brief The endpoint of the Contract `flow.contract` names. @throws std::invalid_argument */
std::unique_ptr<ProbeEndpoint> MakeEndpoint(rclcpp::Node & node,
    const ProbeParams & params,
    const ProbeFlow & flow,
    bool sender,
    ProbeRecorder & recorder,
    std::function<void()> onExpired)
{
#define QRB_CONTRACT_MAPPING(Id, TypeName, Payload, ContractT, DescriptorT, RosMessageT, ...)      \
  if (MatchesContract(flow.contract, TypeName))                                                    \
    return std::make_unique<TypedEndpoint<RosMessageT>>(                                           \
        node, params, flow, sender, recorder, std::move(onExpired));
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING

  std::string supported;
#define QRB_CONTRACT_MAPPING(Id, TypeName, ...)                                                    \
  supported += std::string(supported.empty() ? "" : ", ") + ShortContractName(TypeName);
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
  throw std::invalid_argument(params.adapterConfig + ": unsupported contract_type " +
                              flow.contract + "; supported: " + supported);
}

}  // namespace

// --- RosStreamProbeNode ----------------------------------------------------------

RosStreamProbeNode::RosStreamProbeNode(const rclcpp::NodeOptions & options)
  // The Bridge passes DMA-BUF frames zero-copy only to intra-process peers.
  : rclcpp::Node("qrb_ros_adapter_stream_probe",
        rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  try {
    LoadParams();
    sender_ = flow_.direction == ProbeDirection::kRosToHcm;
    recorder_ = std::make_unique<ProbeRecorder>(get_logger(), params_, flow_, sender_);
    endpoint_ =
        MakeEndpoint(*this, params_, flow_, sender_, *recorder_, [this]() { Finish("duration"); });
  } catch (const std::invalid_argument & error) {
    throw std::invalid_argument(error.what() + std::string(kHelpHint));
  }
  // How this run is set up; --help and `ros2 param describe` explain the parameters.
  RCLCPP_INFO(get_logger(),
      "the ROS side of a probed stream through the ROS IPC Bridge\n"
      "  params file     %s\n"
      "  adapter config  %s\n"
      "  flow            direction %d (%s, this side %s), %s on %s (stream tag 0x%08x)\n"
      "                  QoS %s, %s, depth %u\n"
      "  run             fps %g%s, payload %llu bytes, duration %g s%s, warmup %g s, log %s",
      get_parameter("params_file").as_string().c_str(), params_.adapterConfig.c_str(),
      static_cast<int>(flow_.direction),
      flow_.direction == ProbeDirection::kHcmToRos ? "HCM to ROS" : "ROS to HCM",
      sender_ ? "sends" : "receives", ShortContractName(flow_.contract).c_str(),
      flow_.topic.c_str(), StreamTagOf(flow_.topic), flow_.reliability.c_str(),
      flow_.durability.c_str(), flow_.depth, params_.fps, params_.fps == 0.0 ? " (unlimited)" : "",
      static_cast<unsigned long long>(params_.payloadBytes), params_.duration,
      params_.duration == 0.0 ? " (until unloaded or a signal)" : "", params_.warmup,
      params_.log.empty() ? "none" : params_.log.c_str());

  tickTimer_ = create_wall_timer(std::chrono::seconds(1), [this]() { Tick(); });
  if (sender_)
    senderThread_ = std::thread([this]() { SendLoop(); });
}

RosStreamProbeNode::~RosStreamProbeNode()
{
  stopping_ = true;
  wake_.notify_all();
  if (senderThread_.joinable())
    senderThread_.join();
  Finish("stopped");
  tickTimer_.reset();
  endpoint_.reset();
}

void RosStreamProbeNode::Finish(const std::string & reason)
{
  if (recorder_ == nullptr || !recorder_->Finish(reason))
    return;
  {
    // Pairs with the predicate check of the sending thread.
    std::lock_guard<std::mutex> lock(wakeMutex_);
  }
  wake_.notify_all();
}

bool RosStreamProbeNode::Done() const
{
  return recorder_ == nullptr || recorder_->Done();
}

int RosStreamProbeNode::ExitCode() const
{
  return recorder_ != nullptr ? recorder_->ExitCode() : 0;
}

void RosStreamProbeNode::LoadParams()
{
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();

  std::string configDir;
  try {
    configDir = ament_index_cpp::get_package_share_directory("qrb_ros_adapter_stream_probe") +
                "/probe_configs";
  } catch (const std::exception & error) {
    throw std::runtime_error(std::string("cannot find the default configs: ") + error.what());
  }

  std::string paramsFile = configDir + "/stream_probe.json";
  const auto file = overrides.find("params_file");
  if (file != overrides.end()) {
    if (file->second.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
      throw std::invalid_argument("ROS parameter 'params_file' must be a string");
    paramsFile = file->second.get<std::string>();
  }

  ProbeParams params = DefaultParams(configDir);
  ApplyJsonFile(params, paramsFile);

  nlohmann::json fromRos = nlohmann::json::object();
  for (const auto & [name, value] : overrides) {
    // Parameters of rclcpp itself, already declared, and QoS overrides are not ours.
    if (name == "params_file" || has_parameter(name) || name.rfind("qos_overrides.", 0) == 0)
      continue;
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        fromRos[name] = value.get<bool>();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        fromRos[name] = value.get<std::int64_t>();
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        fromRos[name] = value.get<double>();
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        fromRos[name] = value.get<std::string>();
        break;
      default:
        throw std::invalid_argument("ROS parameter '" + name + "' has an unsupported type " +
                                    rclcpp::to_string(value.get_type()));
    }
  }
  ApplyJson(params, fromRos, "ROS parameters");
  Validate(params);

  // Nothing checks the config for this side but the probe, so any problem is fatal.
  std::string adapterConfig;
  if (!ReadTextFile(params.adapterConfig, adapterConfig))
    throw std::invalid_argument("cannot read 'adapter_config' " + params.adapterConfig);
  std::vector<std::string> problems;
  const ProbeFlow flow = ReadProbeFlow(adapterConfig, ProbeFlow{}, problems);
  if (!problems.empty()) {
    std::string message = params.adapterConfig + ":";
    for (const std::string & problem : problems)
      message += " " + problem + ";";
    message.pop_back();
    throw std::invalid_argument(message);
  }

  params_ = params;
  flow_ = flow;
  DeclareEffectiveParams(paramsFile);
}

void RosStreamProbeNode::DeclareEffectiveParams(const std::string & paramsFile)
{
  // Visible with `ros2 param`, but changing them has no effect: read-only.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.dynamic_typing = true;
  descriptor.description = DescribeParam("params_file");
  declare_parameter("params_file", rclcpp::ParameterValue(paramsFile), descriptor);
  // A named object: items() refers to it, and a temporary would end before the loop.
  const nlohmann::json effective = ToJson(params_);
  for (const auto & [key, value] : effective.items()) {
    rclcpp::ParameterValue parameter;
    if (value.is_string())
      parameter = rclcpp::ParameterValue(value.get<std::string>());
    else if (value.is_number_integer())
      parameter = rclcpp::ParameterValue(value.get<std::int64_t>());
    else
      parameter = rclcpp::ParameterValue(value.get<double>());
    descriptor.description = DescribeParam(key);
    declare_parameter(key, parameter, descriptor);
  }
}

void RosStreamProbeNode::SendLoop()
{
  const auto stopRequested = [this]() { return stopping_.load() || recorder_->Done(); };
  {
    std::unique_lock<std::mutex> lock(wakeMutex_);
    while (!stopRequested() && endpoint_->PeerCount() == 0)
      wake_.wait_for(lock, std::chrono::milliseconds(10), stopRequested);
  }
  if (stopRequested())
    return;
  // `duration` counts from here: frames sent before a match would be lost.
  recorder_->Begin(FormatText("matched: %zu peer(s)", endpoint_->PeerCount()));

  using Clock = std::chrono::steady_clock;
  const Clock::duration period = params_.fps > 0.0 ?
                                     std::chrono::duration_cast<Clock::duration>(
                                         std::chrono::duration<double>(1.0 / params_.fps)) :
                                     Clock::duration::zero();
  const std::uint64_t payloadBytes = endpoint_->PayloadBytes();
  Clock::time_point next = Clock::now();
  std::uint64_t sequence = 0;
  while (!stopRequested()) {
    if (recorder_->Expired(MonotonicNowNs())) {
      Finish("duration");
      return;
    }
    std::uint64_t sendNs = 0;
    const SendOutcome outcome = endpoint_->Send(sequence, sendNs);
    recorder_->OnSend(outcome, payloadBytes, sequence, sendNs);
    // A frame that was not sent keeps its sequence, so the receiver sees no gap.
    if (outcome == SendOutcome::kSent)
      ++sequence;

    std::unique_lock<std::mutex> lock(wakeMutex_);
    if (period > Clock::duration::zero()) {
      next += period;
      // Behind schedule (a slow frame): restart from now rather than burst.
      const Clock::time_point now = Clock::now();
      if (next < now)
        next = now;
      wake_.wait_until(lock, next, stopRequested);
    } else if (outcome != SendOutcome::kSent) {
      wake_.wait_for(lock, std::chrono::microseconds(100), stopRequested);
    }
  }
}

void RosStreamProbeNode::Tick()
{
  if (recorder_->Expired(MonotonicNowNs()))
    Finish("duration");
  else
    recorder_->Tick(endpoint_->PeerCount());
}

}  // namespace qrb::adapter::stream_probe

RCLCPP_COMPONENTS_REGISTER_NODE(qrb::adapter::stream_probe::RosStreamProbeNode)
