// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "hcm_probe_node.hpp"

#define QRB_CONTRACT_MAPPING_INCLUDE_HCM_HEADERS
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING_INCLUDE_HCM_HEADERS

#include <algorithm>
#include <array>
#include <atomic>
#include <climits>
#include <cstring>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QC/Node/Ifs/QCNodeConfigIfs.hpp"
#include "QC/Node/NodeBase.hpp"

namespace qrb::adapter::stream_probe
{

// --- The probe in each Contract's descriptor --------------------------------------
//
// A new Contract needs an HcmProbeSample<> for its descriptor type; without it
// this package does not compile. The layout is the one RosProbeSample<> uses,
// so either side checks what the other sends.

/** @brief What a sender keeps between frames. */
struct HcmSenderState
{
  std::uint32_t streamTag{0};
  std::uint64_t payloadBytes{0};
  /**
   * The buffers that already hold the pattern, so it is written once per
   * buffer of a host that cycles through a few.
   */
  std::array<const void *, 8> patterned{};
  std::size_t nextPatterned{0};
  std::size_t patternedBytes{0};
  /** The send time Stamp() put in the latest frame. */
  std::uint64_t lastSendNs{0};
};

/** @brief What one sent frame became. */
enum class HcmStampResult
{
  kOk,
  kBadBuffer,  // the frame's buffer cannot hold the payload
  kSyncFailed,
};

/** @brief CPU access to the buffer of `descriptor`; a buffer without a DMA-BUF needs none. */
inline bool SyncBuffer(const QC::Memory::BufferDescriptor_t & descriptor, std::uint64_t flags)
{
  if (descriptor.allocatorType != QC::Memory::QC_MEMORY_ALLOCATOR_DMA)
    return true;
  if (descriptor.dmaHandle > static_cast<std::uint64_t>(INT_MAX))
    return false;
  dma_buf_sync sync{flags};
  int result = 0;
  do {
    result = ::ioctl(static_cast<int>(descriptor.dmaHandle), DMA_BUF_IOCTL_SYNC, &sync);
  } while (result != 0 && (errno == EINTR || errno == EAGAIN));
  // Heaps of uncached memory do not implement the ioctl.
  return result == 0 || errno == ENOTTY;
}

template <typename>
inline constexpr bool kAlwaysFalseHcm = false;

/**
 * @brief The probe in descriptor `DescriptorT`. Specialize it for the
 * descriptor of every QRB_CONTRACT_MAPPING entry, with:
 *
 *   // Empty if `payloadBytes` suits the Contract, else why not.
 *   static std::string CheckPayloadBytes( std::uint64_t payloadBytes );
 *   // The payload bytes of one sent frame.
 *   static std::uint64_t SentBytes( const HcmSenderState & );
 *   static HcmStampResult Stamp( DescriptorT &, HcmSenderState &, std::uint64_t sequence );
 *   static FrameReading Check( const DescriptorT &, std::uint32_t streamTag,
 *       std::uint64_t &payloadBytes );
 *
 * Stamp() takes the send time itself, as late as it can, and keeps it in
 * `HcmSenderState::lastSendNs`.
 */
template <typename DescriptorT>
struct HcmProbeSample
{
  static_assert(kAlwaysFalseHcm<DescriptorT>,
      "add an HcmProbeSample<> for the new Contract in test/hcm/hcm_probe_node.cpp");
};

// --- geometry.v1.PoseWithCovarianceStamped (inline) ---------------------------

/** The probe fills the covariance; the pose is the identity. */
template <>
struct HcmProbeSample<adapter::PoseHcmDescriptor>
{
  using Descriptor = adapter::PoseHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";

  static std::string CheckPayloadBytes(std::uint64_t) { return {}; }

  static std::uint64_t SentBytes(const HcmSenderState &)
  {
    return sizeof(std::declval<const Descriptor &>().poseWithCovariance.covariance);
  }

  static HcmStampResult Stamp(Descriptor & pose, HcmSenderState & state, std::uint64_t sequence)
  {
    if (pose.frameId != kFrameId)
      (void)pose.frameId.Assign(kFrameId);  // fits FrameId_t
    pose.poseWithCovariance.pose.position.x = 0.0;
    pose.poseWithCovariance.pose.position.y = 0.0;
    pose.poseWithCovariance.pose.position.z = 0.0;
    pose.poseWithCovariance.pose.orientation.x = 0.0;
    pose.poseWithCovariance.pose.orientation.y = 0.0;
    pose.poseWithCovariance.pose.orientation.z = 0.0;
    pose.poseWithCovariance.pose.orientation.w = 1.0;
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    pose.timestampNs = now;
    StampDoubles(pose.poseWithCovariance.covariance.data(),
        pose.poseWithCovariance.covariance.size(), state.streamTag, sequence, now);
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & pose,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(pose.poseWithCovariance.covariance);
    FrameReading reading = CheckDoubles(pose.poseWithCovariance.covariance.data(),
        pose.poseWithCovariance.covariance.size(), streamTag);
    if (reading.check == FrameCheck::kOk &&
        (pose.frameId != kFrameId || pose.poseWithCovariance.pose.orientation.w != 1.0))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- nav.v1.Odometry (inline) -------------------------------------------------

/**
 * The probe fills the pose covariance; the pose is the identity,
 * the twist is zero and its covariance carries a fixed marker, so a sample that
 * mixes up pose and twist is caught as a bad layout.
 */
template <>
struct HcmProbeSample<adapter::OdometryHcmDescriptor>
{
  using Descriptor = adapter::OdometryHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr const char * kChildFrameId = "stream_probe_base";
  static constexpr double kTwistCovarianceMarker = 0.5;

  static std::string CheckPayloadBytes(std::uint64_t) { return {}; }

  static std::uint64_t SentBytes(const HcmSenderState &)
  {
    return sizeof(std::declval<const Descriptor &>().poseWithCovariance.covariance);
  }

  static HcmStampResult Stamp(Descriptor & odom, HcmSenderState & state, std::uint64_t sequence)
  {
    if (odom.frameId != kFrameId)
      (void)odom.frameId.Assign(kFrameId);  // fits FrameId_t
    if (odom.childFrameId != kChildFrameId)
      (void)odom.childFrameId.Assign(kChildFrameId);       // fits FrameId_t
    odom.poseWithCovariance.pose = QRB::Memory::Pose_t{};  // identity
    odom.twistWithCovariance.twist = QRB::Memory::Twist_t{};
    odom.twistWithCovariance.covariance.fill(kTwistCovarianceMarker);
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    odom.timestampNs = now;
    StampDoubles(odom.poseWithCovariance.covariance.data(),
        odom.poseWithCovariance.covariance.size(), state.streamTag, sequence, now);
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & odom,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(odom.poseWithCovariance.covariance);
    FrameReading reading = CheckDoubles(odom.poseWithCovariance.covariance.data(),
        odom.poseWithCovariance.covariance.size(), streamTag);
    if (reading.check == FrameCheck::kOk &&
        (odom.frameId != kFrameId || odom.childFrameId != kChildFrameId ||
            odom.poseWithCovariance.pose.orientation.w != 1.0 ||
            odom.twistWithCovariance.covariance[0] != kTwistCovarianceMarker ||
            odom.twistWithCovariance.covariance[35] != kTwistCovarianceMarker))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- sensor.v1.PointCloud2 (DMA-BUF) ------------------------------------------

/** One row of `payload_bytes / 16` points x, y, z, intensity (FLOAT32). */
template <>
struct HcmProbeSample<adapter::PointCloudHcmDescriptor>
{
  using Descriptor = adapter::PointCloudHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr std::uint32_t kPointStep = 16;
  static constexpr const char * kFieldNames[] = {"x", "y", "z", "intensity"};
  static constexpr std::size_t kFieldCount = sizeof(kFieldNames) / sizeof(kFieldNames[0]);

  static std::string CheckPayloadBytes(std::uint64_t payloadBytes)
  {
    if (payloadBytes >= kMinPayloadBytes && payloadBytes % kPointStep == 0)
      return {};
    return "'payload_bytes' of PointCloud2 must be a multiple of " + std::to_string(kPointStep) +
           " and at least " + std::to_string(kMinPayloadBytes);
  }

  static std::uint64_t SentBytes(const HcmSenderState & state) { return state.payloadBytes; }

  static HcmStampResult Stamp(Descriptor & cloud, HcmSenderState & state, std::uint64_t sequence)
  {
    const std::size_t bytes = static_cast<std::size_t>(state.payloadBytes);
    if (cloud.pBuf == nullptr || cloud.offset > cloud.size || cloud.size - cloud.offset < bytes)
      return HcmStampResult::kBadBuffer;
    auto * data = static_cast<std::uint8_t *>(cloud.GetDataPtr());

    if (!SyncBuffer(cloud, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE))
      return HcmStampResult::kSyncFailed;
    if (state.patternedBytes != bytes) {
      state.patterned = {};
      state.patternedBytes = bytes;
    }
    if (std::find(state.patterned.begin(), state.patterned.end(), data) == state.patterned.end()) {
      FillPattern(data, bytes, state.streamTag);
      state.patterned[state.nextPatterned] = data;
      state.nextPatterned = (state.nextPatterned + 1) % state.patterned.size();
    }
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    StampFrame(data, bytes, state.streamTag, sequence, now);
    if (!SyncBuffer(cloud, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE))
      return HcmStampResult::kSyncFailed;

    cloud.validSize = bytes;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(bytes / kPointStep);
    if (!HasSenderFields(cloud)) {
      cloud.numFields = static_cast<std::uint32_t>(kFieldCount);
      for (std::size_t i = 0; i < kFieldCount; ++i)
        cloud.fields[i] = QRB::Memory::PointField_t(kFieldNames[i],
            static_cast<std::uint32_t>(i * 4), QRB::Memory::POINT_FIELD_TYPE_FLOAT32, 1);
    }
    cloud.isBigEndian = false;
    cloud.pointStep = kPointStep;
    cloud.rowStep = static_cast<std::uint32_t>(bytes);
    cloud.isDense = true;
    cloud.timestampNs = now;
    if (cloud.frameId != kFrameId)
      (void)cloud.frameId.Assign(kFrameId);  // fits FrameId_t
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & cloud,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = cloud.validSize;
    FrameReading reading;
    if (cloud.pBuf == nullptr || cloud.offset > cloud.size ||
        cloud.validSize > cloud.size - cloud.offset)
      return reading;

    if (!SyncBuffer(cloud, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ))
      return reading;
    reading = CheckFrame(static_cast<const std::uint8_t *>(cloud.GetDataPtr()),
        static_cast<std::size_t>(cloud.validSize), streamTag);
    const bool ended = SyncBuffer(cloud, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
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
  static bool HasSenderFields(const Descriptor & cloud)
  {
    if (cloud.numFields != kFieldCount)
      return false;
    for (std::size_t i = 0; i < kFieldCount; ++i) {
      const QRB::Memory::PointField_t & field = cloud.fields[i];
      if (field.name != kFieldNames[i] || field.offset != i * 4 ||
          field.datatype != QRB::Memory::POINT_FIELD_TYPE_FLOAT32 || field.count != 1)
        return false;
    }
    return true;
  }

  static bool HasSenderLayout(const Descriptor & cloud)
  {
    return cloud.frameId == kFrameId && cloud.height == 1 && cloud.pointStep == kPointStep &&
           cloud.rowStep == std::uint64_t{cloud.width} * kPointStep &&
           cloud.rowStep == cloud.validSize && !cloud.isBigEndian && cloud.isDense &&
           HasSenderFields(cloud);
  }
};

namespace
{

constexpr std::uint64_t kNsPerSecond = 1000000000ull;
constexpr std::uint32_t kMonitoringEntries = 7;

// --- Contracts -------------------------------------------------------------------

/** @brief The probe of one Contract, whatever its descriptor type. */
struct ContractOps
{
  const char * typeName;
  std::string (*checkPayloadBytes)(std::uint64_t);
  std::uint64_t (*sentBytes)(const HcmSenderState &);
  /** False: `buffer` is not the Contract's descriptor. */
  bool (*stamp)(QC::Memory::QCBufferDescriptorBase_t & buffer,
      HcmSenderState & state,
      std::uint64_t sequence,
      HcmStampResult & result);
  bool (*check)(const QC::Memory::QCBufferDescriptorBase_t & buffer,
      std::uint32_t streamTag,
      FrameReading & reading,
      std::uint64_t & payloadBytes);
};

template <typename DescriptorT>
bool StampAs(QC::Memory::QCBufferDescriptorBase_t & buffer,
    HcmSenderState & state,
    std::uint64_t sequence,
    HcmStampResult & result)
{
  auto * descriptor = dynamic_cast<DescriptorT *>(&buffer);
  if (descriptor == nullptr)
    return false;
  result = HcmProbeSample<DescriptorT>::Stamp(*descriptor, state, sequence);
  return true;
}

template <typename DescriptorT>
bool CheckAs(const QC::Memory::QCBufferDescriptorBase_t & buffer,
    std::uint32_t streamTag,
    FrameReading & reading,
    std::uint64_t & payloadBytes)
{
  const auto * descriptor = dynamic_cast<const DescriptorT *>(&buffer);
  if (descriptor == nullptr)
    return false;
  reading = HcmProbeSample<DescriptorT>::Check(*descriptor, streamTag, payloadBytes);
  return true;
}

const ContractOps kContracts[] = {
#define QRB_CONTRACT_MAPPING(Id, TypeName, Payload, ContractT, DescriptorT, ...)                   \
  {TypeName, &HcmProbeSample<DescriptorT>::CheckPayloadBytes,                                      \
      &HcmProbeSample<DescriptorT>::SentBytes, &StampAs<DescriptorT>, &CheckAs<DescriptorT>},
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
};

const ContractOps * FindContract(const std::string & contract)
{
  for (const ContractOps & ops : kContracts) {
    if (MatchesContract(contract, ops.typeName))
      return &ops;
  }
  return nullptr;
}

std::string SupportedContracts()
{
  std::string supported;
  for (const ContractOps & ops : kContracts)
    supported += (supported.empty() ? "" : ", ") + ShortContractName(ops.typeName);
  return supported;
}

// --- Static configuration ----------------------------------------------------------

struct ProbeNodeSettings
{
  std::string name;
  std::uint32_t id{0};
  bool sender{false};
  const ContractOps * contract{nullptr};
  std::string topic{ProbeFlow{}.topic};
  std::uint64_t payloadBytes{ProbeParams{}.payloadBytes};
  double warmup{0.0};
  std::uint32_t globalBufferId{0};
};

// The host writes this config, so only the required fields and the contract
// are checked; a value of the wrong type fails the whole config.
QC::QCStatus_e ParseSettings(const std::string & json,
    ProbeNodeSettings & out,
    std::string & errors)
{
  ProbeNodeSettings settings;
  std::string contract;
  try {
    const nlohmann::json node = nlohmann::json::parse(json).at("static");
    settings.name = node.at("name").get<std::string>();
    const auto direction = node.at("direction").get<std::uint32_t>();
    contract = node.at("contract").get<std::string>();
    if (settings.name.empty() || direction > 1) {
      errors += "\"name\" must be non-empty and \"direction\" 0 or 1; ";
      return QC::QC_STATUS_BAD_ARGUMENTS;
    }
    settings.sender = static_cast<ProbeDirection>(direction) == ProbeDirection::kHcmToRos;
    settings.id = node.value("id", settings.id);
    settings.topic = node.value("topic", settings.topic);
    settings.payloadBytes = node.value("payload_bytes", settings.payloadBytes);
    settings.warmup = node.value("warmup", settings.warmup);
    if (node.contains("globalBufferIdMap"))
      settings.globalBufferId = node["globalBufferIdMap"].at(0).at("id").get<std::uint32_t>();
  } catch (const nlohmann::json::exception & e) {
    errors += std::string(e.what()) + "; ";
    return QC::QC_STATUS_BAD_ARGUMENTS;
  }

  settings.contract = FindContract(contract);
  if (settings.contract == nullptr) {
    errors +=
        "unsupported \"contract\" " + contract + "; supported: " + SupportedContracts() + "; ";
    return QC::QC_STATUS_UNSUPPORTED;
  }
  if (settings.sender) {
    const std::string why = settings.contract->checkPayloadBytes(settings.payloadBytes);
    if (!why.empty()) {
      errors += why + "; ";
      return QC::QC_STATUS_BAD_ARGUMENTS;
    }
  }
  out = std::move(settings);
  return QC::QC_STATUS_OK;
}

struct HcmStreamProbeConfigData : public QC::QCNodeConfigBase_t
{
  ProbeNodeSettings settings;
};

class HcmStreamProbeNodeConfig final : public QC::QCNodeConfigIfs
{
public:
  QC::QCStatus_e VerifyAndSet(const std::string config, std::string & errors) override
  {
    ProbeNodeSettings parsed;
    const QC::QCStatus_e status = ParseSettings(config, parsed, errors);
    if (status != QC::QC_STATUS_OK)
      return status;
    config_.settings = std::move(parsed);
    config_.nodeId.name = config_.settings.name;
    config_.nodeId.id = static_cast<std::uint8_t>(config_.settings.id);
    config_.nodeId.type = kHcmStreamProbeNodeType;
    config_.numOfEntries = 1;
    return QC::QC_STATUS_OK;
  }

  // Both GetOptions() forms, as in the Adapter, for either QCNode SDK.
  const std::string & GetOptions()
  {
    static const std::string options =
        R"({"required":["name","direction","contract"],)"
        R"("optional":["id","topic","payload_bytes","warmup","globalBufferIdMap"]})";
    return options;
  }
  QC::QCStatus_e GetOptions(std::string & options)
  {
    options = GetOptions();
    return QC::QC_STATUS_OK;
  }

  const QC::QCNodeConfigBase_t & Get() override { return config_; }

  const ProbeNodeSettings & Settings() const { return config_.settings; }
  void Clear() { config_ = {}; }

private:
  HcmStreamProbeConfigData config_{};
};

// --- Monitoring --------------------------------------------------------------------

class HcmStreamProbeNodeMonitoring final : public QC::QCNodeMonitoringIfs
{
public:
  HcmStreamProbeNodeMonitoring() { Reset(false, 0.0); }

  QC::QCStatus_e VerifyAndSet(const std::string /*config*/, std::string & /*errors*/) override
  {
    return QC::QC_STATUS_OK;
  }
  const std::string & GetOptions()
  {
    static const std::string options = R"({"format":"HcmStreamProbeMonitoringData"})";
    return options;
  }
  QC::QCStatus_e GetOptions(std::string & options)
  {
    options = GetOptions();
    return QC::QC_STATUS_OK;
  }

  const QC::QCNodeMonitoringBase_t & Get() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    SnapshotLocked(snapshot_);
    return snapshot_;
  }
  std::uint32_t GetMaximalSize() override { return sizeof(HcmStreamProbeMonitoringData); }
  std::uint32_t GetCurrentSize() override { return sizeof(HcmStreamProbeMonitoringData); }

  QC::QCStatus_e Place(void * data, std::uint32_t & size) override
  {
    if (data == nullptr)
      return QC::QC_STATUS_BAD_ARGUMENTS;
    if (size < sizeof(HcmStreamProbeMonitoringData)) {
      size = sizeof(HcmStreamProbeMonitoringData);
      return QC::QC_STATUS_OUT_OF_BOUND;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SnapshotLocked(snapshot_);
    std::memcpy(data, &snapshot_, sizeof(snapshot_));
    size = sizeof(snapshot_);
    return QC::QC_STATUS_OK;
  }

  void Reset(bool sender, double warmupSeconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sender_ = sender;
    warmupNs_ = static_cast<std::uint64_t>(warmupSeconds * static_cast<double>(kNsPerSecond));
    firstFrameNs_ = 0;
    processErrors_ = 0;
    sent_ = {};
    received_ = ReceiverStats();
    lastProblem_ = {};
    lastFrame_ = FrameRecord{};
  }

  void RecordSent(std::uint64_t sequence, std::uint64_t sendNs, std::uint64_t bytes)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (firstFrameNs_ == 0)
      firstFrameNs_ = sendNs;
    ++sent_.sent;
    sent_.bytes += bytes;
    lastFrame_ = FrameRecord{};
    lastFrame_.count = ++frames_;
    lastFrame_.sequence = sequence;
    lastFrame_.sendNs = sendNs;
    lastFrame_.bytes = bytes;
  }

  void RecordSendError()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++sent_.sendErrors;
    ++processErrors_;
  }

  void RecordProcessError()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++processErrors_;
  }

  void RecordReceived(const FrameReading & reading, std::uint64_t receiveNs, std::uint64_t bytes)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (firstFrameNs_ == 0)
      firstFrameNs_ = receiveNs;
    lastFrame_ =
        received_.Record(reading, receiveNs, bytes, receiveNs - firstFrameNs_ >= warmupNs_);
    lastFrame_.count = ++frames_;
    if (reading.check != FrameCheck::kOk)
      lastProblem_ = reading;
  }

private:
  void SnapshotLocked(HcmStreamProbeMonitoringData & out) const
  {
    out.numOfEntries = kMonitoringEntries;
    out.sender = sender_ ? 1u : 0u;
    out.processErrors = processErrors_;
    out.sent = sent_;
    out.received = received_.Counters();
    out.lastProblem = lastProblem_;
    out.latency = received_.Latency();
    out.lastFrame = lastFrame_;
  }

  std::mutex mutex_;
  bool sender_{false};
  std::uint64_t warmupNs_{0};
  std::uint64_t firstFrameNs_{0};
  std::uint64_t processErrors_{0};
  SenderCounters sent_;
  ReceiverStats received_;
  FrameReading lastProblem_;
  /** Not reset by Initialize(), so lastFrame.count never repeats. */
  std::uint64_t frames_{0};
  FrameRecord lastFrame_;
  HcmStreamProbeMonitoringData snapshot_{};
};

// --- Node ----------------------------------------------------------------------------

class HcmStreamProbeNode final : public QC::QCNodeIfs
{
public:
  QC::QCStatus_e Initialize(QC::QCNodeInit_t & config) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_INITIAL)
      return QC::QC_STATUS_BAD_STATE;

    state_.store(QC::QC_OBJECT_STATE_INITIALIZING);
    std::string errors;
    const QC::QCStatus_e status = config_.VerifyAndSet(config.config, errors);
    if (status != QC::QC_STATUS_OK) {
      config_.Clear();
      state_.store(QC::QC_OBJECT_STATE_INITIAL);
      return status;
    }
    const ProbeNodeSettings & settings = config_.Settings();
    monitoring_.Reset(settings.sender, settings.warmup);
    // The sequence goes on across Initialize(): a receiver that keeps
    // running sees one stream, not a restart it would count as reordered.
    senderState_ = HcmSenderState{};
    senderState_.streamTag = StreamTagOf(settings.topic);
    senderState_.payloadBytes = settings.payloadBytes;
    state_.store(QC::QC_OBJECT_STATE_READY);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e Start() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_READY)
      return QC::QC_STATUS_BAD_STATE;
    state_.store(QC::QC_OBJECT_STATE_RUNNING);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e ProcessFrameDescriptor(QC::QCFrameDescriptorNodeIfs & frameDesc) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_RUNNING)
      return QC::QC_STATUS_BAD_STATE;

    const ProbeNodeSettings & settings = config_.Settings();
    QC::Memory::QCBufferDescriptorBase_t & buffer = frameDesc.GetBuffer(settings.globalBufferId);
    if (settings.sender) {
      HcmStampResult result = HcmStampResult::kOk;
      if (!settings.contract->stamp(buffer, senderState_, sequence_, result)) {
        monitoring_.RecordProcessError();
        return QC::QC_STATUS_BAD_ARGUMENTS;
      }
      if (result != HcmStampResult::kOk) {
        monitoring_.RecordSendError();
        return result == HcmStampResult::kBadBuffer ? QC::QC_STATUS_INVALID_BUF :
                                                      QC::QC_STATUS_FAIL;
      }
      monitoring_.RecordSent(
          sequence_, senderState_.lastSendNs, settings.contract->sentBytes(senderState_));
      ++sequence_;
      return QC::QC_STATUS_OK;
    }

    const std::uint64_t receiveNs = MonotonicNowNs();
    FrameReading reading;
    std::uint64_t bytes = 0;
    if (!settings.contract->check(buffer, senderState_.streamTag, reading, bytes)) {
      monitoring_.RecordProcessError();
      return QC::QC_STATUS_BAD_ARGUMENTS;
    }
    // A wrong frame is a finding of the probe, not a failure of the call.
    monitoring_.RecordReceived(reading, receiveNs, bytes);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e Stop() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_RUNNING)
      return QC::QC_STATUS_BAD_STATE;
    state_.store(QC::QC_OBJECT_STATE_READY);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e DeInitialize() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_READY)
      return QC::QC_STATUS_BAD_STATE;
    state_.store(QC::QC_OBJECT_STATE_INITIAL);
    return QC::QC_STATUS_OK;
  }

  QC::QCObjectState_e GetState() override { return state_.load(); }
  QC::QCNodeConfigIfs & GetConfigurationIfs() override { return config_; }
  QC::QCNodeMonitoringIfs & GetMonitoringIfs() override { return monitoring_; }

private:
  std::mutex mutex_;
  std::atomic<QC::QCObjectState_e> state_{QC::QC_OBJECT_STATE_INITIAL};
  HcmStreamProbeNodeConfig config_;
  HcmStreamProbeNodeMonitoring monitoring_;
  HcmSenderState senderState_;
  std::uint64_t sequence_{0};
};

// Register the probe with QCNodeFactory at static-initialization time.
const QC::Node::NodeRegistrar<HcmStreamProbeNode> g_registerStreamProbe{kHcmStreamProbeNodeType};

}  // namespace

}  // namespace qrb::adapter::stream_probe
