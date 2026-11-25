// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

//
// SampleStreamProbe: the HCM side of the end-to-end test, run with
// QCNodeSampleApp (-t StreamProbe) in a pipeline with SampleIpcAdapter. It
// hosts HcmStreamProbeNode and passes frames to and from the Adapter sample
// over the DataBroker:
//
//   HCM to ROS: StreamProbe --output_topic--> IpcAdapter (input_topic)
//   ROS to HCM: IpcAdapter (output_topic) --input_topic--> StreamProbe
//
// The Flow comes from the Adapter's static config, so both samples are given
// the same adapter_config file. Stop() prints a summary and fails on a
// process error, a lost or wrong frame, or no frame received.
//
// Attributes:
//   adapter_config: (required) the Adapter's static config file
//   output_topic  : (required for HCM to ROS) the topic to publish stamped frames on
//   input_topic   : (required for ROS to HCM) the topic of the frames to check
//   fps           : (optional) frames per second a sender sends, 0 for no pause, default 30
//   payload_bytes : (optional) a sender's payload in the buffer, for a DmaBuf Contract
//                   (PointCloud2); others ignore it, default 1048576
//   pool_size     : (optional) frames a sender cycles through, 1..32, default 4
//   warmup        : (optional) seconds a receiver does not measure, default 0
//

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ContractFrame.hpp"
#include "QC/Node/Ifs/QCNodeFactory.hpp"
#include "QC/Node/NodeFrameDescriptor.hpp"
#include "QC/sample/DataBroker.hpp"
#include "QC/sample/SampleIF.hpp"
#include "hcm_probe_node.hpp"

namespace QC
{
namespace sample
{

namespace sp = qrb::adapter::stream_probe;

class SampleStreamProbe : public SampleIF
{
public:
  QCStatus_e Init(std::string name, SampleConfig_t & config) override;
  QCStatus_e Start() override;
  QCStatus_e Stop() override;
  QCStatus_e Deinit() override;

private:
  QCStatus_e Process(ContractFrame & frame);
  bool GetMonitor(sp::HcmStreamProbeMonitoringData & monitor);
  void Send();
  void Receive();

  std::unique_ptr<QCNodeIfs> node_;
  sp::ProbeFlow flow_;
  bool sender_{true};
  std::uint32_t fps_{30};
  std::string inputTopic_;
  std::string outputTopic_;

  ContractFramePool pool_;
  DataSubscriber<ContractFrameRef_t> sub_;
  DataPublisher<ContractFrameRef_t> pub_;

  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::uint64_t errors_{0};
  std::uint64_t poolExhausted_{0};

  static constexpr std::uint64_t kMaxReportedErrors = 10;
  static constexpr std::uint32_t kReceiveTimeoutMs = 100;
  static constexpr auto kPoolWait = std::chrono::microseconds(100);
};

QCStatus_e SampleStreamProbe::Init(std::string name, SampleConfig_t & config)
{
  QCStatus_e ret = SampleIF::Init(name, sp::kHcmStreamProbeNodeType);
  if (ret != QC_STATUS_OK)
    return ret;

  const std::string path = Get(config, "adapter_config", "");
  std::string text;
  if (path.empty() || !sp::ReadTextFile(path, text)) {
    QC_ERROR("adapter_config must name a readable file: '%s'", path.c_str());
    return QC_STATUS_BAD_ARGUMENTS;
  }
  std::vector<std::string> problems;
  flow_ = sp::ReadProbeFlow(text, sp::ProbeFlow{}, problems);
  if (!problems.empty()) {
    QC_ERROR("adapter_config %s: %s", path.c_str(), problems.front().c_str());
    return QC_STATUS_BAD_ARGUMENTS;
  }
  sender_ = flow_.direction == sp::ProbeDirection::kHcmToRos;

  fps_ = Get(config, "fps", static_cast<std::uint32_t>(30));
  const std::uint32_t payloadBytes =
      Get(config, "payload_bytes", static_cast<std::uint32_t>(1048576));
  const std::uint32_t poolSize = Get(config, "pool_size", static_cast<std::uint32_t>(4));
  const float warmup = Get(config, "warmup", 0.0f);
  inputTopic_ = Get(config, "input_topic", "");
  outputTopic_ = Get(config, "output_topic", "");
  if (sender_ ? (outputTopic_.empty() || !inputTopic_.empty()) :
                (inputTopic_.empty() || !outputTopic_.empty())) {
    QC_ERROR("direction %d takes only %s", static_cast<int>(flow_.direction),
        sender_ ? "output_topic" : "input_topic");
    return QC_STATUS_BAD_ARGUMENTS;
  }
  if (poolSize < 1 || poolSize > ContractFramePool::kMaxSize || warmup < 0.0f) {
    QC_ERROR("pool_size must be in [1, %u] and warmup not negative", ContractFramePool::kMaxSize);
    return QC_STATUS_BAD_ARGUMENTS;
  }

  if (sender_) {
    ret = pool_.Init(flow_.contract, payloadBytes, poolSize);
    if (ret != QC_STATUS_OK) {
      QC_ERROR("failed to create %u %s frames of %u bytes: %d", poolSize, flow_.contract.c_str(),
          payloadBytes, ret);
      return ret;
    }
  }

  ret = Node::QCNodeFactory::CreateNode(sp::kHcmStreamProbeNodeType, node_);
  if (ret != QC_STATUS_OK) {
    QC_ERROR("failed to create the stream probe node: %d", ret);
    return ret;
  }
  const nlohmann::json staticConfig = {
      {"static",
          {
              {"name", m_name},
              {"id", m_nodeId.id},
              {"direction", static_cast<int>(flow_.direction)},
              {"contract", flow_.contract},
              {"topic", flow_.topic},
              {"payload_bytes", payloadBytes},
              {"warmup", warmup},
              {"globalBufferIdMap", {{{"name", "Probe"}, {"id", flow_.globalBufferId}}}},
          }},
  };
  QCNodeInit_t nodeInit;
  nodeInit.config = staticConfig.dump();
  ret = node_->Initialize(nodeInit);
  if (ret != QC_STATUS_OK) {
    QC_ERROR(
        "failed to initialize the stream probe node with %s: %d", nodeInit.config.c_str(), ret);
    return ret;
  }

  if (sender_)
    return pub_.Init(name, outputTopic_);
  // FIFO and as deep as any pool, so that no frame of the publisher is dropped.
  return sub_.Init(name, inputTopic_, ContractFramePool::kMaxSize, false);
}

QCStatus_e SampleStreamProbe::Start()
{
  const QCStatus_e ret = node_->Start();
  if (ret != QC_STATUS_OK) {
    QC_ERROR("failed to start the stream probe node: %d", ret);
    return ret;
  }
  stop_ = false;
  thread_ = std::thread(sender_ ? &SampleStreamProbe::Send : &SampleStreamProbe::Receive, this);
  return ret;
}

bool SampleStreamProbe::GetMonitor(sp::HcmStreamProbeMonitoringData & monitor)
{
  std::uint32_t size = sizeof(monitor);
  return node_->GetMonitoringIfs().Place(&monitor, size) == QC_STATUS_OK;
}

QCStatus_e SampleStreamProbe::Process(ContractFrame & frame)
{
  Node::NodeFrameDescriptor frameDesc(flow_.globalBufferId + 1);
  (void)frameDesc.SetBuffer(flow_.globalBufferId, frame.Descriptor());
  const QCStatus_e ret = node_->ProcessFrameDescriptor(frameDesc);
  if (ret != QC_STATUS_OK && ++errors_ <= kMaxReportedErrors)
    QC_ERROR("ProcessFrameDescriptor failed: %d", ret);
  return ret;
}

void SampleStreamProbe::Send()
{
  const auto period = std::chrono::nanoseconds(fps_ == 0 ? 0 : 1000000000ull / fps_);
  auto next = std::chrono::steady_clock::now();
  while (!stop_) {
    ContractFrameRef_t frame = pool_.Take();
    if (frame == nullptr) {
      // Every frame is still held downstream: skip this one.
      ++poolExhausted_;
      if (fps_ == 0)
        std::this_thread::sleep_for(kPoolWait);
    } else if (Process(*frame) == QC_STATUS_OK) {
      (void)pub_.Publish(frame);
    }
    frame.reset();

    const auto now = std::chrono::steady_clock::now();
    next += period;
    if (next > now)
      std::this_thread::sleep_until(next);
    else
      next = now;
  }
}

void SampleStreamProbe::Receive()
{
  while (!stop_) {
    ContractFrameRef_t frame;
    if (sub_.Receive(frame, kReceiveTimeoutMs) == QC_STATUS_OK) {
      (void)Process(*frame);
      // Hand the frame back to its pool.
      frame.reset();
    }
  }
}

QCStatus_e SampleStreamProbe::Stop()
{
  stop_ = true;
  if (thread_.joinable())
    thread_.join();

  QCStatus_e ret = node_->Stop();
  if (ret != QC_STATUS_OK) {
    QC_ERROR("failed to stop the stream probe node: %d", ret);
    return ret;
  }

  sp::HcmStreamProbeMonitoringData monitor{};
  if (!GetMonitor(monitor))
    return QC_STATUS_FAIL;
  printf("%s %s\n", m_name.c_str(), sp::FlowText(flow_, true).c_str());
  bool ok = errors_ == 0;
  if (sender_) {
    sp::SenderCounters sent = monitor.sent;
    sent.poolExhausted = poolExhausted_;
    printf("%s %s\n", m_name.c_str(), sp::SenderText(sent).c_str());
  } else {
    const sp::ReceiverCounters & received = monitor.received;
    printf("%s %s\n%s %s\n", m_name.c_str(), sp::ReceiverText(received).c_str(), m_name.c_str(),
        sp::LatencySummaryText(monitor.latency).c_str());
    ok = ok && received.received != 0 && received.lost == 0 && received.verifyErrors == 0 &&
         received.foreignStream == 0;
  }
  printf("%s %s\n", m_name.c_str(), ok ? "PASS" : "FAIL");
  return ok ? QC_STATUS_OK : QC_STATUS_FAIL;
}

QCStatus_e SampleStreamProbe::Deinit()
{
  QCStatus_e ret = QC_STATUS_OK;
  if (node_ != nullptr) {
    ret = node_->DeInitialize();
    node_.reset();
  }
  pool_.Clear();
  return ret;
}

REGISTER_SAMPLE(StreamProbe, SampleStreamProbe);

}  // namespace sample
}  // namespace QC
