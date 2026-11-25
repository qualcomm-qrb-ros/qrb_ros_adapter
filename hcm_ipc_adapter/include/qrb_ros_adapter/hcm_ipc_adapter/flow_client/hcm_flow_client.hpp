// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_HCM_FLOW_CLIENT_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_HCM_FLOW_CLIENT_HPP

#include <google/protobuf/message_lite.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/flow_context.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/ipc_session.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"

namespace qrb::adapter
{

/**
 * @brief The HCM-side IPC client for one Flow: owns the IPC Session and
 * synchronous request/response processing, and references its immutable
 * Contract conversion binding.
 *
 * Owned by HcmIpcAdapterNode, which also owns the QCNode lifecycle; this class
 * owns only connection resources and one synchronous request/response
 * operation per processed frame.
 *
 * Initialize() registers the Flow through the ROS Bridge Manager, connects to
 * the assigned data endpoint, and performs the handshake.
 *
 * Not thread-safe: the host calls into one instance from one
 * scheduling context.
 */
class HcmFlowClient
{
public:
  HcmFlowClient() = default;

  HcmFlowClient(const HcmFlowClient &) = delete;
  HcmFlowClient & operator=(const HcmFlowClient &) = delete;

  ~HcmFlowClient();

  /**
   * @brief Validate `config`, select the conversion binding and establish the Session.
   *
   * The ROS Bridge Manager must already be running: the bootstrap connect is
   * one attempt and fails at once with kResourceError otherwise. Register,
   * data connect and handshake then share one absolute deadline derived
   * from `config.startupTimeoutMs`. Success leaves the Client resources ready
   * for HcmIpcAdapterNode-driven Fetch()/Publish() calls.
   *
   * @return kOk, kUnsupportedContract (type ID not registered),
   *         kInvalidArgument, kResourceError (no Manager), or the Status the
   *         Manager or the handshake reported.
   */
  Status Initialize(const FlowConfig & config,
      const std::string & bootstrapSocketPath,
      std::string & diagnostic);

  /**
   * @brief ROS_TO_HCM: one FETCH_REQUEST/FETCH_RESPONSE round trip.
   *
   * Called once per processed frame descriptor. On kOk the Contract message
   * has been converted into `target`. For any Contract whose host storage is a
   * local DMA-BUF, the flow client wraps the converter call in DMA-BUF CPU sync:
   * CPU read on the received
   * DMA-BUF, and CPU write on the DMA-BUF behind `target` if it has one (heap
   * storage needs no sync). The received payload is copied into host storage,
   * and FETCH_COMPLETE is sent before this method returns, allowing the ROS
   * peer to release the producer-owned source Buffer. A failed sync is fatal.
   *
   * @return kOk (data converted), kBadState if not initialized, or a fatal
   *         Status that has closed the Session. Without a fresh ROS sample
   *         the bridge answers at once and this returns kNoData, leaving
   *         `target` untouched and the Session open. A frame error
   *         (IsFrameError(), e.g. a sample larger than `target`) drops that
   *         sample and also leaves the Session open; `target` may be
   *         partially written.
   */
  Status Fetch(QC::Memory::QCBufferDescriptorBase_t & target);

  /**
   * @brief HCM_TO_ROS: convert `source` and push it as a PUBLISH_REQUEST.
   *
   * For a DmaBuf Contract, the descriptor's inherited dmaHandle and validSize
   * describe the host-storage payload buffer. The fd travels in the same frame
   * via SCM_RIGHTS, while the valid byte count is checked against the
   * Contract. For a local DMA-BUF source, the flow client wraps the converter
   * call in DMA-BUF CPU read sync on `source`; a failed sync is fatal. The
   * caller and the host retain ownership of the buffer; the Adapter neither
   * duplicates nor recycles it.
   *
   * For an Inline Contract, leave the payload fields at their defaults.
   *
   * Awaits PUBLISH_RESPONSE. For a DmaBuf Contract, kOk means the ROS IPC
   * Bridge has copied the payload out of host storage and accepted the
   * publish, so the host may synchronously reclaim its buffer when this call
   * returns.
   *
   * @return kOk, kBadState if not initialized, a frame error (IsFrameError():
   *         nothing was published, the Session stays open), or a fatal
   *         Status that has closed the Session.
   */
  Status Publish(const QC::Memory::QCBufferDescriptorBase_t & source);

  /**
   * @brief Close the data Session, then best-effort unregister and release
   * resources. Idempotent.
   */
  void Close();

  /** @brief True from a successful Initialize() until Close(), explicit or after a fatal error. */
  bool IsOpen() const { return binding_ != nullptr; }

  const FlowContext & Context() const { return context_; }

private:
  using Deadline = std::chrono::steady_clock::time_point;

  Status RegisterManagedFlow(Deadline deadline, std::string & diagnostic);
  void UnregisterManagedFlow();

  /** @brief Tear down resources after a failed operation and return the original reason. */
  Status Fail(Status reason, const std::string & diagnostic);

  /**
   * @brief Drop a fetched sample with frame error `reason`. For a DmaBuf
   * Contract, FETCH_COMPLETE carries `reason` so the ROS peer releases the
   * producer Buffer and keeps serving. @return `reason`, or the fatal send failure
   */
  Status RejectFetched(std::uint64_t requestId, Status reason, const std::string & diagnostic);

  std::chrono::milliseconds Timeout() const
  {
    return std::chrono::milliseconds(context_.Config().timeoutMs);
  }

  FlowContext context_;
  IpcSession session_;
  const HcmContractBinding * binding_{nullptr};
  std::unique_ptr<google::protobuf::MessageLite> contractMessage_;
  std::string bootstrapSocketPath_;

  bool managerRegistered_{false};
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_HCM_FLOW_CLIENT_HPP
