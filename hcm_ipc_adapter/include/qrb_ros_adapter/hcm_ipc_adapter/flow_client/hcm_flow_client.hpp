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
 * Is owned directly by the GF-managed HcmIpcAdapterNode. The QCNode lifecycle
 * remains owned by HcmIpcAdapterNode; this class owns only connection resources
 * and one synchronous request/response operation per processed frame. This
 * class itself depends only on QCNode buffer descriptor interfaces, not on
 * Graph Framework scheduling types.
 *
 * Initialize() registers the Flow through the ROS Bridge Manager, connects to
 * the assigned data endpoint, and performs the handshake.
 *
 * Not thread-safe: the Graph Framework calls into one instance from one
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
   * Register, data connect and handshake share one absolute deadline derived
   * from `config.startupTimeoutMs`. Success leaves the Client resources ready
   * for HcmIpcAdapterNode-driven Fetch()/Publish() calls.
   *
   * @return kOk, kUnsupportedContract (type ID not registered), or
   *         kInvalidArgument.
   */
  Status Initialize(const FlowConfig & config,
      const std::string & bootstrapSocketPath,
      std::string & diagnostic);

  /**
   * @brief ROS_TO_HCM: one FETCH_REQUEST/FETCH_RESPONSE round trip.
   *
   * Called once per processed frame descriptor. On kOk the Contract message
   * has been converted into `target`. For a DMA-BUF Contract, the received
   * payload is synchronously copied into the GF-owned descriptor buffer
   * and FETCH_COMPLETE is sent before this method returns, allowing the ROS
   * peer to release the producer-owned source Buffer.
   *
   * @return kOk (data converted), kBadState if not initialized, or a fatal
   *         Status. The owning IPC Adapter Node maps a fatal result to the
   *         QCNode Error state. Exhausting the fresh ROS sample wait returns
   *         kNoData without closing the Session.
   */
  Status Fetch(QC::Memory::QCBufferDescriptorBase_t & target);

  /**
   * @brief HCM_TO_ROS: convert `source` and push it as a PUBLISH_REQUEST.
   *
   * For a DMA-BUF Contract, the descriptor's inherited dmaHandle and validSize
   * describe the GF-owned payload buffer. The fd travels in the same frame
   * via SCM_RIGHTS, while the valid byte count is checked against the
   * Contract. The caller and Graph Framework retain ownership of the HCM
   * buffer; the Adapter neither duplicates nor recycles it.
   *
   * For an Inline Contract, leave the payload fields at their defaults.
   *
   * Awaits PUBLISH_RESPONSE. For a DMA-BUF Contract, kOk means the ROS IPC
   * Bridge has copied the payload into ROS-owned storage and accepted the
   * publish, so Graph Framework may synchronously reclaim the HCM buffer when
   * this call returns. No asynchronous completion or receiver thread exists.
   *
   * @return kOk, kBadState if not initialized, kInvalidArgument, or a fatal
   *         Status. The owning IPC Adapter Node maps a fatal result to the
   *         QCNode Error state.
   */
  Status Publish(const QC::Memory::QCBufferDescriptorBase_t & source);

  /**
   * @brief Close the data Session, then best-effort unregister and release
   * resources. Idempotent.
   */
  void Close();

  const FlowContext & Context() const { return context_; }

private:
  using Deadline = std::chrono::steady_clock::time_point;

  Status RegisterManagedFlow(Deadline deadline, std::string & diagnostic);
  void UnregisterManagedFlow();

  /** @brief Tear down resources after a failed operation and return the original reason. */
  Status Fail(Status reason, const std::string & diagnostic);

  std::chrono::milliseconds RequestTimeout() const
  {
    return std::chrono::milliseconds(context_.Config().requestTimeoutMs);
  }

  FlowContext context_;
  IpcSession session_;
  const HcmContractBinding * binding_{ nullptr };
  std::unique_ptr<google::protobuf::MessageLite> contractMessage_;
  std::string bootstrapSocketPath_;

  bool managerRegistered_{ false };
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_HCM_FLOW_CLIENT_HPP
