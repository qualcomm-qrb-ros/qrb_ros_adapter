// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_FLOW_HANDLER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_FLOW_HANDLER_HPP

#include <google/protobuf/message_lite.h>

#include <chrono>
#include <rclcpp/node.hpp>
#include <string>

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/flow_payload.hpp"

namespace qrb::adapter
{

/**
 * @brief Type-specific ROS endpoint and conversion strategy owned by a Flow Bridge.
 *
 * RosFlowBridgeBase owns this object for its complete lifetime. Its service
 * thread is stopped before Close() and the handler destructor run, so handler
 * implementations never need to coordinate with a derived Node destructor.
 */
class RosFlowHandler
{
public:
  virtual ~RosFlowHandler() = default;

  virtual Status Open(rclcpp::Node & node, const FlowConfig & config, std::string & diagnostic) = 0;
  virtual void Close() noexcept = 0;

  virtual Status Fetch(google::protobuf::MessageLite & target,
      OutboundPayload & outPayload,
      std::string & diagnostic,
      std::chrono::milliseconds sampleTimeout) = 0;
  virtual void CancelPendingFetch() noexcept = 0;
  virtual Status Publish(const google::protobuf::MessageLite & source,
      InboundPayload & payload,
      std::string & diagnostic) = 0;
};

/**
 * @brief Centralizes MessageLite type erasure for a Contract-specific handler.
 *
 * Custom handlers, normally only DMA-BUF handlers, implement the typed methods
 * below and cannot accidentally repeat or omit the runtime Contract check.
 */
template <typename ContractT>
class TypedRosFlowHandler : public RosFlowHandler
{
public:
  Status Fetch(google::protobuf::MessageLite & target,
      OutboundPayload & outPayload,
      std::string & diagnostic,
      std::chrono::milliseconds sampleTimeout) final
  {
    auto * contract = dynamic_cast<ContractT *>(&target);
    if (contract == nullptr) {
      diagnostic += "ROS handler received an unexpected Contract target type; ";
      return Status::kInternalError;
    }
    return FetchTyped(*contract, outPayload, diagnostic, sampleTimeout);
  }

  Status Publish(const google::protobuf::MessageLite & source,
      InboundPayload & payload,
      std::string & diagnostic) final
  {
    const auto * contract = dynamic_cast<const ContractT *>(&source);
    if (contract == nullptr) {
      diagnostic += "ROS handler received an unexpected Contract source type; ";
      return Status::kInternalError;
    }
    return PublishTyped(*contract, payload, diagnostic);
  }

protected:
  virtual Status FetchTyped(ContractT & target,
      OutboundPayload & outPayload,
      std::string & diagnostic,
      std::chrono::milliseconds sampleTimeout) = 0;
  virtual Status PublishTyped(const ContractT & source,
      InboundPayload & payload,
      std::string & diagnostic) = 0;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_FLOW_HANDLER_HPP
