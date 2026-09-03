// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_INLINE_ROS_FLOW_HANDLER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_INLINE_ROS_FLOW_HANDLER_HPP

#include <cstddef>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <string>

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/latest_sample.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_flow_handler.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_qos.hpp"

namespace qrb::adapter
{

/**
 * @brief Complete ROS endpoint implementation for an Inline Contract.
 *
 * The message pair supplies a RosConversion policy with two typed methods.
 */
template <typename RosMessageT, typename ContractT>
class InlineRosFlowHandler final : public TypedRosFlowHandler<ContractT>
{
public:
  Status Open(rclcpp::Node & node, const FlowConfig & config, std::string &) override
  {
    const rclcpp::QoS qos = MakeRosQos(config);
    if (config.direction == Direction::kRosToHcm) {
      subscription_ = node.create_subscription<RosMessageT>(
          config.rosTopic, qos, [this](typename RosMessageT::SharedPtr message) {
            if (message != nullptr)
              latest_.Store(*message);
          });
    } else {
      publisher_ = node.create_publisher<RosMessageT>(config.rosTopic, qos);
    }
    RCLCPP_INFO(node.get_logger(), "ROS endpoint QoS: profile=%s, depth=%zu",
        ToString(config.qosProfile), ResolveQosDepth(config.qosDepth));
    return Status::kOk;
  }

  void Close() noexcept override
  {
    subscription_.reset();
    latest_.Clear();
    publisher_.reset();
  }

  void CancelPendingFetch() noexcept override { latest_.Shutdown(); }

protected:
  Status FetchTyped(ContractT & target,
      OutboundPayload & outPayload,
      std::string & diagnostic,
      std::chrono::milliseconds sampleTimeout) override
  {
    if (subscription_ == nullptr)
      return Status::kBadState;

    RosMessageT message;
    if (!latest_.WaitAndTake(message, sampleTimeout))
      return Status::kNoData;

    outPayload = OutboundPayload{};
    ConversionResult converted =
        RosConversion<RosMessageT, ContractT>::RosToContract(message, target);
    if (!converted) {
      diagnostic += converted.Diagnostic();
      return Status::kConversionError;
    }
    return Status::kOk;
  }

  Status PublishTyped(const ContractT & source,
      InboundPayload & payload,
      std::string & diagnostic) override
  {
    if (publisher_ == nullptr)
      return Status::kBadState;
    if (payload.hasPayload) {
      diagnostic += "inline contract received a DMA-BUF payload; ";
      return Status::kProtocolError;
    }

    RosMessageT message;
    ConversionResult converted =
        RosConversion<RosMessageT, ContractT>::ContractToRos(source, message);
    if (!converted) {
      diagnostic += converted.Diagnostic();
      return Status::kConversionError;
    }
    publisher_->publish(message);
    return Status::kOk;
  }

private:
  typename rclcpp::Subscription<RosMessageT>::SharedPtr subscription_;
  LatestSample<RosMessageT> latest_;
  typename rclcpp::Publisher<RosMessageT>::SharedPtr publisher_;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_INLINE_ROS_FLOW_HANDLER_HPP
