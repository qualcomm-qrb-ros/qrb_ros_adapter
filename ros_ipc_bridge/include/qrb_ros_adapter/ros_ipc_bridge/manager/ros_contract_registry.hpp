// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_MANAGER_ROS_CONTRACT_REGISTRY_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_MANAGER_ROS_CONTRACT_REGISTRY_HPP

#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_flow_bridge_base.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/node_options.hpp>

namespace qrb::adapter
{

using RosFlowBridgeFactory = std::shared_ptr<RosFlowBridgeBase> ( * )(
    const rclcpp::NodeOptions &, FlowTerminalCallback );

/** @brief ROS-side implementation registered for one shared Contract Descriptor. */
struct RosContractBinding
{
    const ContractDescriptor *descriptor{ nullptr };
    std::string_view nodeName;
    RosFlowBridgeFactory createBridge{ nullptr };
};

std::size_t RosContractBindingCount();
const RosContractBinding *RosContractBindingAt( std::size_t index );
const RosContractBinding *FindRosContractBinding( std::string_view contractTypeName );

Status CreateFlowBridge( const RosContractBinding &binding, const rclcpp::NodeOptions &options,
    FlowTerminalCallback terminalCallback, std::shared_ptr<RosFlowBridgeBase> &outBridge,
    std::string &diagnostic );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_MANAGER_ROS_CONTRACT_REGISTRY_HPP
