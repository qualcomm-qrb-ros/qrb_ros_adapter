// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/manager/ros_contract_registry.hpp"

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/dma_buf_ros_flow_handler.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/inline_ros_flow_handler.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_qos.hpp"

#define QRB_CONTRACT_MAPPING_INCLUDE_ROS_HEADERS
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING_INCLUDE_ROS_HEADERS

#include <exception>
#include <iterator>

namespace qrb::adapter
{

namespace
{

template <typename RosMessageT, typename ContractT, PayloadClass Payload>
std::shared_ptr<RosFlowBridgeBase> CreateTypedRosFlowBridge(const char * nodeName,
    std::string_view contractTypeName,
    const rclcpp::NodeOptions & options,
    FlowTerminalCallback terminalCallback)
{
  std::unique_ptr<RosFlowHandler> handler;
  if constexpr (Payload == PayloadClass::kInline) {
    handler = std::make_unique<InlineRosFlowHandler<RosMessageT, ContractT>>();
  } else {
    static_assert(Payload == PayloadClass::kDmaBuf);
    handler = std::make_unique<DmaBufRosFlowHandler<RosMessageT, ContractT>>(
        kDefaultRosDmaBufferPoolSize);
  }
  return std::make_shared<RosFlowBridgeBase>(
      nodeName, options, contractTypeName, std::move(handler), std::move(terminalCallback));
}

#define QRB_DEFINE_ROS_FACTORY(                                                                    \
    Id, TypeName, Payload, ContractT, HcmDescriptorT, RosMessageT, NodeName)                       \
  std::shared_ptr<RosFlowBridgeBase> Create##Id##RosFlowBridge(                                    \
      const rclcpp::NodeOptions & options, FlowTerminalCallback terminalCallback)                  \
  {                                                                                                \
    return CreateTypedRosFlowBridge<RosMessageT, ContractT, PayloadClass::k##Payload>(             \
        NodeName, TypeName, options, std::move(terminalCallback));                                 \
  }
#define QRB_CONTRACT_MAPPING QRB_DEFINE_ROS_FACTORY
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
#undef QRB_DEFINE_ROS_FACTORY

const RosContractBinding kRosBindings[] = {
#define QRB_CONTRACT_MAPPING(                                                                      \
    Id, TypeName, Payload, ContractT, HcmDescriptorT, RosMessageT, NodeName)                       \
  { FindContractDescriptor(TypeName), NodeName, &Create##Id##RosFlowBridge },
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
};

}  // namespace

std::size_t RosContractBindingCount()
{
  return std::size(kRosBindings);
}

const RosContractBinding * RosContractBindingAt(std::size_t index)
{
  return index < std::size(kRosBindings) ? &kRosBindings[index] : nullptr;
}

const RosContractBinding * FindRosContractBinding(std::string_view contractTypeName)
{
  for (const RosContractBinding & binding : kRosBindings) {
    if (binding.descriptor != nullptr && binding.descriptor->typeName == contractTypeName) {
      return &binding;
    }
  }
  return nullptr;
}

Status CreateFlowBridge(const RosContractBinding & binding,
    const rclcpp::NodeOptions & options,
    FlowTerminalCallback terminalCallback,
    std::shared_ptr<RosFlowBridgeBase> & outBridge,
    std::string & diagnostic)
{
  outBridge.reset();
  if (binding.descriptor == nullptr || binding.createBridge == nullptr) {
    diagnostic = "Contract has no ROS Flow implementation";
    return Status::kUnsupportedContract;
  }

  try {
    outBridge = binding.createBridge(options, std::move(terminalCallback));
    return outBridge != nullptr ? Status::kOk : Status::kInternalError;
  } catch (const std::exception & error) {
    diagnostic = error.what();
    return Status::kResourceError;
  }
}

}  // namespace qrb::adapter
