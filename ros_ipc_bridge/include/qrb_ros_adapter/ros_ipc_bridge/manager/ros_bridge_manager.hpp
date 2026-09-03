// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_ROS_BRIDGE_MANAGER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_ROS_BRIDGE_MANAGER_HPP

#include <atomic>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/executor.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_flow_bridge_base.hpp"

namespace qrb::adapter
{

/** Persistent bootstrap manager for dynamically created one-Flow ROS nodes. */
class RosBridgeManager : public rclcpp::Node
{
public:
  explicit RosBridgeManager(std::shared_ptr<rclcpp::Executor> executor,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RosBridgeManager() override;

  void Start();
  void Stop();

private:
  struct ManagedFlow
  {
    std::shared_ptr<RosFlowBridgeBase> component;
    FlowConfig config;
    std::string socketPath;
    std::string instanceToken;
  };

  void BootstrapLoop();
  void HandleBootstrapClient(IpcSession & client);
  Status RegisterFlow(const ipc_proto::RegisterFlowRequest & request,
      std::string & dataSocketPath,
      std::string & instanceToken,
      std::string & diagnostic);
  Status UnregisterFlow(const std::string & flowId,
      const std::string & instanceToken,
      std::string & diagnostic);
  void QueueTerminalEvent(FlowTerminalEvent event);
  void DrainTerminalEvents();
  void RemoveTerminatedFlow(const FlowTerminalEvent & event);
  void WakeManager();
  std::string MakeDataSocketPath(const std::string & flowId);
  static std::string MakeInstanceToken();

  std::shared_ptr<rclcpp::Executor> executor_;
  std::string bootstrapSocketPath_;
  std::string flowSocketDirectory_;
  std::size_t maxFlows_{ 32 };
  int rosDmaBufferPoolSize_{ 12 };
  std::uint64_t nextEndpointId_{ 0 };

  mutable std::mutex flowsMutex_;
  std::unordered_map<std::string, ManagedFlow> flows_;

  std::mutex terminalEventsMutex_;
  std::deque<FlowTerminalEvent> terminalEvents_;
  OwnedFd terminalEventFd_;
  IpcSession bootstrapListener_;

  std::atomic<bool> running_{ false };
  std::thread bootstrapThread_;
};

}  // namespace qrb::adapter

#endif
