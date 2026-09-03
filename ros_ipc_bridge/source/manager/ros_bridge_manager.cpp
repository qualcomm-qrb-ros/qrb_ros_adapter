// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/manager/ros_bridge_manager.hpp"

#include <sys/eventfd.h>
#include <sys/random.h>
#include <unistd.h>

#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <rclcpp/utilities.hpp>
#include <stdexcept>
#include <vector>

#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"
#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_qos.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/manager/ros_contract_registry.hpp"

namespace qrb::adapter
{

namespace
{
constexpr std::chrono::milliseconds kBootstrapPollInterval{ 200 };
constexpr std::chrono::milliseconds kBootstrapRequestTimeout{ 2000 };
}  // namespace

RosBridgeManager::RosBridgeManager(std::shared_ptr<rclcpp::Executor> executor,
    const rclcpp::NodeOptions & options)
  : rclcpp::Node("qrb_ros_bridge_manager", options), executor_(std::move(executor))
{
  if (executor_ == nullptr)
    throw std::invalid_argument("executor must not be null");
  bootstrapSocketPath_ = this->declare_parameter<std::string>(
      "bootstrap_socket_path", "/run/qrb_ros_adapter/bootstrap.sock");
  flowSocketDirectory_ =
      this->declare_parameter<std::string>("flow_socket_directory", "/run/qrb_ros_adapter/flows");
  const int maxFlows = this->declare_parameter<int>("max_flows", 32);
  const int rosDmaBufferPoolSize =
      this->declare_parameter<int>("ros_dma_buffer_pool_size", kDefaultRosDmaBufferPoolSize);
  if (bootstrapSocketPath_.empty() || flowSocketDirectory_.empty() || maxFlows <= 0 ||
      rosDmaBufferPoolSize <= 0) {
    throw std::invalid_argument("invalid ROS Bridge Manager parameters");
  }
  if (!std::filesystem::is_directory(flowSocketDirectory_)) {
    throw std::invalid_argument("flow_socket_directory must be created by the deployment system");
  }
  maxFlows_ = static_cast<std::size_t>(maxFlows);
  rosDmaBufferPoolSize_ = rosDmaBufferPoolSize;

  terminalEventFd_.Reset(::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK));
  if (!terminalEventFd_.IsValid()) {
    throw std::runtime_error("failed to create Manager terminal eventfd");
  }
}

RosBridgeManager::~RosBridgeManager()
{
  Stop();
}

void RosBridgeManager::Start()
{
  if (running_.exchange(true))
    return;
  Status listened = bootstrapListener_.Listen(bootstrapSocketPath_);
  if (listened != Status::kOk) {
    running_.store(false);
    RCLCPP_FATAL(get_logger(), "bootstrap listen failed: %s", ToString(listened));
    throw std::runtime_error(std::string("bootstrap listen failed: ") + ToString(listened));
  }
  bootstrapThread_ = std::thread(&RosBridgeManager::BootstrapLoop, this);
}

void RosBridgeManager::Stop()
{
  const bool wasRunning = running_.exchange(false);
  if (wasRunning)
    WakeManager();
  if (bootstrapThread_.joinable())
    bootstrapThread_.join();
  bootstrapListener_.Close();

  std::unordered_map<std::string, ManagedFlow> flows;
  {
    std::lock_guard<std::mutex> lock(flowsMutex_);
    flows.swap(flows_);
  }
  for (auto & [flowId, flow] : flows) {
    (void)flowId;
    executor_->remove_node(flow.component->get_node_base_interface());
  }

  std::lock_guard<std::mutex> lock(terminalEventsMutex_);
  terminalEvents_.clear();
}

void RosBridgeManager::BootstrapLoop()
{
  while (running_.load()) {
    IpcSession client;
    bool woken = false;
    Status accepted = bootstrapListener_.AcceptConnection(
        client, kBootstrapPollInterval, terminalEventFd_.Get(), woken);
    if (woken) {
      DrainTerminalEvents();
      continue;
    }
    if (accepted == Status::kTimeout)
      continue;
    if (accepted != Status::kOk) {
      RCLCPP_ERROR(get_logger(), "bootstrap accept failed: %s", ToString(accepted));
      running_.store(false);
      rclcpp::shutdown(
          this->get_node_base_interface()->get_context(), "fatal bootstrap Listener failure");
      break;
    }
    HandleBootstrapClient(client);
    client.Close();
    DrainTerminalEvents();
  }
}

void RosBridgeManager::HandleBootstrapClient(IpcSession & client)
{
  TransportHeader header;
  std::vector<std::uint8_t> payload;
  std::vector<OwnedFd> fds;
  Status received = client.ReceiveFrame(header, payload, fds, kBootstrapRequestTimeout);
  if (received != Status::kOk || header.fdCount != 0)
    return;

  if (header.operation == static_cast<std::uint16_t>(Operation::kRegisterFlowRequest)) {
    ipc_proto::RegisterFlowRequest request;
    ipc_proto::RegisterFlowResponse response;
    std::string socketPath;
    std::string instanceToken;
    std::string diagnostic;
    Status result = IpcSession::ParseProtoPayload(payload, request);
    if (result == Status::kOk) {
      result = RegisterFlow(request, socketPath, instanceToken, diagnostic);
    }
    response.set_status(ToProtoStatusCode(result));
    response.set_data_socket_path(socketPath);
    response.set_instance_token(instanceToken);
    response.set_diagnostic_message(diagnostic);
    Status sent =
        client.SendProtoFrame(Operation::kRegisterFlowResponse, header.requestId, response);
    if (result == Status::kOk && sent != Status::kOk) {
      std::string ignored;
      UnregisterFlow(request.flow_id(), instanceToken, ignored);
    }
    return;
  }

  if (header.operation == static_cast<std::uint16_t>(Operation::kUnregisterFlowRequest)) {
    ipc_proto::UnregisterFlowRequest request;
    ipc_proto::UnregisterFlowResponse response;
    std::string diagnostic;
    Status result = IpcSession::ParseProtoPayload(payload, request);
    if (result == Status::kOk) {
      result = UnregisterFlow(request.flow_id(), request.instance_token(), diagnostic);
    }
    response.set_status(ToProtoStatusCode(result));
    response.set_diagnostic_message(diagnostic);
    client.SendProtoFrame(Operation::kUnregisterFlowResponse, header.requestId, response);
  }
}

Status RosBridgeManager::RegisterFlow(const ipc_proto::RegisterFlowRequest & request,
    std::string & dataSocketPath,
    std::string & instanceToken,
    std::string & diagnostic)
{
  const RosContractBinding * rosBinding = FindRosContractBinding(request.contract_type_name());
  if (rosBinding == nullptr || rosBinding->descriptor == nullptr) {
    diagnostic = "contract_type_name has no ROS implementation";
    return Status::kUnsupportedContract;
  }
  const ContractDescriptor & descriptor = *rosBinding->descriptor;

  FlowConfig config;
  config.flowId = request.flow_id();
  config.contractTypeName = request.contract_type_name();
  config.direction = FromProtoDirection(request.direction());
  config.rosTopic = request.ros_topic();
  config.requestTimeoutMs = request.request_timeout_ms();
  config.sampleTimeoutMs = request.sample_timeout_ms();
  config.startupTimeoutMs = request.startup_timeout_ms();
  config.qosProfile = FromProtoQosProfile(request.qos_profile());
  config.qosDepth = request.qos_depth();
  FlowRegistration registration;
  {
    std::lock_guard<std::mutex> lock(flowsMutex_);
    if (flows_.find(config.flowId) != flows_.end()) {
      diagnostic = "flow_id is already registered";
      return Status::kInvalidArgument;
    }
    if (flows_.size() >= maxFlows_) {
      diagnostic = "maximum Flow count reached";
      return Status::kResourceError;
    }
    registration.dataSocketPath = MakeDataSocketPath(config.flowId);
    registration.instanceToken = MakeInstanceToken();
    if (registration.instanceToken.empty()) {
      diagnostic = "failed to generate Flow instance token";
      return Status::kResourceError;
    }
  }

  Status validated = ValidateFlowConfig(config, descriptor, diagnostic);
  if (validated != Status::kOk)
    return validated;

  const std::size_t effectiveDepth = ResolveQosDepth(config.qosDepth);
  if (descriptor.payloadClass == PayloadClass::kDmaBuf &&
      config.direction == Direction::kHcmToRos &&
      effectiveDepth >= static_cast<std::size_t>(rosDmaBufferPoolSize_)) {
    diagnostic =
        "effective qos_depth must be smaller than ros_dma_buffer_pool_size "
        "for HCM_TO_ROS DMA-BUF flows";
    return Status::kInvalidArgument;
  }

  {
    std::lock_guard<std::mutex> lock(flowsMutex_);
    for (const auto & [existingId, existing] : flows_) {
      (void)existingId;
      if (existing.config.rosTopic == config.rosTopic &&
          existing.config.contractTypeName == config.contractTypeName &&
          existing.config.direction != config.direction) {
        diagnostic =
            "opposite-direction Adapter flows for the same Contract and ROS "
            "topic are not allowed; use distinct ROS topics to prevent a "
            "feedback loop";
        return Status::kInvalidArgument;
      }
    }
  }

  std::string nodeName = "qrb_flow_";
  for (char c : config.flowId) {
    nodeName.push_back(std::isalnum(static_cast<unsigned char>(c)) ? c : '_');
  }
  std::vector<rclcpp::Parameter> parameters = {
    { "flow_id", config.flowId },
    { "contract_type_name", config.contractTypeName },
    { "direction", std::string(ToString(config.direction)) },
    { "ros_topic", config.rosTopic },
    { "data_socket_path", registration.dataSocketPath },
    { "instance_token", registration.instanceToken },
    { "request_timeout_ms", static_cast<std::int64_t>(config.requestTimeoutMs) },
    { "sample_timeout_ms", static_cast<std::int64_t>(config.sampleTimeoutMs) },
    { "startup_timeout_ms", static_cast<std::int64_t>(config.startupTimeoutMs) },
    { "qos_profile", std::string(ToString(config.qosProfile)) },
    { "qos_depth", static_cast<std::int64_t>(config.qosDepth) },
    { "ros_dma_buffer_pool_size", rosDmaBufferPoolSize_ },
  };
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.parameter_overrides(parameters);
  options.arguments({ "--ros-args", "-r", "__node:=" + nodeName });

  std::shared_ptr<RosFlowBridgeBase> component;
  Status created = CreateFlowBridge(
      *rosBinding, options,
      [this](FlowTerminalEvent event) { QueueTerminalEvent(std::move(event)); }, component,
      diagnostic);
  if (created != Status::kOk)
    return created;
  executor_->add_node(component->get_node_base_interface());

  {
    std::lock_guard<std::mutex> lock(flowsMutex_);
    flows_.emplace(config.flowId,
        ManagedFlow{ component, config, registration.dataSocketPath, registration.instanceToken });
  }
  dataSocketPath = registration.dataSocketPath;
  instanceToken = registration.instanceToken;
  RCLCPP_INFO(get_logger(), "registered flow '%s' at '%s'", config.flowId.c_str(),
      registration.dataSocketPath.c_str());
  return Status::kOk;
}

Status RosBridgeManager::UnregisterFlow(const std::string & flowId,
    const std::string & instanceToken,
    std::string & diagnostic)
{
  std::shared_ptr<RosFlowBridgeBase> component;
  {
    std::lock_guard<std::mutex> lock(flowsMutex_);
    auto it = flows_.find(flowId);
    if (it == flows_.end()) {
      return Status::kOk;
    }
    if (instanceToken.empty() || it->second.instanceToken != instanceToken) {
      diagnostic = "Flow instance token does not match the active registration";
      return Status::kInvalidArgument;
    }
    component = it->second.component;
    flows_.erase(it);
  }
  executor_->remove_node(component->get_node_base_interface());
  component.reset();
  RCLCPP_INFO(get_logger(), "unregistered flow '%s'", flowId.c_str());
  return Status::kOk;
}

void RosBridgeManager::QueueTerminalEvent(FlowTerminalEvent event)
{
  if (!running_.load())
    return;
  {
    std::lock_guard<std::mutex> lock(terminalEventsMutex_);
    terminalEvents_.push_back(std::move(event));
  }
  WakeManager();
}

void RosBridgeManager::WakeManager()
{
  if (!terminalEventFd_.IsValid())
    return;
  const std::uint64_t one = 1;
  const ssize_t written = ::write(terminalEventFd_.Get(), &one, sizeof(one));
  (void)written;
}

void RosBridgeManager::DrainTerminalEvents()
{
  std::uint64_t value = 0;
  while (::read(terminalEventFd_.Get(), &value, sizeof(value)) > 0) {
  }

  std::deque<FlowTerminalEvent> events;
  {
    std::lock_guard<std::mutex> lock(terminalEventsMutex_);
    events.swap(terminalEvents_);
  }
  for (const FlowTerminalEvent & event : events)
    RemoveTerminatedFlow(event);
}

void RosBridgeManager::RemoveTerminatedFlow(const FlowTerminalEvent & event)
{
  std::shared_ptr<RosFlowBridgeBase> component;
  {
    std::lock_guard<std::mutex> lock(flowsMutex_);
    auto it = flows_.find(event.flowId);
    if (it == flows_.end() || it->second.instanceToken != event.instanceToken)
      return;
    component = it->second.component;
    flows_.erase(it);
  }

  executor_->remove_node(component->get_node_base_interface());
  component.reset();
  RCLCPP_WARN(get_logger(), "removed terminated flow '%s': %s (%s)", event.flowId.c_str(),
      event.diagnostic.c_str(), ToString(event.reason));
}

std::string RosBridgeManager::MakeDataSocketPath(const std::string & flowId)
{
  std::string safe;
  for (char c : flowId) {
    safe.push_back(std::isalnum(static_cast<unsigned char>(c)) ? c : '_');
    if (safe.size() == 32)
      break;
  }
  if (safe.empty())
    safe = "flow";
  return flowSocketDirectory_ + "/" + safe + "-" + std::to_string(++nextEndpointId_) + ".sock";
}

std::string RosBridgeManager::MakeInstanceToken()
{
  std::array<unsigned char, 16> bytes{};
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t received = ::getrandom(bytes.data() + offset, bytes.size() - offset, 0);
    if (received > 0) {
      offset += static_cast<std::size_t>(received);
      continue;
    }
    if (received < 0 && errno == EINTR)
      continue;
    return {};
  }

  constexpr char kHex[] = "0123456789abcdef";
  std::string token;
  token.reserve(bytes.size() * 2);
  for (unsigned char byte : bytes) {
    token.push_back(kHex[byte >> 4]);
    token.push_back(kHex[byte & 0x0F]);
  }
  return token;
}

}  // namespace qrb::adapter
