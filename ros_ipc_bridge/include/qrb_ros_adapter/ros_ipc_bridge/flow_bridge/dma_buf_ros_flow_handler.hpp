// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DMA_BUF_ROS_FLOW_HANDLER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DMA_BUF_ROS_FLOW_HANDLER_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <limits>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/type_adapter.hpp>
#include <string>
#include <utility>

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/latest_sample.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_dma_buffer_pool.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_flow_handler.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_qos.hpp"

namespace qrb::adapter
{

/**
 * @brief Complete ROS endpoint and ownership implementation for one DMA-BUF Contract.
 *
 * The message pair supplies one RosConversion policy. Its two methods bind the
 * message's DMA-BUF through typed payload contexts while this handler owns all
 * transport validation, IPC transfer and copy lifecycle.
 */
template <typename RosMessageT, typename ContractT>
class DmaBufRosFlowHandler final : public TypedRosFlowHandler<ContractT>
{
  static_assert(rclcpp::TypeAdapter<RosMessageT>::is_specialized::value,
      "a DMA-BUF Flow must use a qrb_ros_transport custom type with a registered "
      "rclcpp::TypeAdapter");

public:
  DmaBufRosFlowHandler(int defaultPoolSize, std::string heapName = "/dev/dma_heap/system")
    : defaultPoolSize_(defaultPoolSize), heapName_(std::move(heapName))
  {
  }

  Status Open(rclcpp::Node & node, const FlowConfig & config, std::string & diagnostic) override
  {
    if (defaultPoolSize_ <= 0) {
      diagnostic += "ROS DMA-BUF pool size must be positive; ";
      return Status::kInvalidArgument;
    }
    const int configuredPoolSize =
        node.declare_parameter<int>("ros_dma_buffer_pool_size", defaultPoolSize_);
    if (configuredPoolSize <= 0) {
      diagnostic += "ros_dma_buffer_pool_size must be positive; ";
      return Status::kInvalidArgument;
    }
    const std::size_t effectiveDepth = ResolveQosDepth(config.qosDepth);
    if (config.direction == Direction::kHcmToRos &&
        effectiveDepth >= static_cast<std::size_t>(configuredPoolSize)) {
      diagnostic +=
          "effective qos_depth must be smaller than "
          "ros_dma_buffer_pool_size for HCM_TO_ROS DMA-BUF flows; ";
      return Status::kInvalidArgument;
    }

    config_ = config;
    const rclcpp::QoS qos = MakeRosQos(config);
    if (config.direction == Direction::kRosToHcm) {
      subscription_ = node.create_subscription<RosMessageT>(config.rosTopic, qos,
          [this](std::shared_ptr<RosMessageT> message) { latest_.Store(std::move(message)); });
    } else {
      pool_ = std::make_unique<RosDmaBufferPool>(
          static_cast<std::size_t>(configuredPoolSize), heapName_);
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
    if (pool_ != nullptr)
      pool_->Shutdown();
    pool_.reset();
  }

  void CancelPendingFetch() noexcept override { latest_.Shutdown(); }

protected:
  Status FetchTyped(ContractT & target,
      OutboundPayload & outPayload,
      std::string & diagnostic,
      std::chrono::milliseconds sampleTimeout) override
  {
    outPayload = OutboundPayload{};
    if (subscription_ == nullptr)
      return Status::kBadState;

    std::shared_ptr<RosMessageT> message;
    if (!latest_.WaitAndTake(message, sampleTimeout))
      return Status::kNoData;

    if (message == nullptr) {
      diagnostic += "received a null ROS transport message; ";
      return Status::kConversionError;
    }
    RosToContractPayload conversionPayload;
    ConversionResult converted =
        RosConversion<RosMessageT, ContractT>::RosToContract(*message, target, conversionPayload);
    if (!converted) {
      diagnostic += converted.Diagnostic();
      return Status::kConversionError;
    }
    if (!conversionPayload.IsBound()) {
      diagnostic += "DMA-BUF conversion did not bind a source payload; ";
      return Status::kConversionError;
    }
    // SCM_RIGHTS transfers another reference to the same kernel Buffer.
    // RosFlowBridgeBase retains this owner until HCM confirms its copy.
    outPayload.fd = conversionPayload.Buffer()->fd();
    outPayload.keepAlive = conversionPayload.Buffer();
    outPayload.offset = conversionPayload.Offset();
    outPayload.hasPayload = true;
    return Status::kOk;
  }

  Status PublishTyped(const ContractT & source,
      InboundPayload & payload,
      std::string & diagnostic) override
  {
    if (publisher_ == nullptr)
      return Status::kBadState;
    if (!payload.hasPayload || !payload.fd.IsValid()) {
      diagnostic += "DMA-BUF contract received no payload fd; ";
      return Status::kProtocolError;
    }

    RosMessageT message;
    if (payload.offset > payload.capacity) {
      diagnostic += "Contract payload exceeds the received DMA-BUF range; ";
      return Status::kConversionError;
    }

    ContractToRosPayload conversionPayload(payload.capacity - payload.offset);
    ConversionResult converted =
        RosConversion<RosMessageT, ContractT>::ContractToRos(source, message, conversionPayload);
    if (!converted) {
      diagnostic += converted.Diagnostic();
      return Status::kConversionError;
    }
    if (!conversionPayload.IsBound() ||
        conversionPayload.Size() > std::numeric_limits<std::size_t>::max()) {
      diagnostic += "DMA-BUF conversion did not bind a valid destination payload; ";
      return Status::kConversionError;
    }

    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer;
    Status copied = CopyToRosBuffer(
        payload, static_cast<std::size_t>(conversionPayload.Size()), buffer, diagnostic);
    if (copied != Status::kOk)
      return copied;

    if (!conversionPayload.Assign(std::move(buffer))) {
      diagnostic += "failed to assign the ROS DMA-BUF destination; ";
      return Status::kInternalError;
    }
    publisher_->publish(std::move(message));
    return Status::kOk;
  }

private:
  Status CopyToRosBuffer(InboundPayload & payload,
      std::size_t size,
      std::shared_ptr<lib_mem_dmabuf::DmaBuffer> & destination,
      std::string & diagnostic)
  {
    if (pool_ == nullptr || size == 0 || payload.offset > payload.capacity ||
        size > payload.capacity - payload.offset) {
      diagnostic += "invalid inbound DMA-BUF size or unavailable ROS buffer pool; ";
      return Status::kBadState;
    }

    destination = pool_->Acquire(size, std::chrono::milliseconds(config_.requestTimeoutMs));
    if (destination == nullptr) {
      diagnostic += "ROS DMA-BUF pool acquisition timed out or allocation failed; ";
      return Status::kResourceError;
    }

    auto source =
        std::make_shared<lib_mem_dmabuf::DmaBuffer>(payload.fd.Release(), payload.capacity);
    if (!source->map()) {
      diagnostic += "failed to map the HCM DMA-BUF; ";
      return Status::kResourceError;
    }
    if (!source->sync_start()) {
      source->unmap();
      diagnostic += "failed to synchronize the HCM DMA-BUF; ";
      return Status::kResourceError;
    }

    auto * sourceBytes = static_cast<std::uint8_t *>(source->addr());
    const bool copied = destination->set_data(sourceBytes + payload.offset, size);
    const bool sourceSyncEnded = source->sync_end();
    const bool sourceUnmapped = source->unmap();
    const bool destinationUnmapped = destination->unmap();
    if (!copied || !sourceSyncEnded || !sourceUnmapped || !destinationUnmapped) {
      diagnostic += "failed to copy or synchronize the ROS-owned DMA-BUF; ";
      destination.reset();
      return Status::kResourceError;
    }

    payload.hasPayload = false;
    return Status::kOk;
  }

  int defaultPoolSize_;
  std::string heapName_;
  FlowConfig config_;
  typename rclcpp::Subscription<RosMessageT>::SharedPtr subscription_;
  LatestSample<std::shared_ptr<RosMessageT>> latest_;
  typename rclcpp::Publisher<RosMessageT>::SharedPtr publisher_;
  std::unique_ptr<RosDmaBufferPool> pool_;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DMA_BUF_ROS_FLOW_HANDLER_HPP
