// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_FLOW_PAYLOAD_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_FLOW_PAYLOAD_HPP

#include "qrb_ros_adapter/ipc_runtime/owned_fd.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace qrb::adapter
{

struct OutboundPayload
{
    int fd{ -1 };
    std::shared_ptr<void> keepAlive;
    std::uint64_t offset{ 0 };
    bool hasPayload{ false };
};

struct InboundPayload
{
    OwnedFd fd;
    std::size_t capacity{ 0 };
    std::size_t offset{ 0 };
    bool hasPayload{ false };
};

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_FLOW_PAYLOAD_HPP
