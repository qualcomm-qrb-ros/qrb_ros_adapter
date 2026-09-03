// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP

#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"

#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"
#include "qrb/contract/sensor/v1/point_cloud2.pb.h"

#include <cstdint>
#include <string>

namespace qrb::adapter
{

ConversionResult ValidateFrameId( const std::string &frameId );

ConversionResult ValidatePoseWithCovarianceStamped(
    const qrb::contract::geometry::v1::PoseWithCovarianceStamped &pose );

ConversionResult ValidatePointCloud2(
    const qrb::contract::sensor::v1::PointCloud2 &cloud,
    std::uint64_t externalPayloadSize );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
