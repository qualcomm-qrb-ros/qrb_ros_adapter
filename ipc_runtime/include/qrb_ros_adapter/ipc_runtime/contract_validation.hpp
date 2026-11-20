// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP

#include <cstdint>
#include <string>

#include "qrb/contract/common/v1/header.pb.h"
#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"
#include "qrb/contract/nav/v1/odometry.pb.h"
#include "qrb/contract/sensor/v1/point_cloud2.pb.h"
#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

ConversionResult ValidateFrameId(const std::string & frameId);

/** @brief Checks `frame_id` and that `stamp.nanosec` is below one second. */
ConversionResult ValidateHeader(const qrb::contract::common::v1::Header & header);

ConversionResult ValidatePoseWithCovarianceStamped(
    const qrb::contract::geometry::v1::PoseWithCovarianceStamped & pose);

/**
 * @brief Checks the header, `child_frame_id` (as a frame_id), the pose as
 * ValidatePoseWithCovarianceStamped() does, and that the twist is finite with an
 * empty or 36-value covariance.
 */
ConversionResult ValidateOdometry(const qrb::contract::nav::v1::Odometry & odometry);

ConversionResult ValidatePointCloud2(const qrb::contract::sensor::v1::PointCloud2 & cloud,
    std::uint64_t externalPayloadSize);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
