// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_GEOMETRY_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_GEOMETRY_HPP

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include "qrb/contract/geometry/v1/pose_with_covariance.pb.h"
#include "qrb/contract/geometry/v1/twist_with_covariance.pb.h"

namespace qrb::adapter
{

/**
 * @brief Field-for-field copies between the ROS and Contract geometry messages,
 * shared by the inline geometry converters.
 *
 * ROS to Contract always writes all 36 covariance values. Contract to ROS
 * expects a validated Contract: its covariance is empty (copied as the zero
 * matrix, "unknown") or holds the 36 values the ROS array holds.
 */
void PoseWithCovarianceToContract(const geometry_msgs::msg::PoseWithCovariance & source,
    qrb::contract::geometry::v1::PoseWithCovariance & target);
void PoseWithCovarianceToRos(const qrb::contract::geometry::v1::PoseWithCovariance & source,
    geometry_msgs::msg::PoseWithCovariance & target);
void TwistWithCovarianceToContract(const geometry_msgs::msg::TwistWithCovariance & source,
    qrb::contract::geometry::v1::TwistWithCovariance & target);
void TwistWithCovarianceToRos(const qrb::contract::geometry::v1::TwistWithCovariance & source,
    geometry_msgs::msg::TwistWithCovariance & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_GEOMETRY_HPP
