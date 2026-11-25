// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_GEOMETRY_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_GEOMETRY_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "qrb/contract/geometry/v1/pose.pb.h"
#include "qrb/contract/geometry/v1/pose_with_covariance.pb.h"
#include "qrb/contract/geometry/v1/transform.pb.h"
#include "qrb/contract/geometry/v1/twist.pb.h"
#include "qrb/contract/geometry/v1/twist_with_covariance.pb.h"
#include "qrb/contract/geometry/v1/vector3.pb.h"

namespace qrb::adapter
{

/**
 * @brief Field-for-field copies between the ROS and Contract geometry messages,
 * shared by the Inline geometry converters.
 *
 * ROS to Contract always writes all 36 covariance values. Contract to ROS
 * expects a validated Contract: its covariance is empty (copied as the zero
 * matrix, "unknown") or holds the 36 values the ROS array holds.
 */
void PoseToContract(const geometry_msgs::msg::Pose & source,
    qrb::contract::geometry::v1::Pose & target);
void PoseToRos(const qrb::contract::geometry::v1::Pose & source, geometry_msgs::msg::Pose & target);
void Vector3ToContract(const geometry_msgs::msg::Vector3 & source,
    qrb::contract::geometry::v1::Vector3 & target);
void Vector3ToRos(const qrb::contract::geometry::v1::Vector3 & source,
    geometry_msgs::msg::Vector3 & target);
void PoseWithCovarianceToContract(const geometry_msgs::msg::PoseWithCovariance & source,
    qrb::contract::geometry::v1::PoseWithCovariance & target);
void PoseWithCovarianceToRos(const qrb::contract::geometry::v1::PoseWithCovariance & source,
    geometry_msgs::msg::PoseWithCovariance & target);
void TransformToContract(const geometry_msgs::msg::Transform & source,
    qrb::contract::geometry::v1::Transform & target);
void TransformToRos(const qrb::contract::geometry::v1::Transform & source,
    geometry_msgs::msg::Transform & target);
void TwistToContract(const geometry_msgs::msg::Twist & source,
    qrb::contract::geometry::v1::Twist & target);
void TwistToRos(const qrb::contract::geometry::v1::Twist & source,
    geometry_msgs::msg::Twist & target);
void TwistWithCovarianceToContract(const geometry_msgs::msg::TwistWithCovariance & source,
    qrb::contract::geometry::v1::TwistWithCovariance & target);
void TwistWithCovarianceToRos(const qrb::contract::geometry::v1::TwistWithCovariance & source,
    geometry_msgs::msg::TwistWithCovariance & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_GEOMETRY_HPP
