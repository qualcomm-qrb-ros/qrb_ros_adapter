// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"

#include <array>
#include <cstddef>

namespace qrb::adapter
{

namespace
{

template <typename Repeated>
void CovarianceToContract(const std::array<double, 36> & source, Repeated & target)
{
  target.Clear();
  target.Reserve(static_cast<int>(source.size()));
  for (double value : source) {
    target.Add(value);
  }
}

template <typename Repeated>
void CovarianceToRos(const Repeated & source, std::array<double, 36> & target)
{
  target.fill(0.0);
  for (int i = 0; i < source.size() && i < static_cast<int>(target.size()); ++i) {
    target[static_cast<std::size_t>(i)] = source.Get(i);
  }
}

}  // namespace

void PoseWithCovarianceToContract(const geometry_msgs::msg::PoseWithCovariance & source,
    qrb::contract::geometry::v1::PoseWithCovariance & target)
{
  auto * position = target.mutable_pose()->mutable_position();
  position->set_x(source.pose.position.x);
  position->set_y(source.pose.position.y);
  position->set_z(source.pose.position.z);
  auto * orientation = target.mutable_pose()->mutable_orientation();
  orientation->set_x(source.pose.orientation.x);
  orientation->set_y(source.pose.orientation.y);
  orientation->set_z(source.pose.orientation.z);
  orientation->set_w(source.pose.orientation.w);
  CovarianceToContract(source.covariance, *target.mutable_covariance());
}

void PoseWithCovarianceToRos(const qrb::contract::geometry::v1::PoseWithCovariance & source,
    geometry_msgs::msg::PoseWithCovariance & target)
{
  const auto & position = source.pose().position();
  const auto & orientation = source.pose().orientation();
  target.pose.position.x = position.x();
  target.pose.position.y = position.y();
  target.pose.position.z = position.z();
  target.pose.orientation.x = orientation.x();
  target.pose.orientation.y = orientation.y();
  target.pose.orientation.z = orientation.z();
  target.pose.orientation.w = orientation.w();
  CovarianceToRos(source.covariance(), target.covariance);
}

void TwistWithCovarianceToContract(const geometry_msgs::msg::TwistWithCovariance & source,
    qrb::contract::geometry::v1::TwistWithCovariance & target)
{
  auto * linear = target.mutable_twist()->mutable_linear();
  linear->set_x(source.twist.linear.x);
  linear->set_y(source.twist.linear.y);
  linear->set_z(source.twist.linear.z);
  auto * angular = target.mutable_twist()->mutable_angular();
  angular->set_x(source.twist.angular.x);
  angular->set_y(source.twist.angular.y);
  angular->set_z(source.twist.angular.z);
  CovarianceToContract(source.covariance, *target.mutable_covariance());
}

void TwistWithCovarianceToRos(const qrb::contract::geometry::v1::TwistWithCovariance & source,
    geometry_msgs::msg::TwistWithCovariance & target)
{
  const auto & linear = source.twist().linear();
  const auto & angular = source.twist().angular();
  target.twist.linear.x = linear.x();
  target.twist.linear.y = linear.y();
  target.twist.linear.z = linear.z();
  target.twist.angular.x = angular.x();
  target.twist.angular.y = angular.y();
  target.twist.angular.z = angular.z();
  CovarianceToRos(source.covariance(), target.covariance);
}

}  // namespace qrb::adapter
