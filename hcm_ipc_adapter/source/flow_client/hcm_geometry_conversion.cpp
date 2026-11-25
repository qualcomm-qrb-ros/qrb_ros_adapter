// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <array>
#include <cstddef>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

namespace
{

template <typename Repeated>
void CovarianceToHcm(const Repeated & source, std::array<double, 36> & target)
{
  target.fill(0.0);
  for (int i = 0; i < source.size() && i < static_cast<int>(target.size()); ++i) {
    target[static_cast<std::size_t>(i)] = source.Get(i);
  }
}

template <typename Repeated>
void CovarianceToContract(const std::array<double, 36> & source, Repeated & target)
{
  target.Clear();
  target.Reserve(static_cast<int>(source.size()));
  for (double value : source) {
    target.Add(value);
  }
}

}  // namespace

void PoseToHcm(const ::qrb::contract::geometry::v1::Pose & source, QRB::Memory::Pose_t & target)
{
  const auto & position = source.position();
  const auto & orientation = source.orientation();
  target.position.x = position.x();
  target.position.y = position.y();
  target.position.z = position.z();
  target.orientation.x = orientation.x();
  target.orientation.y = orientation.y();
  target.orientation.z = orientation.z();
  target.orientation.w = orientation.w();
}

void PoseToContract(const QRB::Memory::Pose_t & source,
    ::qrb::contract::geometry::v1::Pose & target)
{
  auto * position = target.mutable_position();
  position->set_x(source.position.x);
  position->set_y(source.position.y);
  position->set_z(source.position.z);
  auto * orientation = target.mutable_orientation();
  orientation->set_x(source.orientation.x);
  orientation->set_y(source.orientation.y);
  orientation->set_z(source.orientation.z);
  orientation->set_w(source.orientation.w);
}

void Vector3ToHcm(const ::qrb::contract::geometry::v1::Vector3 & source,
    QRB::Memory::Vector3d_t & target)
{
  target.x = source.x();
  target.y = source.y();
  target.z = source.z();
}

void Vector3ToContract(const QRB::Memory::Vector3d_t & source,
    ::qrb::contract::geometry::v1::Vector3 & target)
{
  target.set_x(source.x);
  target.set_y(source.y);
  target.set_z(source.z);
}

void PoseWithCovarianceToHcm(const ::qrb::contract::geometry::v1::PoseWithCovariance & source,
    QRB::Memory::PoseWithCovariance_t & target)
{
  PoseToHcm(source.pose(), target.pose);
  CovarianceToHcm(source.covariance(), target.covariance);
}

void PoseWithCovarianceToContract(const QRB::Memory::PoseWithCovariance_t & source,
    ::qrb::contract::geometry::v1::PoseWithCovariance & target)
{
  PoseToContract(source.pose, *target.mutable_pose());
  CovarianceToContract(source.covariance, *target.mutable_covariance());
}

void TransformToHcm(const ::qrb::contract::geometry::v1::Transform & source,
    QRB::Memory::Transform_t & target)
{
  const auto & translation = source.translation();
  const auto & rotation = source.rotation();
  target.translation.x = translation.x();
  target.translation.y = translation.y();
  target.translation.z = translation.z();
  target.rotation.x = rotation.x();
  target.rotation.y = rotation.y();
  target.rotation.z = rotation.z();
  target.rotation.w = rotation.w();
}

void TransformToContract(const QRB::Memory::Transform_t & source,
    ::qrb::contract::geometry::v1::Transform & target)
{
  auto * translation = target.mutable_translation();
  translation->set_x(source.translation.x);
  translation->set_y(source.translation.y);
  translation->set_z(source.translation.z);
  auto * rotation = target.mutable_rotation();
  rotation->set_x(source.rotation.x);
  rotation->set_y(source.rotation.y);
  rotation->set_z(source.rotation.z);
  rotation->set_w(source.rotation.w);
}

void TwistToHcm(const ::qrb::contract::geometry::v1::Twist & source, QRB::Memory::Twist_t & target)
{
  target.linear.x = source.linear().x();
  target.linear.y = source.linear().y();
  target.linear.z = source.linear().z();
  target.angular.x = source.angular().x();
  target.angular.y = source.angular().y();
  target.angular.z = source.angular().z();
}

void TwistToContract(const QRB::Memory::Twist_t & source,
    ::qrb::contract::geometry::v1::Twist & target)
{
  auto * linear = target.mutable_linear();
  linear->set_x(source.linear.x);
  linear->set_y(source.linear.y);
  linear->set_z(source.linear.z);
  auto * angular = target.mutable_angular();
  angular->set_x(source.angular.x);
  angular->set_y(source.angular.y);
  angular->set_z(source.angular.z);
}

void TwistWithCovarianceToHcm(const ::qrb::contract::geometry::v1::TwistWithCovariance & source,
    QRB::Memory::TwistWithCovariance_t & target)
{
  TwistToHcm(source.twist(), target.twist);
  CovarianceToHcm(source.covariance(), target.covariance);
}

void TwistWithCovarianceToContract(const QRB::Memory::TwistWithCovariance_t & source,
    ::qrb::contract::geometry::v1::TwistWithCovariance & target)
{
  TwistToContract(source.twist, *target.mutable_twist());
  CovarianceToContract(source.covariance, *target.mutable_covariance());
}

}  // namespace qrb::adapter
