// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "QC/Common/QCString.hpp"
#include "QRB/Memory/Common/Types.hpp"
#include "qrb/contract/geometry/v1/pose.pb.h"
#include "qrb/contract/geometry/v1/pose_with_covariance.pb.h"
#include "qrb/contract/geometry/v1/transform.pb.h"
#include "qrb/contract/geometry/v1/twist.pb.h"
#include "qrb/contract/geometry/v1/twist_with_covariance.pb.h"
#include "qrb/contract/geometry/v1/vector3.pb.h"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

/** @brief Read-only mapping of a received DMA-BUF. */
struct DmaBufView
{
  const void * data{nullptr};
  std::size_t size{0};
};

/** @brief Copy a Contract string into a fixed-size descriptor string; fails if it does not fit. */
template <std::size_t N>
[[nodiscard]] bool AssignHcmString(const std::string & source, QC::QCString<N> & target)
{
  return source.size() <= N && target.Assign(source.data(),
                                   static_cast<std::uint32_t>(source.size())) == QC::QC_STATUS_OK;
}

/** @brief Read a fixed-size descriptor string back as a Contract string. */
template <std::size_t N>
std::string HcmStringToContract(const QC::QCString<N> & source)
{
  return std::string(source.Data(), source.Size());
}

/**
 * @brief Field-for-field copies between the Contract geometry messages and the
 * qrb_contracts types, shared by the Inline geometry converters.
 *
 * The Contract side must already be validated: its covariance is empty (copied
 * as the zero matrix, "unknown") or holds the 36 values the array holds. The
 * HCM to Contract direction always writes all 36 values.
 */
void PoseToHcm(const ::qrb::contract::geometry::v1::Pose & source, QRB::Memory::Pose_t & target);
void PoseToContract(const QRB::Memory::Pose_t & source,
    ::qrb::contract::geometry::v1::Pose & target);
void Vector3ToHcm(const ::qrb::contract::geometry::v1::Vector3 & source,
    QRB::Memory::Vector3d_t & target);
void Vector3ToContract(const QRB::Memory::Vector3d_t & source,
    ::qrb::contract::geometry::v1::Vector3 & target);
void PoseWithCovarianceToHcm(const ::qrb::contract::geometry::v1::PoseWithCovariance & source,
    QRB::Memory::PoseWithCovariance_t & target);
void PoseWithCovarianceToContract(const QRB::Memory::PoseWithCovariance_t & source,
    ::qrb::contract::geometry::v1::PoseWithCovariance & target);
void TransformToHcm(const ::qrb::contract::geometry::v1::Transform & source,
    QRB::Memory::Transform_t & target);
void TransformToContract(const QRB::Memory::Transform_t & source,
    ::qrb::contract::geometry::v1::Transform & target);
void TwistToHcm(const ::qrb::contract::geometry::v1::Twist & source, QRB::Memory::Twist_t & target);
void TwistToContract(const QRB::Memory::Twist_t & source,
    ::qrb::contract::geometry::v1::Twist & target);
void TwistWithCovarianceToHcm(const ::qrb::contract::geometry::v1::TwistWithCovariance & source,
    QRB::Memory::TwistWithCovariance_t & target);
void TwistWithCovarianceToContract(const QRB::Memory::TwistWithCovariance_t & source,
    ::qrb::contract::geometry::v1::TwistWithCovariance & target);

/** @brief Convert a Contract and optional external payload to an HCM descriptor. */
template <typename ContractT, typename DescriptorT>
ConversionResult ConvertContractToHcm(const ContractT & source,
    const DmaBufView & payload,
    DescriptorT & target) = delete;

/** @brief Convert an HCM descriptor to its transport Contract. */
template <typename ContractT, typename DescriptorT>
ConversionResult ConvertHcmToContract(const DescriptorT & source, ContractT & target) = delete;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP
