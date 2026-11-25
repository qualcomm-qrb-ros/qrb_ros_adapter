// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_VECTOR3_STAMPED_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_VECTOR3_STAMPED_HCM_CONVERTER_HPP

#include "QRB/Memory/Geometry/Vector3Descriptor.hpp"
#include "qrb/contract/geometry/v1/vector3_stamped.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using Vector3StampedHcmContract = ::qrb::contract::geometry::v1::Vector3Stamped;
using Vector3StampedHcmDescriptor = QRB::Memory::Vector3Descriptor_t;

template <>
ConversionResult ConvertContractToHcm<Vector3StampedHcmContract, Vector3StampedHcmDescriptor>(
    const Vector3StampedHcmContract & source,
    const DmaBufView & payload,
    Vector3StampedHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<Vector3StampedHcmContract, Vector3StampedHcmDescriptor>(
    const Vector3StampedHcmDescriptor & source,
    Vector3StampedHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_VECTOR3_STAMPED_HCM_CONVERTER_HPP
