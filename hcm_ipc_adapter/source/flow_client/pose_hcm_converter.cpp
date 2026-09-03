// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"

namespace qrb::adapter
{

namespace geometry_proto = ::qrb::contract::geometry::v1;

template <>
ConversionResult ConvertContractToHcm<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmContract &pose, const DmaBufView & /*payload*/,
    PoseHcmDescriptor &descriptor )
{
    ConversionResult validation = ValidatePoseWithCovarianceStamped( pose );
    if ( !validation ) return validation;

    descriptor.timestampNs = pose.timestamp_ns();
    descriptor.frameId = pose.frame_id();

    const auto &value = pose.pose_with_covariance();
    descriptor.poseWithCovariance.pose.position.x = value.pose().position().x();
    descriptor.poseWithCovariance.pose.position.y = value.pose().position().y();
    descriptor.poseWithCovariance.pose.position.z = value.pose().position().z();
    descriptor.poseWithCovariance.pose.orientation.x = value.pose().orientation().x();
    descriptor.poseWithCovariance.pose.orientation.y = value.pose().orientation().y();
    descriptor.poseWithCovariance.pose.orientation.z = value.pose().orientation().z();
    descriptor.poseWithCovariance.pose.orientation.w = value.pose().orientation().w();
    descriptor.poseWithCovariance.covariance.fill( 0.0 );
    for ( int i = 0; i < value.covariance_size(); ++i )
    {
        descriptor.poseWithCovariance.covariance[static_cast<std::size_t>( i )] =
            value.covariance( i );
    }
    return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmDescriptor &descriptor, PoseHcmContract &pose )
{
    pose.Clear();
    pose.set_timestamp_ns( descriptor.timestampNs );
    pose.set_frame_id( descriptor.frameId );
    auto *value = pose.mutable_pose_with_covariance();
    auto *position = value->mutable_pose()->mutable_position();
    position->set_x( descriptor.poseWithCovariance.pose.position.x );
    position->set_y( descriptor.poseWithCovariance.pose.position.y );
    position->set_z( descriptor.poseWithCovariance.pose.position.z );
    auto *orientation = value->mutable_pose()->mutable_orientation();
    orientation->set_x( descriptor.poseWithCovariance.pose.orientation.x );
    orientation->set_y( descriptor.poseWithCovariance.pose.orientation.y );
    orientation->set_z( descriptor.poseWithCovariance.pose.orientation.z );
    orientation->set_w( descriptor.poseWithCovariance.pose.orientation.w );
    for ( double covariance : descriptor.poseWithCovariance.covariance )
    {
        value->add_covariance( covariance );
    }

    return ValidatePoseWithCovarianceStamped( pose );
}

}   // namespace qrb::adapter
