// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_time.hpp"

namespace qrb::adapter
{

ConversionResult PoseRosConversion::RosToContract(
    const PoseRosMessage &source, PoseRosContract &target )
{
    std::uint64_t timestampNs = 0;
    ConversionResult time = StampToContractNanos( source.header.stamp, timestampNs );
    if ( !time )
    {
        return ConversionResult::Failure(
            "header.stamp cannot be represented as timestamp_ns: " + time.Diagnostic() );
    }

    target.Clear();
    target.set_timestamp_ns( timestampNs );
    target.set_frame_id( source.header.frame_id );
    auto *value = target.mutable_pose_with_covariance();
    auto *position = value->mutable_pose()->mutable_position();
    position->set_x( source.pose.pose.position.x );
    position->set_y( source.pose.pose.position.y );
    position->set_z( source.pose.pose.position.z );
    auto *orientation = value->mutable_pose()->mutable_orientation();
    orientation->set_x( source.pose.pose.orientation.x );
    orientation->set_y( source.pose.pose.orientation.y );
    orientation->set_z( source.pose.pose.orientation.z );
    orientation->set_w( source.pose.pose.orientation.w );
    for ( double covariance : source.pose.covariance ) value->add_covariance( covariance );

    return ValidatePoseWithCovarianceStamped( target );
}

ConversionResult PoseRosConversion::ContractToRos(
    const PoseRosContract &source, PoseRosMessage &target )
{
    ConversionResult validation = ValidatePoseWithCovarianceStamped( source );
    if ( !validation ) return validation;

    ConversionResult time = ContractNanosToStamp( source.timestamp_ns(), target.header.stamp );
    if ( !time )
    {
        return ConversionResult::Failure(
            "timestamp_ns cannot be represented as a ROS stamp: " + time.Diagnostic() );
    }
    target.header.frame_id = source.frame_id();

    const auto &value = source.pose_with_covariance();
    target.pose.pose.position.x = value.pose().position().x();
    target.pose.pose.position.y = value.pose().position().y();
    target.pose.pose.position.z = value.pose().position().z();
    target.pose.pose.orientation.x = value.pose().orientation().x();
    target.pose.pose.orientation.y = value.pose().orientation().y();
    target.pose.pose.orientation.z = value.pose().orientation().z();
    target.pose.pose.orientation.w = value.pose().orientation().w();
    target.pose.covariance.fill( 0.0 );
    for ( int i = 0; i < value.covariance_size(); ++i )
    {
        target.pose.covariance[static_cast<std::size_t>( i )] = value.covariance( i );
    }
    return ConversionResult::Success();
}

}   // namespace qrb::adapter
