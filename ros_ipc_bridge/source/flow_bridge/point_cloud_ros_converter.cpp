// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/point_cloud_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_time.hpp"

#include <limits>
#include <utility>

namespace qrb::adapter
{

namespace
{

using PointFieldContractType = ::qrb::contract::sensor::v1::PointFieldType;

struct PointFieldTypeMapping
{
    PointFieldContractType contract;
    std::uint8_t ros;
};

constexpr PointFieldTypeMapping kPointFieldTypes[] = {
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_INT8, sensor_msgs::msg::PointField::INT8 },
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_UINT8, sensor_msgs::msg::PointField::UINT8 },
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_INT16, sensor_msgs::msg::PointField::INT16 },
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_UINT16, sensor_msgs::msg::PointField::UINT16 },
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_INT32, sensor_msgs::msg::PointField::INT32 },
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_UINT32, sensor_msgs::msg::PointField::UINT32 },
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_FLOAT32,
        sensor_msgs::msg::PointField::FLOAT32 },
    { ::qrb::contract::sensor::v1::POINT_FIELD_TYPE_FLOAT64,
        sensor_msgs::msg::PointField::FLOAT64 },
};

ConversionResult ConvertPointFieldToContract(
    const PointFieldRosMessage &source, PointFieldContract &target )
{
    PointFieldContractType datatype{};
    bool found = false;
    for ( const PointFieldTypeMapping &mapping : kPointFieldTypes )
    {
        if ( mapping.ros == source.datatype )
        {
            datatype = mapping.contract;
            found = true;
            break;
        }
    }
    if ( !found )
    {
        return ConversionResult::Failure(
            "ROS datatype " + std::to_string( static_cast<int>( source.datatype ) )
            + " is not supported" );
    }

    target.set_name( source.name );
    target.set_offset( source.offset );
    target.set_datatype( datatype );
    target.set_count( source.count );
    return ConversionResult::Success();
}

ConversionResult ConvertPointFieldToRos(
    const PointFieldContract &source, PointFieldRosMessage &target )
{
    std::uint8_t datatype = 0;
    bool found = false;
    for ( const PointFieldTypeMapping &mapping : kPointFieldTypes )
    {
        if ( mapping.contract == source.datatype() )
        {
            datatype = mapping.ros;
            found = true;
            break;
        }
    }
    if ( !found )
    {
        return ConversionResult::Failure( "Contract datatype is not supported" );
    }

    target.name = source.name();
    target.offset = source.offset();
    target.datatype = datatype;
    target.count = source.count();
    return ConversionResult::Success();
}

ConversionResult GetPointCloudPayloadSize(
    const PointCloudRosContract &source, std::uint64_t &size )
{
    if ( source.height() != 0
        && source.row_step() > std::numeric_limits<std::uint64_t>::max() / source.height() )
    {
        return ConversionResult::Failure( "row_step * height overflows" );
    }

    size = static_cast<std::uint64_t>( source.row_step() ) * source.height();
    if ( size == 0 )
    {
        return ConversionResult::Failure( "DMA-BUF point cloud payload is empty" );
    }
    return ConversionResult::Success();
}

ConversionResult FillContractMetadata( const std_msgs::msg::Header &header,
    std::uint32_t height, std::uint32_t width, bool isBigendian,
    std::uint32_t pointStep, std::uint32_t rowStep, bool isDense,
    const std::vector<PointFieldRosMessage> &fields, PointCloudRosContract &target,
    std::uint64_t &payloadSize )
{
    std::uint64_t timestampNs = 0;
    ConversionResult time = StampToContractNanos( header.stamp, timestampNs );
    if ( !time )
    {
        return ConversionResult::Failure(
            "header.stamp cannot be represented as timestamp_ns: " + time.Diagnostic() );
    }

    target.Clear();
    target.set_timestamp_ns( timestampNs );
    target.set_frame_id( header.frame_id );
    target.set_height( height );
    target.set_width( width );
    target.set_is_bigendian( isBigendian );
    target.set_point_step( pointStep );
    target.set_row_step( rowStep );
    target.set_is_dense( isDense );

    for ( const PointFieldRosMessage &field : fields )
    {
        PointFieldContract convertedField;
        ConversionResult converted = ConvertPointFieldToContract( field, convertedField );
        if ( !converted )
        {
            return ConversionResult::Failure(
                "point field '" + field.name + "': " + converted.Diagnostic() );
        }
        *target.add_fields() = std::move( convertedField );
    }

    ConversionResult sized = GetPointCloudPayloadSize( target, payloadSize );
    if ( !sized ) return sized;
    return ValidatePointCloud2( target, payloadSize );
}

ConversionResult FillRosMetadata( const PointCloudRosContract &source,
    std_msgs::msg::Header &header, std::uint32_t &height, std::uint32_t &width,
    bool &isBigendian, std::uint32_t &pointStep, std::uint32_t &rowStep,
    bool &isDense, std::vector<PointFieldRosMessage> &fields )
{
    ConversionResult time = ContractNanosToStamp( source.timestamp_ns(), header.stamp );
    if ( !time )
    {
        return ConversionResult::Failure(
            "timestamp_ns cannot be represented as a ROS stamp: " + time.Diagnostic() );
    }
    header.frame_id = source.frame_id();
    height = source.height();
    width = source.width();
    isBigendian = source.is_bigendian();
    pointStep = source.point_step();
    rowStep = source.row_step();
    isDense = source.is_dense();

    fields.clear();
    fields.reserve( static_cast<std::size_t>( source.fields_size() ) );
    for ( const PointFieldContract &field : source.fields() )
    {
        PointFieldRosMessage convertedField;
        ConversionResult converted = ConvertPointFieldToRos( field, convertedField );
        if ( !converted )
        {
            return ConversionResult::Failure(
                "point field '" + field.name() + "': " + converted.Diagnostic() );
        }
        fields.push_back( std::move( convertedField ) );
    }
    return ConversionResult::Success();
}

}   // namespace

ConversionResult PointCloudRosConversion::RosToContract(
    const PointCloudRosMessage &source, PointCloudRosContract &target,
    RosToContractPayload &payload )
{
    std::uint64_t payloadSize = 0;
    ConversionResult converted = FillContractMetadata( source.header, source.height, source.width,
        source.is_bigendian, source.point_step, source.row_step, source.is_dense,
        source.fields, target, payloadSize );
    if ( !converted ) return converted;
    return payload.Bind( source.data_fd, payloadSize );
}

ConversionResult PointCloudRosConversion::ContractToRos(
    const PointCloudRosContract &source, PointCloudRosMessage &target,
    ContractToRosPayload &payload )
{
    std::uint64_t expectedSize = 0;
    ConversionResult sized = GetPointCloudPayloadSize( source, expectedSize );
    if ( !sized ) return sized;

    ConversionResult validation = ValidatePointCloud2( source, expectedSize );
    if ( !validation ) return validation;

    ConversionResult converted = FillRosMetadata( source, target.header, target.height, target.width,
        target.is_bigendian, target.point_step, target.row_step, target.is_dense,
        target.fields );
    if ( !converted ) return converted;
    return payload.Bind( target.data_fd, expectedSize );
}

}   // namespace qrb::adapter
