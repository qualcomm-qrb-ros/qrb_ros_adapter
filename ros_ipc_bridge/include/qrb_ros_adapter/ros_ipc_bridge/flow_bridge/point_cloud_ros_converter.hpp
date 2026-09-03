// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POINT_CLOUD_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POINT_CLOUD_ROS_CONVERTER_HPP

#include <cstdint>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "qrb/contract/sensor/v1/point_cloud2.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"
#include "qrb_ros_transport_point_cloud2_type/point_cloud2.hpp"

namespace qrb::adapter
{

using PointFieldRosMessage = sensor_msgs::msg::PointField;
using PointFieldContract = ::qrb::contract::sensor::v1::PointField;
using PointCloudRosMessage = qrb_ros::transport::type::PointCloud2;
using PointCloudRosContract = ::qrb::contract::sensor::v1::PointCloud2;
using QrbPointCloudType = PointCloudRosMessage;
using PointCloudContract = PointCloudRosContract;

template <>
struct RosConversion<PointCloudRosMessage, PointCloudRosContract>
{
  using RosType = PointCloudRosMessage;
  using ContractType = PointCloudRosContract;

  static ConversionResult RosToContract(const RosType & source,
      ContractType & target,
      RosToContractPayload & payload);
  static ConversionResult ContractToRos(const ContractType & source,
      RosType & target,
      ContractToRosPayload & payload);
};

using PointCloudRosConversion = RosConversion<PointCloudRosMessage, PointCloudRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POINT_CLOUD_ROS_CONVERTER_HPP
