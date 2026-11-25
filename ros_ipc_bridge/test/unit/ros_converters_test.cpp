// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

// Converters are stateless functions over message structures: no ROS Context,
// no node, no DMA heap. A memfd stands in for a DMA-BUF where only fd identity
// and size matter.

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <limits>
#include <memory>
#include <sensor_msgs/msg/point_field.hpp>

#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/joint_state_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/odometry_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/point_cloud_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_array_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_stamped_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/tf_message_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/twist_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/vector3_stamped_ros_converter.hpp"

using namespace qrb::adapter;

namespace sensor_proto = ::qrb::contract::sensor::v1;

namespace
{

std::shared_ptr<lib_mem_dmabuf::DmaBuffer> MakeMemfdBuffer(std::size_t size)
{
  const int fd = ::memfd_create("qrb_ros_adapter_converter_test", MFD_CLOEXEC);
  if (fd < 0)
    return nullptr;
  if (::ftruncate(fd, static_cast<off_t>(size)) != 0) {
    ::close(fd);
    return nullptr;
  }
  return std::make_shared<lib_mem_dmabuf::DmaBuffer>(fd, size);
}

sensor_msgs::msg::PointField Field(const char * name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

constexpr std::uint32_t kPointStep = 32;
constexpr std::uint32_t kPoints = 4;
constexpr std::uint32_t kCloudBytes = kPointStep * kPoints;

/** @brief One point of every PointField datatype, packed into 32 bytes. */
PointCloudRosMessage MakeRosCloud(std::size_t bufferSize = kCloudBytes)
{
  using PF = sensor_msgs::msg::PointField;
  // The transport type leaves its scalars uninitialized; an indeterminate
  // bool serializes as a malformed varint.
  auto cloud = PointCloudRosMessage();
  cloud.header.stamp.sec = 100;
  cloud.header.stamp.nanosec = 500;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = kPoints;
  cloud.point_step = kPointStep;
  cloud.row_step = kCloudBytes;
  cloud.is_dense = true;
  cloud.fields = {Field("i8", 0, PF::INT8), Field("u8", 1, PF::UINT8), Field("i16", 2, PF::INT16),
      Field("u16", 4, PF::UINT16), Field("i32", 8, PF::INT32), Field("u32", 12, PF::UINT32),
      Field("f32", 16, PF::FLOAT32), Field("f64", 24, PF::FLOAT64)};
  cloud.data_fd = MakeMemfdBuffer(bufferSize);
  return cloud;
}

}  // namespace

// PointCloud2 metadata and every PointField datatype map 1:1 and the
// payload context binds the DMA-BUF instead of copying bytes; a layout the
// payload cannot back is rejected, and a payload context binds once within
// offset + length <= capacity.
TEST(PointCloudRosConverter, MapsMetadataBindsTheDmaBufAndRejectsWhatItCannotBack)
{
  const PointCloudRosMessage source = MakeRosCloud(kCloudBytes + 64);
  ASSERT_NE(source.data_fd, nullptr);

  PointCloudRosContract contract;
  RosToContractPayload outbound;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(source, contract, outbound));
  EXPECT_EQ(contract.header().stamp().sec(), 100);
  EXPECT_EQ(contract.header().stamp().nanosec(), 500u);
  EXPECT_EQ(contract.header().frame_id(), "lidar");
  EXPECT_EQ(contract.row_step(), kCloudBytes);
  ASSERT_EQ(contract.fields_size(), 8);
  EXPECT_EQ(contract.fields(7).datatype(), sensor_proto::POINT_FIELD_TYPE_FLOAT64);
  // The valid range is the layout size, not the buffer capacity.
  EXPECT_EQ(outbound.Buffer(), source.data_fd);
  EXPECT_EQ(outbound.Size(), kCloudBytes);
  EXPECT_EQ(outbound.Offset(), 0u);

  auto back = PointCloudRosMessage();
  ContractToRosPayload inbound(kCloudBytes);
  ASSERT_TRUE(PointCloudRosConversion::ContractToRos(contract, back, inbound));
  EXPECT_EQ(inbound.Size(), kCloudBytes);
  EXPECT_EQ(back.data_fd, nullptr);  // the caller supplies the host storage buffer
  const auto destination = MakeMemfdBuffer(kCloudBytes);
  ASSERT_TRUE(inbound.Assign(destination));
  EXPECT_EQ(back.data_fd, destination);

  EXPECT_EQ(back.header, source.header);
  EXPECT_EQ(back.fields, source.fields);
  EXPECT_EQ(back.width, source.width);
  EXPECT_EQ(back.point_step, source.point_step);
  EXPECT_EQ(back.row_step, source.row_step);
  EXPECT_EQ(back.is_dense, source.is_dense);

  {
    PointCloudRosMessage invalid = MakeRosCloud();
    invalid.fields[0].datatype = 0;  // not a PointField datatype
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(invalid, contract, payload));
    EXPECT_FALSE(payload.IsBound());
  }
  {
    PointCloudRosMessage tooShort = MakeRosCloud(kCloudBytes - 1);
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(tooShort, contract, payload));
  }
  {
    PointCloudRosMessage withoutBuffer = MakeRosCloud();
    withoutBuffer.data_fd.reset();
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(withoutBuffer, contract, payload));
  }

  // HCM -> ROS: the received DMA-BUF range must cover the declared layout.
  RosToContractPayload valid;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(MakeRosCloud(), contract, valid));
  auto target = PointCloudRosMessage();
  ContractToRosPayload tooSmall(kCloudBytes - 1);
  EXPECT_FALSE(PointCloudRosConversion::ContractToRos(contract, target, tooSmall));
  EXPECT_FALSE(tooSmall.IsBound());

  // Source side: offset + length must fit the producer's buffer.
  const auto buffer = MakeMemfdBuffer(256);
  ASSERT_NE(buffer, nullptr);
  RosToContractPayload bounded;
  EXPECT_FALSE(bounded.Bind(nullptr, 16));
  EXPECT_FALSE(bounded.Bind(buffer, 129, 128));
  EXPECT_FALSE(bounded.IsBound());
  ASSERT_TRUE(bounded.Bind(buffer, 128, 128));
  EXPECT_EQ(bounded.Offset(), 128u);
  EXPECT_FALSE(bounded.Bind(buffer, 16));  // already bound
  EXPECT_EQ(bounded.Size(), 128u);

  // Destination side: the length must fit the received range.
  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> unassigned;
  ContractToRosPayload received(64);
  EXPECT_FALSE(received.Assign(MakeMemfdBuffer(64)));  // nothing bound yet
  EXPECT_FALSE(received.Bind(unassigned, 65));
  ASSERT_TRUE(received.Bind(unassigned, 64));
  EXPECT_FALSE(received.Bind(unassigned, 32));  // already bound
  EXPECT_FALSE(received.Assign(nullptr));
  EXPECT_EQ(unassigned, nullptr);
}

// nav_msgs/Odometry maps 1:1 to the Contract, both frames, pose and
// twist with their covariances; an invalid value is rejected in either direction.
TEST(OdometryRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  OdometryRosMessage ros;
  ros.header.stamp.sec = 5;
  ros.header.stamp.nanosec = 6;
  ros.header.frame_id = "odom";
  ros.child_frame_id = "base_link";
  ros.pose.pose.position.x = 1.0;
  ros.pose.pose.orientation.w = 1.0;
  ros.twist.twist.linear.x = 0.3;
  ros.twist.twist.angular.z = -0.1;
  for (std::size_t i = 0; i < 36; ++i) {
    ros.pose.covariance[i] = 0.01 * static_cast<double>(i);
    ros.twist.covariance[i] = 0.02 * static_cast<double>(i);
  }

  OdometryRosContract contract;
  ASSERT_TRUE(OdometryRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "odom");
  EXPECT_EQ(contract.child_frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 6u);
  EXPECT_DOUBLE_EQ(contract.twist().twist().linear().x(), 0.3);
  ASSERT_EQ(contract.twist().covariance_size(), 36);
  EXPECT_DOUBLE_EQ(contract.twist().covariance(35), 0.70);

  OdometryRosMessage back;
  ASSERT_TRUE(OdometryRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // An empty Contract covariance means unknown; ROS gets the zero matrix.
  contract.mutable_twist()->clear_covariance();
  ASSERT_TRUE(OdometryRosConversion::ContractToRos(contract, back));
  EXPECT_DOUBLE_EQ(back.twist.covariance[35], 0.0);

  OdometryRosMessage invalid = ros;
  invalid.twist.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(OdometryRosConversion::RosToContract(invalid, contract));
  OdometryRosContract badContract;
  ASSERT_TRUE(OdometryRosConversion::RosToContract(ros, badContract));
  badContract.set_child_frame_id("/base_link");
  EXPECT_FALSE(OdometryRosConversion::ContractToRos(badContract, back));
}

// geometry_msgs/TwistStamped maps 1:1 to the Contract; an invalid
// value is rejected in either direction.
TEST(TwistRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  TwistRosMessage ros;
  ros.header.stamp.sec = 7;
  ros.header.stamp.nanosec = 8;
  ros.header.frame_id = "base_link";
  ros.twist.linear.x = 0.5;
  ros.twist.linear.y = -0.2;
  ros.twist.angular.z = 0.25;

  TwistRosContract contract;
  ASSERT_TRUE(TwistRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 8u);
  EXPECT_DOUBLE_EQ(contract.twist().linear().y(), -0.2);
  EXPECT_DOUBLE_EQ(contract.twist().angular().z(), 0.25);

  TwistRosMessage back;
  ASSERT_TRUE(TwistRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  TwistRosMessage invalid = ros;
  invalid.twist.linear.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(TwistRosConversion::RosToContract(invalid, contract));
  TwistRosContract badContract;
  ASSERT_TRUE(TwistRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(TwistRosConversion::ContractToRos(badContract, back));
}

// tf2_msgs/TFMessage maps every transform 1:1 to the Contract; an
// invalid transform is rejected in either direction.
TEST(TFMessageRosConverter, MapsEveryTransformBothWaysAndRejectsInvalidValues)
{
  TFMessageRosMessage ros;
  ros.transforms.resize(2);
  for (std::size_t i = 0; i < ros.transforms.size(); ++i) {
    auto & transform = ros.transforms[i];
    transform.header.stamp.sec = static_cast<std::int32_t>(10 + i);
    transform.header.stamp.nanosec = static_cast<std::uint32_t>(3 * i);
    transform.header.frame_id = i == 0 ? "map" : "odom";
    transform.child_frame_id = i == 0 ? "odom" : "base_link";
    transform.transform.translation.x = 0.5 * static_cast<double>(i);
    transform.transform.rotation.z = 0.6;
    transform.transform.rotation.w = 0.8;
  }

  TFMessageRosContract contract;
  ASSERT_TRUE(TFMessageRosConversion::RosToContract(ros, contract));
  ASSERT_EQ(contract.transforms_size(), 2);
  EXPECT_EQ(contract.transforms(1).child_frame_id(), "base_link");
  EXPECT_EQ(contract.transforms(1).header().stamp().nanosec(), 3u);
  EXPECT_DOUBLE_EQ(contract.transforms(1).transform().translation().x(), 0.5);

  TFMessageRosMessage back;
  back.transforms.resize(5);  // stale
  ASSERT_TRUE(TFMessageRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  TFMessageRosMessage invalid = ros;
  invalid.transforms[1].transform.rotation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TFMessageRosConversion::RosToContract(invalid, contract));
  TFMessageRosContract badContract;
  ASSERT_TRUE(TFMessageRosConversion::RosToContract(ros, badContract));
  badContract.mutable_transforms(0)->mutable_header()->set_frame_id("/map");
  EXPECT_FALSE(TFMessageRosConversion::ContractToRos(badContract, back));
}

// UT-ROS-05: sensor_msgs/JointState maps 1:1 to the Contract; an empty
// measurement array stays empty, and malformed array lengths or invalid values
// are rejected in either direction.
TEST(JointStateRosConverter, MapsEveryFieldBothWaysAndRejectsMalformedMeasurements)
{
  JointStateRosMessage ros;
  ros.header.stamp.sec = 5;
  ros.header.stamp.nanosec = 6;
  ros.header.frame_id = "stream_probe";
  ros.name = {"joint_0", "joint_1", "joint_2"};
  ros.position = {0.1, 0.2, 0.3};
  ros.velocity = {1.0, 2.0, 3.0};
  ros.effort = {0.0, 0.0, 0.0};

  JointStateRosContract contract;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "stream_probe");
  EXPECT_EQ(contract.header().stamp().nanosec(), 6u);
  ASSERT_EQ(contract.name_size(), 3);
  EXPECT_EQ(contract.name(2), "joint_2");
  ASSERT_EQ(contract.position_size(), 3);
  EXPECT_DOUBLE_EQ(contract.position(1), 0.2);
  EXPECT_DOUBLE_EQ(contract.velocity(2), 3.0);
  EXPECT_EQ(contract.effort_size(), 3);

  JointStateRosMessage back;
  back.name.resize(5);  // stale
  ASSERT_TRUE(JointStateRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // A measurement that is not supplied stays empty and is never zero-filled.
  JointStateRosMessage positionOnly = ros;
  positionOnly.velocity.clear();
  positionOnly.effort.clear();
  JointStateRosContract partial;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(positionOnly, partial));
  EXPECT_EQ(partial.velocity_size(), 0);
  EXPECT_EQ(partial.effort_size(), 0);
  JointStateRosMessage partialBack;
  ASSERT_TRUE(JointStateRosConversion::ContractToRos(partial, partialBack));
  EXPECT_EQ(partialBack, positionOnly);

  // A measurement array that is neither empty nor as long as name.
  JointStateRosMessage shortPosition = ros;
  shortPosition.position.pop_back();
  EXPECT_FALSE(JointStateRosConversion::RosToContract(shortPosition, contract));
  JointStateRosMessage longEffort = ros;
  longEffort.effort.push_back(0.0);
  EXPECT_FALSE(JointStateRosConversion::RosToContract(longEffort, contract));

  JointStateRosMessage invalid = ros;
  invalid.velocity[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(JointStateRosConversion::RosToContract(invalid, contract));
  JointStateRosMessage emptyName = ros;
  emptyName.name[0].clear();
  EXPECT_FALSE(JointStateRosConversion::RosToContract(emptyName, contract));

  JointStateRosContract badContract;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/stream_probe");
  EXPECT_FALSE(JointStateRosConversion::ContractToRos(badContract, back));
  JointStateRosContract mismatched;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(ros, mismatched));
  mismatched.mutable_position()->RemoveLast();
  EXPECT_FALSE(JointStateRosConversion::ContractToRos(mismatched, back));
}

// UT-ROS-05: geometry_msgs/PoseStamped maps 1:1 to the Contract; an invalid value is rejected in
// either direction.
TEST(PoseStampedRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  PoseStampedRosMessage ros;
  ros.header.stamp.sec = 9;
  ros.header.stamp.nanosec = 10;
  ros.header.frame_id = "base_link";
  ros.pose.position.x = 1.5;
  ros.pose.position.y = -2.0;
  ros.pose.position.z = 0.25;
  ros.pose.orientation.z = 0.6;
  ros.pose.orientation.w = 0.8;

  PoseStampedRosContract contract;
  ASSERT_TRUE(PoseStampedRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().sec(), 9);
  EXPECT_EQ(contract.header().stamp().nanosec(), 10u);
  EXPECT_DOUBLE_EQ(contract.pose().position().y(), -2.0);
  EXPECT_DOUBLE_EQ(contract.pose().orientation().z(), 0.6);
  EXPECT_DOUBLE_EQ(contract.pose().orientation().w(), 0.8);

  PoseStampedRosMessage back;
  back.header.frame_id = "stale";
  back.pose.position.x = 99.0;
  ASSERT_TRUE(PoseStampedRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  PoseStampedRosMessage invalid = ros;
  invalid.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PoseStampedRosConversion::RosToContract(invalid, contract));
  PoseStampedRosMessage zero = ros;
  zero.pose.orientation.z = 0.0;
  zero.pose.orientation.w = 0.0;
  EXPECT_FALSE(PoseStampedRosConversion::RosToContract(zero, contract));

  // A rejected Contract leaves the ROS message as it was.
  PoseStampedRosContract badContract;
  ASSERT_TRUE(PoseStampedRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(PoseStampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  badContract.mutable_header()->set_frame_id("base_link");
  badContract.clear_pose();
  EXPECT_FALSE(PoseStampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  // A pose with no orientation reads as the zero quaternion.
  badContract.mutable_pose()->mutable_position()->set_x(1.0);
  EXPECT_FALSE(PoseStampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
}

// UT-ROS-06: geometry_msgs/PoseArray maps every pose 1:1 to the Contract, an empty array
// included; an invalid pose, or more poses than a Contract holds, is rejected in either direction.
TEST(PoseArrayRosConverter, MapsEveryPoseBothWaysAndRejectsInvalidValues)
{
  PoseArrayRosMessage ros;
  ros.header.stamp.sec = 11;
  ros.header.stamp.nanosec = 12;
  ros.header.frame_id = "base_link";
  ros.poses.resize(3);
  for (std::size_t i = 0; i < ros.poses.size(); ++i) {
    auto & pose = ros.poses[i];
    pose.position.x = 0.5 * static_cast<double>(i);
    pose.position.z = -0.25;
    pose.orientation.z = 0.6;
    pose.orientation.w = 0.8;
  }

  PoseArrayRosContract contract;
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(ros, contract));
  ASSERT_EQ(contract.poses_size(), 3);
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 12u);
  EXPECT_DOUBLE_EQ(contract.poses(2).position().x(), 1.0);
  EXPECT_DOUBLE_EQ(contract.poses(2).position().z(), -0.25);
  EXPECT_DOUBLE_EQ(contract.poses(2).orientation().w(), 0.8);

  PoseArrayRosMessage back;
  back.header.frame_id = "stale";
  back.poses.resize(5);  // stale
  ASSERT_TRUE(PoseArrayRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // An empty array is a result: it keeps its header and replaces the poses of a stale message.
  PoseArrayRosMessage empty;
  empty.header.stamp.sec = 3;
  empty.header.frame_id = "base_link";
  PoseArrayRosContract emptyContract;
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(empty, emptyContract));
  EXPECT_EQ(emptyContract.poses_size(), 0);
  EXPECT_EQ(emptyContract.header().frame_id(), "base_link");
  ASSERT_TRUE(PoseArrayRosConversion::ContractToRos(emptyContract, back));
  EXPECT_EQ(back, empty);

  PoseArrayRosMessage invalid = ros;
  invalid.poses[1].position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PoseArrayRosConversion::RosToContract(invalid, contract));
  PoseArrayRosMessage zero = ros;
  zero.poses[2].orientation.z = 0.0;
  zero.poses[2].orientation.w = 0.0;
  EXPECT_FALSE(PoseArrayRosConversion::RosToContract(zero, contract));

  // A rejected Contract leaves the ROS message as it was.
  back = ros;
  PoseArrayRosContract badContract;
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(ros, badContract));
  badContract.mutable_poses(0)->mutable_orientation()->set_z(0.0);
  badContract.mutable_poses(0)->mutable_orientation()->set_w(0.0);
  EXPECT_FALSE(PoseArrayRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(PoseArrayRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);

  // As many poses as a Contract may hold go through; one more is rejected both ways.
  PoseArrayRosMessage full;
  full.poses.resize(contract_limits::kMaxPoseCount);
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(full, contract));
  ASSERT_TRUE(PoseArrayRosConversion::ContractToRos(contract, back));
  EXPECT_TRUE(back == full);
  PoseArrayRosMessage tooMany;
  tooMany.poses.resize(contract_limits::kMaxPoseCount + 1);
  EXPECT_FALSE(PoseArrayRosConversion::RosToContract(tooMany, contract));
  PoseArrayRosContract tooManyContract;
  for (std::size_t i = 0; i <= contract_limits::kMaxPoseCount; ++i)
    tooManyContract.add_poses()->mutable_orientation()->set_w(1.0);
  EXPECT_FALSE(PoseArrayRosConversion::ContractToRos(tooManyContract, back));
}

// UT-ROS-07: geometry_msgs/Vector3Stamped maps 1:1 to the Contract, the zero vector included; an
// invalid value is rejected in either direction.
TEST(Vector3StampedRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  Vector3StampedRosMessage ros;
  ros.header.stamp.sec = 15;
  ros.header.stamp.nanosec = 16;
  ros.header.frame_id = "base_link";
  ros.vector.x = 0.5;
  ros.vector.y = -0.25;
  ros.vector.z = -1.0;

  Vector3StampedRosContract contract;
  ASSERT_TRUE(Vector3StampedRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 16u);
  EXPECT_DOUBLE_EQ(contract.vector().x(), 0.5);
  EXPECT_DOUBLE_EQ(contract.vector().y(), -0.25);
  EXPECT_DOUBLE_EQ(contract.vector().z(), -1.0);

  Vector3StampedRosMessage back;
  back.header.frame_id = "stale";
  back.vector.x = 99.0;
  ASSERT_TRUE(Vector3StampedRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // The zero vector is a value like any other, and a Contract holds it as present.
  Vector3StampedRosMessage zero = ros;
  zero.vector = geometry_msgs::msg::Vector3();
  Vector3StampedRosContract zeroContract;
  ASSERT_TRUE(Vector3StampedRosConversion::RosToContract(zero, zeroContract));
  EXPECT_TRUE(zeroContract.has_vector());
  ASSERT_TRUE(Vector3StampedRosConversion::ContractToRos(zeroContract, back));
  EXPECT_EQ(back, zero);

  Vector3StampedRosMessage invalid = ros;
  invalid.vector.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(Vector3StampedRosConversion::RosToContract(invalid, contract));

  // A rejected Contract leaves the ROS message as it was.
  back = ros;
  Vector3StampedRosContract badContract;
  ASSERT_TRUE(Vector3StampedRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(Vector3StampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  badContract.mutable_header()->set_frame_id("base_link");
  badContract.clear_vector();
  EXPECT_FALSE(Vector3StampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
}
