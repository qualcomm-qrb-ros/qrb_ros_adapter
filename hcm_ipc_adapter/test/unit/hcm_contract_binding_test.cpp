// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/joint_state_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/odometry_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/point_cloud_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/tf_message_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/twist_hcm_converter.hpp"

using namespace qrb::adapter;

// UT-HCM-02: every shared Contract has exactly one complete HCM binding, and the
// descriptor factory creates, by Contract name, descriptors only that binding
// accepts.
TEST(HcmContractBinding, EveryContractHasOneCompleteBindingAndAMatchingFactory)
{
  ASSERT_EQ(HcmContractBindingCount(), ContractDescriptorCount());
  for (std::size_t i = 0; i < HcmContractBindingCount(); ++i) {
    const HcmContractBinding * binding = HcmContractBindingAt(i);
    ASSERT_NE(binding, nullptr);
    ASSERT_NE(binding->descriptor, nullptr);
    const std::string name(binding->descriptor->typeName);
    EXPECT_EQ(FindHcmContractBinding(name), binding) << name;
    EXPECT_NE(binding->acceptsDescriptor, nullptr) << name;
    EXPECT_NE(binding->contractToHcm, nullptr) << name;
    EXPECT_NE(binding->hcmToContract, nullptr) << name;
    EXPECT_NE(binding->createDescriptor, nullptr) << name;
  }
  EXPECT_EQ(HcmContractBindingAt(HcmContractBindingCount()), nullptr);
  EXPECT_EQ(FindHcmContractBinding("qrb.contract.unknown.v1.Missing"), nullptr);

  // Descriptors are told apart by C++ type, so a created descriptor is
  // accepted by its own binding and by no other, whatever its buffer type.
  for (std::size_t i = 0; i < HcmContractBindingCount(); ++i) {
    const HcmContractBinding & binding = *HcmContractBindingAt(i);
    const std::string name(binding.descriptor->typeName);
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> created;
    ASSERT_EQ(CreateQrbBufferDescriptor(name, created), QC::QC_STATUS_OK) << name;
    ASSERT_NE(created, nullptr) << name;
    for (std::size_t j = 0; j < HcmContractBindingCount(); ++j) {
      EXPECT_EQ(HcmContractBindingAt(j)->acceptsDescriptor(*created), i == j) << name;
    }
  }

  std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> descriptor =
      std::make_unique<QC::Memory::QCBufferDescriptorBase_t>();
  EXPECT_EQ(CreateQrbBufferDescriptor("qrb.contract.unknown.v1.Missing", descriptor),
      QC::QC_STATUS_UNSUPPORTED);
  EXPECT_EQ(descriptor, nullptr);

  // A binding never accepts or converts another Contract's descriptor.
  const HcmContractBinding & pose = *FindHcmContractBinding(kPoseWithCovarianceStampedContractType);
  PointCloudHcmDescriptor cloudDescriptor;
  PoseHcmContract poseContract;
  EXPECT_FALSE(pose.acceptsDescriptor(cloudDescriptor));
  EXPECT_FALSE(pose.contractToHcm(poseContract, DmaBufView{}, cloudDescriptor));
  EXPECT_FALSE(pose.hcmToContract(cloudDescriptor, poseContract));

  // Pose and Odometry are both inline geometry; neither takes the other's descriptor.
  const HcmContractBinding & odom = *FindHcmContractBinding(kOdometryContractType);
  OdometryHcmDescriptor odomDescriptor;
  PoseHcmDescriptor poseDescriptor;
  OdometryHcmContract odomContract;
  EXPECT_TRUE(odom.acceptsDescriptor(odomDescriptor));
  EXPECT_FALSE(odom.acceptsDescriptor(poseDescriptor));
  EXPECT_FALSE(pose.acceptsDescriptor(odomDescriptor));
  EXPECT_FALSE(odom.contractToHcm(odomContract, DmaBufView{}, poseDescriptor));
  EXPECT_FALSE(odom.hcmToContract(poseDescriptor, odomContract));

  // Twist and Odometry are inline geometry too; neither takes the other's descriptor.
  const HcmContractBinding & twist = *FindHcmContractBinding(kTwistStampedContractType);
  TwistHcmDescriptor twistDescriptor;
  TwistHcmContract twistContract;
  EXPECT_TRUE(twist.acceptsDescriptor(twistDescriptor));
  EXPECT_FALSE(twist.acceptsDescriptor(odomDescriptor));
  EXPECT_FALSE(odom.acceptsDescriptor(twistDescriptor));
  EXPECT_FALSE(twist.contractToHcm(twistContract, DmaBufView{}, odomDescriptor));
  EXPECT_FALSE(twist.hcmToContract(odomDescriptor, twistContract));

  // TFMessage is inline geometry too; neither takes the other's descriptor.
  const HcmContractBinding & tf = *FindHcmContractBinding(kTFMessageContractType);
  TFMessageHcmDescriptor tfDescriptor;
  TFMessageHcmContract tfContract;
  EXPECT_TRUE(tf.acceptsDescriptor(tfDescriptor));
  EXPECT_FALSE(tf.acceptsDescriptor(twistDescriptor));
  EXPECT_FALSE(twist.acceptsDescriptor(tfDescriptor));
  EXPECT_FALSE(tf.contractToHcm(tfContract, DmaBufView{}, twistDescriptor));
  EXPECT_FALSE(tf.hcmToContract(twistDescriptor, tfContract));

  // JointState and TFMessage share CUSTOM_4, but their C++ descriptors are distinct.
  const HcmContractBinding & joints = *FindHcmContractBinding(kJointStateContractType);
  JointStateHcmDescriptor jointDescriptor;
  JointStateHcmContract jointContract;
  EXPECT_TRUE(joints.acceptsDescriptor(jointDescriptor));
  EXPECT_FALSE(joints.acceptsDescriptor(tfDescriptor));
  EXPECT_FALSE(tf.acceptsDescriptor(jointDescriptor));
  EXPECT_FALSE(joints.contractToHcm(jointContract, DmaBufView{}, tfDescriptor));
  EXPECT_FALSE(joints.hcmToContract(tfDescriptor, jointContract));
}
