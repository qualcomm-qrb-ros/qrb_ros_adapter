// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_TYPES_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_TYPES_HPP

#include <cstdint>
#include <string_view>

namespace qrb::adapter
{

enum class PayloadClass : std::uint8_t
{
  kInline,
  kDmaBuf,
};

enum class Direction : std::uint8_t
{
  kUnspecified = 0,
  kRosToHcm = 1,
  kHcmToRos = 2,
};

/**
 * @brief The integer that configures a Direction: the Adapter's static
 * `direction` field and the Flow node's `direction` parameter.
 */
constexpr std::int64_t kDirectionCodeHcmToRos = 0;  // the HCM pipeline publishes to ROS
constexpr std::int64_t kDirectionCodeRosToHcm = 1;  // the HCM pipeline fetches from ROS

/** @brief ROS reliability policy; values keep the rclcpp names. */
enum class QosReliability : std::uint8_t
{
  kReliable = 0,
  kBestEffort = 1,
};

/** @brief ROS durability policy; values keep the rclcpp names. */
enum class QosDurability : std::uint8_t
{
  kVolatile = 0,
  kTransientLocal = 1,
};

/**
 * @brief Bounds on the KEEP_LAST depth. The maximum matches the 32 descriptors
 * of QCNode's cross-process SharedRing. It bounds the publisher's KEEP_LAST
 * history, and the pool of a DmaBuf Contract's Flow (depth + 2 buffers).
 */
constexpr std::uint32_t kMinQosDepth = 1;
constexpr std::uint32_t kMaxQosDepth = 32;

/**
 * @brief The ROS QoS of one dynamically created Topic endpoint.
 *
 * Only reliability, durability and the KEEP_LAST depth are configurable; the
 * defaults are those of `rclcpp::QoS( 10 )`. History is always KEEP_LAST and
 * every other policy keeps the rclcpp default.
 */
struct QosProfile
{
  QosReliability reliability{QosReliability::kReliable};
  QosDurability durability{QosDurability::kVolatile};
  std::uint32_t depth{10};
};

inline bool operator==(const QosProfile & a, const QosProfile & b)
{
  return a.reliability == b.reliability && a.durability == b.durability && a.depth == b.depth;
}

inline bool operator!=(const QosProfile & a, const QosProfile & b)
{
  return !(a == b);
}

struct ContractDescriptor
{
  std::string_view typeName;
  PayloadClass payloadClass{PayloadClass::kInline};
};

const char * ToString(PayloadClass payloadClass);
const char * ToString(Direction direction);
const char * ToString(QosReliability reliability);
const char * ToString(QosDurability durability);
/** @brief The direction code of `direction`; -1 for kUnspecified. */
std::int64_t ToDirectionCode(Direction direction);
bool ParseDirectionCode(std::int64_t code, Direction & out);
bool ParseQosReliability(std::string_view text, QosReliability & out);
bool ParseQosDurability(std::string_view text, QosDurability & out);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_TYPES_HPP
