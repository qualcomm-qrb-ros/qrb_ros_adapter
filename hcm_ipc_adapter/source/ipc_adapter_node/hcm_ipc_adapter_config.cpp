// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_config.hpp"

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <limits>
#include <string_view>

namespace qrb::adapter
{

namespace
{

namespace field
{
constexpr char kStatic[] = "static";
constexpr char kName[] = "name";
constexpr char kId[] = "id";
constexpr char kFlowId[] = "flow_id";
constexpr char kContractTypeName[] = "contract_type_name";
constexpr char kDirection[] = "direction";
constexpr char kRosTopic[] = "ros_topic";
constexpr char kBootstrapSocketPath[] = "bootstrap_socket_path";
constexpr char kDescriptorLocalSlot[] = "descriptor_local_slot";
constexpr char kStartupTimeoutMs[] = "startup_timeout_ms";
constexpr char kRequestTimeoutMs[] = "request_timeout_ms";
constexpr char kSampleTimeoutMs[] = "sample_timeout_ms";
constexpr char kQosProfile[] = "qos_profile";
constexpr char kQosDepth[] = "qos_depth";
constexpr char kEnableLatencyMonitoring[] = "enable_latency_monitoring";
constexpr char kContractTypeId[] = "contract_type_id";
constexpr char kRosTransportMode[] = "ros_transport_mode";
constexpr char kSocketPath[] = "socket_path";
constexpr char kPayloadLocalSlot[] = "payload_local_slot";
}   // namespace field

constexpr std::array<const char *, 8> kRequiredOptionFields = { field::kName, field::kId,
    field::kFlowId, field::kContractTypeName, field::kDirection, field::kRosTopic,
    field::kBootstrapSocketPath, field::kDescriptorLocalSlot };
constexpr std::array<const char *, 6> kOptionalOptionFields = { field::kRequestTimeoutMs,
    field::kSampleTimeoutMs, field::kStartupTimeoutMs, field::kQosProfile, field::kQosDepth,
    field::kEnableLatencyMonitoring };

bool ParseConfiguredDirection( std::string_view text, Direction &out )
{
    if ( text == "ros_to_hcm" )
    {
        out = Direction::kRosToHcm;
        return true;
    }
    if ( text == "hcm_to_ros" )
    {
        out = Direction::kHcmToRos;
        return true;
    }
    return false;
}

bool GetRequiredString(
    const nlohmann::json &node, const char *key, std::string &out, std::string &diagnostic )
{
    auto it = node.find( key );
    if ( it == node.end() || !it->is_string() )
    {
        diagnostic += std::string( "missing or non-string field \"" ) + key + "\"; ";
        return false;
    }
    out = it->get<std::string>();
    return true;
}

bool GetRequiredUint(
    const nlohmann::json &node, const char *key, std::uint32_t &out, std::string &diagnostic )
{
    auto it = node.find( key );
    if ( it == node.end() || !it->is_number_unsigned() )
    {
        diagnostic += std::string( "missing or non-unsigned field \"" ) + key + "\"; ";
        return false;
    }
    const std::uint64_t value = it->get<std::uint64_t>();
    if ( value > std::numeric_limits<std::uint32_t>::max() )
    {
        diagnostic += std::string( "field \"" ) + key + "\" exceeds uint32 range; ";
        return false;
    }
    out = static_cast<std::uint32_t>( value );
    return true;
}

bool GetOptionalUint(
    const nlohmann::json &node, const char *key, std::uint32_t &out, std::string &diagnostic )
{
    auto it = node.find( key );
    if ( it == node.end() ) return true;   // keep the default
    if ( !it->is_number_unsigned() )
    {
        diagnostic += std::string( "\"" ) + key + "\" must be a non-negative integer; ";
        return false;
    }
    const std::uint64_t value = it->get<std::uint64_t>();
    if ( value > std::numeric_limits<std::uint32_t>::max() )
    {
        diagnostic += std::string( "\"" ) + key + "\" exceeds uint32 range; ";
        return false;
    }
    out = static_cast<std::uint32_t>( value );
    return true;
}

bool GetOptionalBool(
    const nlohmann::json &node, const char *key, bool &out, std::string &diagnostic )
{
    auto it = node.find( key );
    if ( it == node.end() ) return true;
    if ( !it->is_boolean() )
    {
        diagnostic += std::string( "\"" ) + key + "\" must be a boolean; ";
        return false;
    }
    out = it->get<bool>();
    return true;
}

bool GetRequiredLocalSlot(
    const nlohmann::json &node, const char *key, std::uint32_t &out, std::string &diagnostic )
{
    if ( !GetRequiredUint( node, key, out, diagnostic ) ) return false;
    if ( out > UINT8_MAX )
    {
        diagnostic += std::string( "\"" ) + key + "\" must be in [0, 255]; ";
        return false;
    }
    return true;
}

}   // namespace

const std::string &HcmIpcAdapterConfigOptions()
{
    static const std::string options = []() {
        nlohmann::json schema;
        for ( const char *key : kRequiredOptionFields )
            schema["required"].push_back( std::string( "static." ) + key );
        for ( const char *key : kOptionalOptionFields )
            schema["optional"].push_back( std::string( "static." ) + key );
        return schema.dump();
    }();
    return options;
}

Status ParseFlowConfigJson( const std::string &json, FlowConfig &out, std::string &diagnostic )
{
    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse( json );
    }
    catch ( const nlohmann::json::parse_error &e )
    {
        diagnostic += std::string( "JSON parse error: " ) + e.what() + "; ";
        return Status::kInvalidArgument;
    }

    auto staticIt = root.find( field::kStatic );
    if ( staticIt == root.end() || !staticIt->is_object() )
    {
        diagnostic += "missing \"static\" object; ";
        return Status::kInvalidArgument;
    }
    const nlohmann::json &node = *staticIt;

    FlowConfig config;
    bool ok = true;
    ok &= GetRequiredString( node, field::kFlowId, config.flowId, diagnostic );
    ok &= GetRequiredString( node, field::kRosTopic, config.rosTopic, diagnostic );
    ok &= GetRequiredString(
        node, field::kContractTypeName, config.contractTypeName, diagnostic );

    std::string directionText;
    ok &= GetRequiredString( node, field::kDirection, directionText, diagnostic );

    std::string qosProfileText = ToString( config.qosProfile );
    auto qosProfileIt = node.find( field::kQosProfile );
    if ( qosProfileIt != node.end() )
    {
        if ( qosProfileIt->is_string() ) qosProfileText = qosProfileIt->get<std::string>();
        else
        {
            diagnostic += "\"qos_profile\" must be a string; ";
            ok = false;
        }
    }

    ok &= GetOptionalUint(
        node, field::kStartupTimeoutMs, config.startupTimeoutMs, diagnostic );
    ok &= GetOptionalUint(
        node, field::kRequestTimeoutMs, config.requestTimeoutMs, diagnostic );
    ok &= GetOptionalUint(
        node, field::kSampleTimeoutMs, config.sampleTimeoutMs, diagnostic );
    ok &= GetOptionalUint( node, field::kQosDepth, config.qosDepth, diagnostic );

    if ( node.contains( field::kContractTypeId ) )
    {
        diagnostic += "\"contract_type_id\" is not supported; use \"contract_type_name\"; ";
        ok = false;
    }
    if ( node.contains( field::kRosTransportMode ) )
    {
        diagnostic += "\"ros_transport_mode\" is not configurable; ";
        ok = false;
    }
    if ( node.contains( field::kSocketPath ) )
    {
        diagnostic += "\"socket_path\" is assigned by the ROS Bridge Manager; ";
        ok = false;
    }

    if ( !ok ) return Status::kInvalidArgument;

    if ( !ParseConfiguredDirection( directionText, config.direction ) )
    {
        diagnostic += "\"direction\" must be \"ros_to_hcm\" or \"hcm_to_ros\", got \""
            + directionText + "\"; ";
        return Status::kInvalidArgument;
    }
    if ( !ParseQosProfile( qosProfileText, config.qosProfile ) )
    {
        diagnostic += "\"qos_profile\" must be \"ros_default\", \"reliable\", "
            "\"sensor_data\" or \"transient_local\", got \"" + qosProfileText + "\"; ";
        return Status::kInvalidArgument;
    }
    out = config;
    return Status::kOk;
}

Status ParseHcmIpcAdapterConfigJson(
    const std::string &json, HcmIpcAdapterConfig &out, std::string &diagnostic )
{
    FlowConfig flowConfig;
    Status parsed = ParseFlowConfigJson( json, flowConfig, diagnostic );
    if ( parsed != Status::kOk ) return parsed;

    const HcmContractBinding *binding = FindHcmContractBinding( flowConfig.contractTypeName );
    if ( binding == nullptr || binding->descriptor == nullptr )
    {
        diagnostic += "contract_type_name has no HCM implementation; ";
        return Status::kUnsupportedContract;
    }

    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse( json );
    }
    catch ( const nlohmann::json::parse_error & )
    {
        return Status::kInvalidArgument;
    }
    const nlohmann::json &node = root.at( field::kStatic );

    HcmIpcAdapterConfig config;
    config.flowConfig = std::move( flowConfig );
    bool ok = true;
    ok &= GetRequiredString(
        node, field::kBootstrapSocketPath, config.bootstrapSocketPath, diagnostic );
    ok &= GetRequiredString( node, field::kName, config.nodeName, diagnostic );
    ok &= GetRequiredUint( node, field::kId, config.nodeId, diagnostic );
    ok &= GetRequiredLocalSlot(
        node, field::kDescriptorLocalSlot, config.buffers.descriptorLocalSlot, diagnostic );
    ok &= GetOptionalBool( node, field::kEnableLatencyMonitoring,
        config.enableLatencyMonitoring, diagnostic );

    if ( node.contains( field::kPayloadLocalSlot ) )
    {
        diagnostic += "payload_local_slot is obsolete; native qrb_contracts descriptors use one slot; ";
        ok = false;
    }

    if ( !ok ) return Status::kInvalidArgument;
    if ( config.nodeName.empty() || config.nodeId > UINT8_MAX )
    {
        diagnostic += "name must not be empty and id must be in [0, 255]; ";
        return Status::kInvalidArgument;
    }
    out = std::move( config );
    return Status::kOk;
}

}   // namespace qrb::adapter
