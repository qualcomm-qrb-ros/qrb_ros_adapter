// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleIpcAdapter.cpp
 * @brief Sample for the IPC Adapter QCNode, runnable with QCNodeSampleApp (-t IpcAdapter).
 *
 * The sample creates the IPC Adapter node through QCNodeFactory with one Flow and calls its
 * ProcessFrameDescriptor() from a worker thread, with a frame descriptor holding the Contract
 * descriptor. The ROS Bridge (ros_ipc_bridge) must be running.
 *
 * HCM to ROS (direction 0): with input_topic, it sends every frame another sample publishes
 * there, for any Contract the Adapter maps; without, it sends a frame it fills itself
 * (FillFrame(), UpdateFrame()) every 1/fps second, for a Contract in kFilledContracts, and
 * rejects any other Contract in Init().
 * ROS to HCM (direction 1), for any mapped Contract: it fetches the latest ROS message into a
 * frame; with output_topic, it publishes every frame fetched there, and without, it fetches
 * every 1/fps second.
 *
 * It prints the node state after each lifecycle call, the configuration read back through
 * GetConfigurationIfs() after Initialize(), and the monitoring and frame counts after Stop().
 *
 * Attributes:
 *   adapter_config: (optional) a file with the Adapter's static config; it replaces the six
 *                   Flow attributes below. Without an "id", the sample's node id is used
 *   contract_type : (optional) a mapped Contract, default qrb.contract.sensor.v1.PointCloud2
 *   direction     : (optional) 0: HCM to ROS (default), 1: ROS to HCM
 *   ros_topic     : (optional) default /qrb_ros_adapter/sample/points for PointCloud2,
 *                   .../pose for PoseWithCovarianceStamped, .../detections_2d and
 *                   .../detections_3d for Detection2DArray and Detection3DArray, else
 *                   .../<message name in snake_case, e.g. odometry for Odometry>
 *   reliability   : (optional) reliable or best_effort, default the ROS default
 *   durability    : (optional) volatile or transient_local, default the ROS default
 *   depth         : (optional) 1..32, default the ROS default
 *   input_topic   : (optional) direction 0: the topic of the frames to send
 *   output_topic  : (optional) direction 1: the topic to publish the fetched frames on
 *   pool_size     : (optional) frames to fetch into with output_topic, 1..32, default 4
 *   fps           : (optional) frames per second, 0 for no pause, default 30
 *   payload_bytes : (optional) bytes of points sent to ROS, for PointCloud2, or of the rgb8
 *                   rows of an ImageLayout, default 1048576; for Detection2DArray and
 *                   Detection3DArray, the buffer that holds the two detections the sample sends
 *   buffer_bytes  : (optional) direction 1, for a DmaBuf Contract and for the detections: size of
 *                   the DMA buffer the sample receives into, default 33554432
 */

#include "ContractFrame.hpp"
#include "QC/Node/Ifs/QCNodeFactory.hpp"
#include "QC/Node/NodeFrameDescriptor.hpp"
#include "QC/sample/DataBroker.hpp"
#include "QC/sample/SampleIF.hpp"
#include "QRB/Memory/Geometry/PoseArrayDescriptor.hpp"
#include "QRB/Memory/Geometry/PoseDescriptor.hpp"
#include "QRB/Memory/Geometry/PoseWithCovarianceDescriptor.hpp"
#include "QRB/Memory/Geometry/TFMessageDescriptor.hpp"
#include "QRB/Memory/Geometry/TwistDescriptor.hpp"
#include "QRB/Memory/Geometry/Vector3Descriptor.hpp"
#include "QRB/Memory/Navigation/OdometryDescriptor.hpp"
#include "QRB/Memory/Sensor/ImuDescriptor.hpp"
#include "QRB/Memory/Sensor/JointStateBufferDescriptor.hpp"
#include "QRB/Memory/Sensor/PointCloudDescriptor.hpp"
#include "QRB/Memory/Vision/ObjectDetection2dDescriptor.hpp"
#include "QRB/Memory/Vision/ObjectDetection3dDescriptor.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/image_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_node.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>

namespace QC
{
namespace sample
{

using namespace QC::Node;

static const char *StateName( QCObjectState_e state )
{
    switch ( state )
    {
        case QC_OBJECT_STATE_INITIAL:
            return "INITIAL";
        case QC_OBJECT_STATE_INITIALIZING:
            return "INITIALIZING";
        case QC_OBJECT_STATE_READY:
            return "READY";
        case QC_OBJECT_STATE_STARTING:
            return "STARTING";
        case QC_OBJECT_STATE_RUNNING:
            return "RUNNING";
        case QC_OBJECT_STATE_STOPING:
            return "STOPING";
        case QC_OBJECT_STATE_ERROR:
            return "ERROR";
        default:
            return "OTHER";
    }
}

/** Copy @p text into a char array of a detection record, cut to fit and always NUL-terminated. */
template<size_t N>
static void SetText( char ( &dest )[N], const char *text )
{
    (void) snprintf( dest, N, "%s", text );
}

/** Add the class @p classId with @p score to @p record, with its pose at the frame's origin. */
template<typename RecordT>
static void AddHypothesis( RecordT &record, const char *classId, double score )
{
    if ( record.resultCount < QRB_DETECTION_MAX_RESULTS )
    {
        QRB::Memory::ObjectHypothesisRaw_t &result = record.results[record.resultCount++];
        SetText( result.classId, classId );
        result.score = score;
    }
}

/** Whether the buffer of @p descriptor holds @p count detections from its offset on. */
template<typename DescriptorT>
static bool FitsDetections( const DescriptorT &descriptor, uint32_t count )
{
    return ( nullptr != descriptor.pBuf ) &&
           ( descriptor.size >= descriptor.offset + DescriptorT::CalcBufferSize( count ) );
}

class SampleIpcAdapter : public SampleIF
{
public:
    SampleIpcAdapter() = default;
    ~SampleIpcAdapter() = default;

    QCStatus_e Init( std::string name, SampleConfig_t &config ) override;
    QCStatus_e Start() override;
    QCStatus_e Stop() override;
    QCStatus_e Deinit() override;

private:
    QCStatus_e LoadAdapterConfig( const std::string &path, std::string &config );
    std::string DefaultTopic() const;
    std::string MakeAdapterConfig( SampleConfig_t &config );
    QCStatus_e SetupFrames();
    void FillFrame( ContractFrame &frame );
    void UpdateFrame( ContractFrame &frame );
    QCStatus_e Process( ContractFrame &frame );
    bool GetMonitor( qrb::adapter::HcmIpcAdapterMonitoringData &monitor );
    void PrintState( const char *call, QCStatus_e ret );
    void PrintConfig();
    void PrintMonitor();
    void SendFromTopic();
    void FetchToTopic();
    void RunStandalone();
    void ThreadMain();

private:
    std::unique_ptr<QCNodeIfs> m_node;
    std::string m_contract;
    bool m_isPointCloud = true;
    bool m_isImage = false;
    bool m_isHcmToRos = true;
    uint32_t m_bufferId = 0;
    std::string m_inputTopic;
    std::string m_outputTopic;
    uint32_t m_poolSize = 4;
    uint32_t m_fps = 30;
    uint32_t m_payloadBytes = 1048576;
    uint32_t m_bufferBytes = 33554432;

    ContractFrame m_frame;
    ContractFramePool m_pool;
    DataSubscriber<ContractFrameRef_t> m_sub;
    DataPublisher<ContractFrameRef_t> m_pub;

    std::thread m_thread;
    std::atomic<bool> m_stop{ false };
    uint64_t m_noDataCount = 0;
    uint64_t m_frames = 0;
    uint64_t m_errorCount = 0;
    uint64_t m_poolExhausted = 0;

    static constexpr const char *kPointCloudContract = "qrb.contract.sensor.v1.PointCloud2";
    static constexpr const char *kImageContract = "qrb.contract.sensor.v1.ImageLayout";
    static constexpr const char *kPoseContract =
            "qrb.contract.geometry.v1.PoseWithCovarianceStamped";
    static constexpr const char *kDetection2DContract = "qrb.contract.vision.v1.Detection2DArray";
    static constexpr const char *kDetection3DContract = "qrb.contract.vision.v1.Detection3DArray";
    static constexpr const char *kImuContract = "qrb.contract.sensor.v1.Imu";
    /**
     * The Contracts FillFrame() / UpdateFrame() can fill, so the sample sends them without
     * input_topic. To add one, add it here and give both a case for its descriptor.
     */
    static constexpr const char *kFilledContracts[] = {
            kPointCloudContract,
            kPoseContract,
            "qrb.contract.nav.v1.Odometry",
            "qrb.contract.geometry.v1.TwistStamped",
            "qrb.contract.tf2.v1.TFMessage",
            kImageContract,
            "qrb.contract.sensor.v1.JointState",
            "qrb.contract.geometry.v1.PoseStamped",
            "qrb.contract.geometry.v1.PoseArray",
            "qrb.contract.geometry.v1.Vector3Stamped",
            kDetection2DContract,
            kDetection3DContract,
            kImuContract,
    };
    bool IsFilledContract() const;
    /** Bytes the two detections of the sample take in a buffer, 0 for another Contract. */
    uint32_t DetectionBytes() const;

    // The detections of a frame the sample fills, in the buffer of the frame.
    static constexpr uint32_t kSampleDetections = 2;

    static constexpr uint32_t kPointStep = 3 * sizeof( float );
    // An rgb8 image of kImageWidth, its rows ALIGN( width * 3, 16 ) apart as the QCNode Camera
    // sample allocates them.
    static constexpr uint32_t kImageWidth = 640;
    static constexpr uint32_t kImageStride = ( kImageWidth * 3 + 15 ) / 16 * 16;
    static constexpr uint64_t kMaxReportedErrors = 10;
    static constexpr uint32_t kReceiveTimeoutMs = 100;
    // How long to wait after a fetch that found no ROS message, or with every frame held.
    static constexpr auto kNoDataWait = std::chrono::microseconds( 200 );
    static constexpr auto kPoolWait = std::chrono::microseconds( 100 );
};

QCStatus_e SampleIpcAdapter::LoadAdapterConfig( const std::string &path, std::string &config )
{
    std::ifstream in( path );
    if ( !in )
    {
        QC_ERROR( "cannot read adapter_config %s", path.c_str() );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    std::ostringstream text;
    text << in.rdbuf();
    config = text.str();

    // The Adapter checks the config; the sample only reads what it needs from it.
    DataTree dt;
    DataTree node;
    std::string errors;
    if ( ( QC_STATUS_OK != dt.Load( config, errors ) ) ||
         ( QC_STATUS_OK != dt.Get( "static", node ) ) )
    {
        QC_ERROR( "adapter_config %s has no \"static\" object: %s", path.c_str(), errors.c_str() );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    // Without an "id", the file takes the sample's node id; an "id" in the file wins.
    if ( !dt.Exists( "static.id" ) )
    {
        dt.Set<uint32_t>( "static.id", m_nodeId.id );
        config = dt.Dump();
    }
    m_contract = node.Get<std::string>( "contract_type", "" );
    m_isHcmToRos = ( 0 == node.Get<uint32_t>( "direction", 0 ) );
    std::vector<DataTree> map;
    if ( ( QC_STATUS_OK == node.Get( "globalBufferIdMap", map ) ) && ( false == map.empty() ) )
    {
        m_bufferId = map[0].Get<uint32_t>( "id", 0 );
    }

    return QC_STATUS_OK;
}

bool SampleIpcAdapter::IsFilledContract() const
{
    for ( const char *contract : kFilledContracts )
    {
        if ( m_contract == contract )
        {
            return true;
        }
    }
    return false;
}

uint32_t SampleIpcAdapter::DetectionBytes() const
{
    if ( kDetection2DContract == m_contract )
    {
        return (uint32_t) QRB::Memory::ObjectDetection2dDescriptor_t::CalcBufferSize(
                kSampleDetections );
    }
    if ( kDetection3DContract == m_contract )
    {
        return (uint32_t) QRB::Memory::ObjectDetection3dDescriptor_t::CalcBufferSize(
                kSampleDetections );
    }
    return 0;
}

std::string SampleIpcAdapter::DefaultTopic() const
{
    std::string topic = "/qrb_ros_adapter/sample/";
    if ( kPointCloudContract == m_contract )
    {
        return topic + "points";
    }
    if ( kPoseContract == m_contract )
    {
        return topic + "pose";
    }
    if ( kDetection2DContract == m_contract )
    {
        return topic + "detections_2d";
    }
    if ( kDetection3DContract == m_contract )
    {
        return topic + "detections_3d";
    }

    // The message name of the Contract in snake_case, e.g. Odometry: odometry.
    std::string message = m_contract.substr( m_contract.rfind( '.' ) + 1 );
    bool first = true;
    for ( char c : message )
    {
        if ( ( c >= 'A' ) && ( c <= 'Z' ) )
        {
            if ( !first )
            {
                topic += '_';
            }
            c = (char) ( c - 'A' + 'a' );
        }
        topic += c;
        first = false;
    }

    return topic;
}

std::string SampleIpcAdapter::MakeAdapterConfig( SampleConfig_t &config )
{
    std::string defaultTopic = DefaultTopic();
    DataTree dt;
    dt.Set<std::string>( "static.name", m_name );
    dt.Set<uint32_t>( "static.id", m_nodeId.id );
    dt.Set<std::string>( "static.contract_type", m_contract );
    dt.Set<uint32_t>( "static.direction", m_isHcmToRos ? 0 : 1 );
    dt.Set<std::string>( "static.ros_topic", Get( config, "ros_topic", defaultTopic ) );
    std::string reliability = Get( config, "reliability", "" );
    std::string durability = Get( config, "durability", "" );
    uint32_t depth = Get( config, "depth", (uint32_t) 0 );
    if ( !reliability.empty() )
    {
        dt.Set<std::string>( "static.qos_profile.reliability", reliability );
    }
    if ( !durability.empty() )
    {
        dt.Set<std::string>( "static.qos_profile.durability", durability );
    }
    if ( 0 != depth )
    {
        dt.Set<uint32_t>( "static.qos_profile.depth", depth );
    }

    return dt.Dump();
}

QCStatus_e SampleIpcAdapter::Init( std::string name, SampleConfig_t &config )
{
    QCStatus_e ret = SampleIF::Init( name, QC_NODE_TYPE_IPC_ADAPTER );
    std::string adapterConfig;

    if ( QC_STATUS_OK == ret )
    {
        std::string path = Get( config, "adapter_config", "" );
        if ( !path.empty() )
        {
            ret = LoadAdapterConfig( path, adapterConfig );
        }
        else
        {
            m_contract = Get( config, "contract_type", kPointCloudContract );
            uint32_t direction = Get( config, "direction", (uint32_t) 0 );
            if ( direction > 1 )
            {
                QC_ERROR( "direction must be 0 (HCM to ROS) or 1 (ROS to HCM): %u", direction );
                ret = QC_STATUS_BAD_ARGUMENTS;
            }
            m_isHcmToRos = ( 0 == direction );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        m_isPointCloud = ( kPointCloudContract == m_contract );
        m_isImage = ( kImageContract == m_contract );
        if ( nullptr == qrb::adapter::FindHcmContractBinding( m_contract ) )
        {
            QC_ERROR( "contract_type is not mapped by the Adapter: %s", m_contract.c_str() );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
        m_inputTopic = Get( config, "input_topic", "" );
        m_outputTopic = Get( config, "output_topic", "" );
        m_poolSize = Get( config, "pool_size", (uint32_t) 4 );
        m_fps = Get( config, "fps", (uint32_t) 30 );
        m_payloadBytes = Get( config, "payload_bytes", (uint32_t) 1048576 );
        m_bufferBytes = Get( config, "buffer_bytes", (uint32_t) 33554432 );
        if ( ( !m_inputTopic.empty() && !m_isHcmToRos ) ||
             ( !m_outputTopic.empty() && m_isHcmToRos ) )
        {
            QC_ERROR( "input_topic is for direction 0 (HCM to ROS), output_topic for 1" );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( ( m_poolSize < 1 ) || ( m_poolSize > ContractFramePool::kMaxSize ) )
        {
            QC_ERROR( "pool_size must be in [1, %u]: %u", ContractFramePool::kMaxSize, m_poolSize );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        ret = SetupFrames();
    }

    if ( QC_STATUS_OK == ret )
    {
        ret = QCNodeFactory::CreateNode( QC_NODE_TYPE_IPC_ADAPTER, m_node );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to create the IPC Adapter node: %d", ret );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        QCNodeInit_t nodeInit;
        nodeInit.config = adapterConfig.empty() ? MakeAdapterConfig( config ) : adapterConfig;
        ret = m_node->Initialize( nodeInit );
        PrintState( "Initialize", ret );
        if ( QC_STATUS_OK == ret )
        {
            PrintConfig();
        }
        else
        {
            // NO_RESOURCE: the ROS Bridge is not running.
            QC_ERROR( "failed to initialize the IPC Adapter node with %s: %d",
                      nodeInit.config.c_str(), ret );
        }
    }

    if ( ( QC_STATUS_OK == ret ) && !m_inputTopic.empty() )
    {
        // FIFO and as deep as any pool, so that no frame of the publisher is dropped.
        ret = m_sub.Init( name, m_inputTopic, ContractFramePool::kMaxSize, false );
    }

    if ( ( QC_STATUS_OK == ret ) && !m_outputTopic.empty() )
    {
        ret = m_pub.Init( name, m_outputTopic );
    }

    return ret;
}

QCStatus_e SampleIpcAdapter::SetupFrames()
{
    QCStatus_e ret = QC_STATUS_OK;

    if ( !m_inputTopic.empty() )
    {
        // The publisher of input_topic owns the frames.
        return ret;
    }
    if ( m_isHcmToRos && !IsFilledContract() )
    {
        // FillFrame() knows no other descriptor, and an unfilled one is rejected every frame.
        QC_ERROR( "without input_topic, the sample sends only the Contracts it fills; send %s "
                  "frames on input_topic, for example from the StreamProbe sample",
                  m_contract.c_str() );
        return QC_STATUS_BAD_ARGUMENTS;
    }

    uint32_t bytes = m_isHcmToRos ? m_payloadBytes : m_bufferBytes;
    const qrb::adapter::HcmContractBinding *binding =
            qrb::adapter::FindHcmContractBinding( m_contract );
    if ( ( nullptr != binding ) &&
         ( qrb::adapter::PayloadClass::kDmaBuf == binding->descriptor->payloadClass ) &&
         ( 0 == bytes ) )
    {
        QC_ERROR( "%s must not be 0 for %s", m_isHcmToRos ? "payload_bytes" : "buffer_bytes",
                  m_contract.c_str() );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    if ( m_isPointCloud && m_isHcmToRos && ( bytes < kPointStep ) )
    {
        QC_ERROR( "payload_bytes must hold at least one point: %u", bytes );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    if ( m_isImage && m_isHcmToRos && ( bytes < kImageStride ) )
    {
        QC_ERROR( "payload_bytes must hold at least one image row of %u bytes: %u", kImageStride,
                  bytes );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    if ( m_isHcmToRos && ( bytes < DetectionBytes() ) )
    {
        QC_ERROR( "payload_bytes must hold the %u detections of a %s frame, %u bytes: %u",
                  kSampleDetections, m_contract.c_str(), DetectionBytes(), bytes );
        return QC_STATUS_BAD_ARGUMENTS;
    }

    if ( !m_outputTopic.empty() )
    {
        ret = m_pool.Init( m_contract, bytes, m_poolSize );
    }
    else
    {
        ret = m_frame.Init( m_contract, bytes );
        if ( ( QC_STATUS_OK == ret ) && m_isHcmToRos )
        {
            FillFrame( m_frame );
        }
    }
    if ( QC_STATUS_OK != ret )
    {
        QC_ERROR( "failed to create a %s frame of %u bytes: %d", m_contract.c_str(), bytes, ret );
    }

    return ret;
}

/** Set the parts of a frame that stay the same from frame to frame; UpdateFrame() does the rest. */
void SampleIpcAdapter::FillFrame( ContractFrame &frame )
{
    auto *pose = dynamic_cast<QRB::Memory::PoseWithCovarianceDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != pose )
    {
        (void) pose->frameId.Assign( "map" );
    }

    auto *odom = dynamic_cast<QRB::Memory::OdometryDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != odom )
    {
        (void) odom->frameId.Assign( "odom" );
        (void) odom->childFrameId.Assign( "base_link" );
    }

    auto *twist = dynamic_cast<QRB::Memory::TwistDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != twist )
    {
        (void) twist->frameId.Assign( "base_link" );
    }

    auto *tf = dynamic_cast<QRB::Memory::TFMessageDescriptor_t *>( &frame.Descriptor() );
    if ( ( nullptr != tf ) &&
         ( tf->size >= QRB::Memory::TFMessageDescriptor_t::CalcBufferSize( 2 ) ) )
    {
        // map -> odom and odom -> base_link, in the heap storage of the frame.
        const char *frames[2][2] = { { "map", "odom" }, { "odom", "base_link" } };
        tf->transformCount = 2;
        tf->validSize = QRB::Memory::TFMessageDescriptor_t::CalcBufferSize( 2 );
        for ( uint32_t i = 0; i < 2; i++ )
        {
            QRB::Memory::TransformStamped_t *transform = tf->GetTransformPtr( i );
            *transform = QRB::Memory::TransformStamped_t{};
            (void) transform->frameId.Assign( frames[i][0] );
            (void) transform->childFrameId.Assign( frames[i][1] );
        }
    }

    auto *image = dynamic_cast<qrb::adapter::ImageHcmDescriptor *>( &frame.Descriptor() );
    if ( nullptr != image )
    {
        // A kImageWidth x ( payload_bytes / kImageStride ) rgb8 Camera frame; UpdateFrame()
        // draws it.
        const uint32_t height = m_payloadBytes / kImageStride;
        image->format = QC_IMAGE_FORMAT_RGB888;
        image->width = kImageWidth;
        image->height = height;
        image->batchSize = 1;
        image->numPlanes = 1;
        for ( uint32_t i = 0; i < QC_NUM_IMAGE_PLANES; i++ )
        {
            image->stride[i] = 0;
            image->actualHeight[i] = 0;
            image->planeBufSize[i] = 0;
        }
        image->stride[0] = kImageStride;
        image->actualHeight[0] = height;
        image->planeBufSize[0] = kImageStride * height;
        image->validSize = image->planeBufSize[0];
        image->timestampQGPTP = 0;
        image->flags = 0;
    }

    auto *joints = dynamic_cast<QRB::Memory::JointStateBufferDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != joints )
    {
        (void) joints->frameId.Assign( "base_link" );
        joints->jointCount = 3;
        joints->validSize = QRB::Memory::JointStateBufferDescriptor_t::CalcBufferSize( 3 );
        for ( uint32_t i = 0; i < joints->jointCount; i++ )
        {
            QRB::Memory::JointState_t *joint = joints->GetJointPtr( i );
            if ( nullptr != joint )
            {
                *joint = QRB::Memory::JointState_t{};
                (void) joint->name.Assign( ( "joint_" + std::to_string( i ) ).c_str() );
            }
        }
    }

    auto *poseStamped = dynamic_cast<QRB::Memory::PoseDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != poseStamped )
    {
        (void) poseStamped->frameId.Assign( "map" );
    }

    auto *poses = dynamic_cast<QRB::Memory::PoseArrayDescriptor_t *>( &frame.Descriptor() );
    if ( ( nullptr != poses ) &&
         ( poses->size >= QRB::Memory::PoseArrayDescriptor_t::CalcBufferSize( 2 ) ) )
    {
        // Two poses in "map", in the heap storage of the frame: one at the origin, one 1 m above.
        (void) poses->frameId.Assign( "map" );
        poses->poseCount = 2;
        poses->validSize = QRB::Memory::PoseArrayDescriptor_t::CalcBufferSize( 2 );
        for ( uint32_t i = 0; i < 2; i++ )
        {
            *poses->GetPosePtr( i ) = QRB::Memory::Pose_t{};
        }
        poses->GetPosePtr( 1 )->position.z = 1.0;
    }

    auto *vector3 = dynamic_cast<QRB::Memory::Vector3Descriptor_t *>( &frame.Descriptor() );
    if ( nullptr != vector3 )
    {
        // Straight down in "base_link"; UpdateFrame() tilts it.
        (void) vector3->frameId.Assign( "base_link" );
        vector3->vector.z = -1.0;
    }

    // The records of the two detections go into the DMA-BUF of the frame once, as the points of
    // a cloud do; UpdateFrame() only moves the stamp.
    auto *detections2d =
            dynamic_cast<QRB::Memory::ObjectDetection2dDescriptor_t *>( &frame.Descriptor() );
    if ( ( nullptr != detections2d ) && FitsDetections( *detections2d, kSampleDetections ) )
    {
        // A person and a cup, in the pixels of an image.
        QRB::Memory::Detection2dRaw_t person{};
        SetText( person.id, "track-1" );
        person.bbox.center.position.x = 320.0;
        person.bbox.center.position.y = 240.0;
        person.bbox.sizeX = 64.0;
        person.bbox.sizeY = 128.0;
        AddHypothesis( person, "person", 0.9 );

        QRB::Memory::Detection2dRaw_t cup{};
        SetText( cup.id, "track-2" );
        cup.bbox.center.position.x = 480.0;
        cup.bbox.center.position.y = 300.0;
        cup.bbox.sizeX = 40.0;
        cup.bbox.sizeY = 40.0;
        AddHypothesis( cup, "cup", 0.6 );
        AddHypothesis( cup, "bottle", 0.3 );

        (void) detections2d->frameId.Assign( "camera" );
        *detections2d->GetDetectionPtr( 0 ) = person;
        *detections2d->GetDetectionPtr( 1 ) = cup;
        detections2d->objCount = kSampleDetections;
        detections2d->validSize =
                QRB::Memory::ObjectDetection2dDescriptor_t::CalcBufferSize( kSampleDetections );
    }

    auto *detections3d =
            dynamic_cast<QRB::Memory::ObjectDetection3dDescriptor_t *>( &frame.Descriptor() );
    if ( ( nullptr != detections3d ) && FitsDetections( *detections3d, kSampleDetections ) )
    {
        // A car and a person, in meters.
        QRB::Memory::Detection3dRaw_t car{};
        SetText( car.id, "track-1" );
        car.bbox.center.position.x = 5.0;
        car.bbox.center.position.z = 0.75;
        car.bbox.size.x = 4.5;
        car.bbox.size.y = 2.0;
        car.bbox.size.z = 1.5;
        AddHypothesis( car, "car", 0.85 );

        QRB::Memory::Detection3dRaw_t person{};
        SetText( person.id, "track-2" );
        person.bbox.center.position.x = 2.0;
        person.bbox.center.position.y = 1.0;
        person.bbox.center.position.z = 0.9;
        person.bbox.size.x = 0.6;
        person.bbox.size.y = 0.6;
        person.bbox.size.z = 1.8;
        AddHypothesis( person, "person", 0.7 );

        (void) detections3d->frameId.Assign( "base_link" );
        *detections3d->GetDetectionPtr( 0 ) = car;
        *detections3d->GetDetectionPtr( 1 ) = person;
        detections3d->objCount = kSampleDetections;
        detections3d->validSize =
                QRB::Memory::ObjectDetection3dDescriptor_t::CalcBufferSize( kSampleDetections );
    }

    auto *imu = dynamic_cast<QRB::Memory::ImuDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != imu )
    {
        (void) imu->frameId.Assign( "imu_link" );
        imu->orientation.x = 0.0;
        imu->orientation.y = 0.0;
        imu->orientation.z = 0.0;
        imu->orientation.w = 1.0;
    }

    auto *cloud = dynamic_cast<QRB::Memory::PointCloudDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr == cloud )
    {
        return;
    }

    // An unordered XYZ cloud: point i is at ( i, i, i ) millimeters.
    cloud->width = m_payloadBytes / kPointStep;
    cloud->height = 1;
    cloud->pointStep = kPointStep;
    cloud->rowStep = cloud->width * kPointStep;
    cloud->isDense = true;
    const char *names[3] = { "x", "y", "z" };
    for ( uint32_t i = 0; i < 3; i++ )
    {
        cloud->fields[i] = QRB::Memory::PointField_t( names[i], i * (uint32_t) sizeof( float ),
                                                      QRB::Memory::POINT_FIELD_TYPE_FLOAT32 );
    }
    cloud->numFields = 3;
    cloud->validSize = cloud->rowStep;
    (void) cloud->frameId.Assign( "map" );
    for ( uint32_t i = 0; i < cloud->width; i++ )
    {
        float xyz[3] = { i * 0.001f, i * 0.001f, i * 0.001f };
        (void) memcpy( cloud->GetPointPtr( i ), xyz, sizeof( xyz ) );
    }
}

void SampleIpcAdapter::UpdateFrame( ContractFrame &frame )
{
    uint64_t nowNs = (uint64_t) std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch() )
                             .count();
    auto *cloud = dynamic_cast<QRB::Memory::PointCloudDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != cloud )
    {
        cloud->timestampNs = nowNs;
    }

    // Both detection descriptors share this base, which holds the stamp.
    auto *detections =
            dynamic_cast<QRB::Memory::ObjectDetectionDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != detections )
    {
        detections->timestampNs = nowNs;
    }

    auto *pose = dynamic_cast<QRB::Memory::PoseWithCovarianceDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != pose )
    {
        // Move along the x axis of "map" by 1 cm per frame.
        pose->timestampNs = nowNs;
        pose->poseWithCovariance.pose.position.x = (double) m_frames * 0.01;
    }

    auto *odom = dynamic_cast<QRB::Memory::OdometryDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != odom )
    {
        // Drive along the x axis of "odom" by 1 cm per frame; the twist is that speed at fps
        // frames per second (0 with fps 0, where frames are sent without a pause).
        odom->timestampNs = nowNs;
        odom->poseWithCovariance.pose.position.x = (double) m_frames * 0.01;
        odom->twistWithCovariance.twist.linear.x = 0.01 * (double) m_fps;
    }

    auto *twist = dynamic_cast<QRB::Memory::TwistDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != twist )
    {
        // Turn in place, faster with every frame and back to rest every 100 frames.
        twist->timestampNs = nowNs;
        twist->twist.angular.z = (double) ( m_frames % 100 ) * 0.01;
    }

    auto *image = dynamic_cast<qrb::adapter::ImageHcmDescriptor *>( &frame.Descriptor() );
    if ( ( nullptr != image ) && ( nullptr != image->pBuf ) )
    {
        // Camera frames carry the qtimer, which the Adapter converts to ROS time. Red rises
        // across the image and scrolls by 4 pixels per frame, green rises down it.
        image->timestamp = qrb::adapter::SampleHcmClocks().qtimerNs;
        uint8_t *row = static_cast<uint8_t *>( image->GetDataPtr() );
        for ( uint32_t y = 0; y < image->height; y++, row += image->stride[0] )
        {
            for ( uint32_t x = 0; x < image->width; x++ )
            {
                row[x * 3] = (uint8_t) ( x + m_frames * 4 );
                row[x * 3 + 1] = (uint8_t) y;
                row[x * 3 + 2] = 128;
            }
        }
    }

    auto *tf = dynamic_cast<QRB::Memory::TFMessageDescriptor_t *>( &frame.Descriptor() );
    QRB::Memory::TransformStamped_t *mapToOdom =
            ( nullptr != tf ) ? tf->GetTransformPtr( 0 ) : nullptr;
    QRB::Memory::TransformStamped_t *odomToBase =
            ( nullptr != tf ) ? tf->GetTransformPtr( 1 ) : nullptr;
    if ( ( nullptr != mapToOdom ) && ( nullptr != odomToBase ) )
    {
        // map -> odom stays put; odom -> base_link moves along x by 1 cm per frame.
        mapToOdom->timestampNs = nowNs;
        odomToBase->timestampNs = nowNs;
        odomToBase->transform.translation.x = (double) m_frames * 0.01;
    }

    auto *joints = dynamic_cast<QRB::Memory::JointStateBufferDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != joints )
    {
        joints->timestampNs = nowNs;
        for ( uint32_t i = 0; i < joints->jointCount; i++ )
        {
            QRB::Memory::JointState_t *joint = joints->GetJointPtr( i );
            if ( nullptr != joint )
            {
                // Revolute joints move by 0.01 rad per frame; effort remains zero.
                joint->position = (double) m_frames * 0.01 + (double) i;
                joint->velocity = 0.01 * (double) m_fps;
            }
        }
    }

    auto *poseStamped = dynamic_cast<QRB::Memory::PoseDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != poseStamped )
    {
        // Move along the x axis of "map" by 1 cm per frame.
        poseStamped->timestampNs = nowNs;
        poseStamped->pose.position.x = (double) m_frames * 0.01;
    }

    auto *poses = dynamic_cast<QRB::Memory::PoseArrayDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != poses )
    {
        poses->timestampNs = nowNs;
        // The first pose stays put; the second moves along x by 1 cm per frame.
        QRB::Memory::Pose_t *second = poses->GetPosePtr( 1 );
        if ( nullptr != second )
        {
            second->position.x = (double) m_frames * 0.01;
        }
    }

    auto *vector3 = dynamic_cast<QRB::Memory::Vector3Descriptor_t *>( &frame.Descriptor() );
    if ( nullptr != vector3 )
    {
        // Tilt from straight down towards x by 1 percent per frame, and back every 100 frames.
        vector3->timestampNs = nowNs;
        vector3->vector.x = (double) ( m_frames % 100 ) * 0.01;
    }

    auto *imu = dynamic_cast<QRB::Memory::ImuDescriptor_t *>( &frame.Descriptor() );
    if ( nullptr != imu )
    {
        // Constant gravity along -z; yaw rate cycles through 0..2π over 360 frames.
        imu->timestampNs = nowNs;
        imu->linear_acceleration.z = -9.8;
        imu->angular_velocity.z = (double) ( m_frames % 360 ) * 0.01745329;
    }
}

QCStatus_e SampleIpcAdapter::Start()
{
    QCStatus_e ret = m_node->Start();

    PrintState( "Start", ret );
    if ( QC_STATUS_OK == ret )
    {
        m_stop = false;
        m_thread = std::thread( &SampleIpcAdapter::ThreadMain, this );
    }
    else
    {
        QC_ERROR( "failed to start the IPC Adapter node: %d", ret );
    }

    return ret;
}

bool SampleIpcAdapter::GetMonitor( qrb::adapter::HcmIpcAdapterMonitoringData &monitor )
{
    uint32_t size = sizeof( monitor );
    return QC_STATUS_OK == m_node->GetMonitoringIfs().Place( &monitor, size );
}

void SampleIpcAdapter::PrintState( const char *call, QCStatus_e ret )
{
    printf( "%s %s returned %d, state %s\n", m_name.c_str(), call, (int) ret,
            StateName( m_node->GetState() ) );
}

void SampleIpcAdapter::PrintConfig()
{
    QCNodeConfigIfs &configIfs = m_node->GetConfigurationIfs();
    const QCNodeConfigBase_t &base = configIfs.Get();
    printf( "%s config: name=%s | type=%d | id=%u", m_name.c_str(), base.nodeId.name.c_str(),
            (int) base.nodeId.type, (unsigned) base.nodeId.id );
    auto *config = dynamic_cast<qrb::adapter::HcmIpcAdapterNodeConfig *>( &configIfs );
    if ( nullptr != config )
    {
        const qrb::adapter::HcmIpcAdapterConfig &adapter = config->Adapter();
        const qrb::adapter::FlowConfig &flow = adapter.flowConfig;
        printf( " | contract_type=%s | direction=%lld | ros_topic=%s | reliability=%s | "
                "durability=%s | depth=%u | global_buffer_id=%u",
                flow.contractTypeName.c_str(),
                (long long) qrb::adapter::ToDirectionCode( flow.direction ), flow.rosTopic.c_str(),
                qrb::adapter::ToString( flow.qosProfile.reliability ),
                qrb::adapter::ToString( flow.qosProfile.durability ), flow.qosProfile.depth,
                adapter.globalBufferId );
    }
    printf( "\n" );
}

void SampleIpcAdapter::PrintMonitor()
{
    qrb::adapter::HcmIpcAdapterMonitoringData monitor{};
    if ( GetMonitor( monitor ) )
    {
        printf( "%s monitor: sessionConnected=%u | frameCount=%llu | errorCount=%llu | "
                "lastLatencyNs=%llu | averageLatencyNs=%llu | minLatencyNs=%llu | "
                "maxLatencyNs=%llu | noDataCount=%llu\n",
                m_name.c_str(), (unsigned) monitor.sessionConnected,
                (unsigned long long) monitor.frameCount, (unsigned long long) monitor.errorCount,
                (unsigned long long) monitor.lastLatencyNs,
                (unsigned long long) monitor.averageLatencyNs,
                (unsigned long long) monitor.minLatencyNs,
                (unsigned long long) monitor.maxLatencyNs,
                (unsigned long long) monitor.noDataCount );
    }
}

QCStatus_e SampleIpcAdapter::Process( ContractFrame &frame )
{
    NodeFrameDescriptor frameDesc( m_bufferId + 1 );
    (void) frameDesc.SetBuffer( m_bufferId, frame.Descriptor() );

    QCStatus_e ret = m_node->ProcessFrameDescriptor( frameDesc );
    if ( QC_STATUS_OK != ret )
    {
        m_errorCount++;
        if ( m_errorCount <= kMaxReportedErrors )
        {
            QC_ERROR( "ProcessFrameDescriptor failed: %d", ret );
        }
    }
    else if ( false == m_isHcmToRos )
    {
        // A fetch that found no ROS message returns OK and counts in noDataCount.
        qrb::adapter::HcmIpcAdapterMonitoringData monitor{};
        if ( GetMonitor( monitor ) && ( monitor.noDataCount != m_noDataCount ) )
        {
            m_noDataCount = monitor.noDataCount;
            ret = QC_STATUS_TIMEOUT;
        }
    }
    if ( QC_STATUS_OK == ret )
    {
        m_frames++;
    }

    return ret;
}

void SampleIpcAdapter::SendFromTopic()
{
    while ( false == m_stop )
    {
        ContractFrameRef_t frame;
        if ( QC_STATUS_OK == m_sub.Receive( frame, kReceiveTimeoutMs ) )
        {
            (void) Process( *frame );
            // Hand the frame back to its pool.
            frame.reset();
        }
    }
}

void SampleIpcAdapter::FetchToTopic()
{
    while ( false == m_stop )
    {
        ContractFrameRef_t frame = m_pool.Take();
        if ( nullptr == frame )
        {
            m_poolExhausted++;
            std::this_thread::sleep_for( kPoolWait );
        }
        else
        {
            QCStatus_e ret = Process( *frame );
            if ( QC_STATUS_OK == ret )
            {
                (void) m_pub.Publish( frame );
            }
            frame.reset();
            if ( QC_STATUS_TIMEOUT == ret )
            {
                std::this_thread::sleep_for( kNoDataWait );
            }
        }
    }
}

void SampleIpcAdapter::RunStandalone()
{
    const auto period = std::chrono::nanoseconds( ( 0 == m_fps ) ? 0 : 1000000000ULL / m_fps );
    auto next = std::chrono::steady_clock::now();

    while ( false == m_stop )
    {
        if ( m_isHcmToRos )
        {
            UpdateFrame( m_frame );
        }
        (void) Process( m_frame );

        auto now = std::chrono::steady_clock::now();
        next += period;
        if ( next > now )
        {
            std::this_thread::sleep_until( next );
        }
        else
        {
            next = now;
        }
    }
}

void SampleIpcAdapter::ThreadMain()
{
    qrb::adapter::HcmIpcAdapterMonitoringData monitor{};
    if ( GetMonitor( monitor ) )
    {
        m_noDataCount = monitor.noDataCount;
    }

    if ( !m_inputTopic.empty() )
    {
        SendFromTopic();
    }
    else if ( !m_outputTopic.empty() )
    {
        FetchToTopic();
    }
    else
    {
        RunStandalone();
    }
}

QCStatus_e SampleIpcAdapter::Stop()
{
    m_stop = true;
    if ( m_thread.joinable() )
    {
        m_thread.join();
    }

    QCStatus_e ret = m_node->Stop();
    PrintState( "Stop", ret );
    if ( QC_STATUS_OK != ret )
    {
        QC_ERROR( "failed to stop the IPC Adapter node: %d", ret );
    }
    PrintMonitor();
    printf( "%s %s %llu frames, %llu errors, every frame held %llu times\n", m_name.c_str(),
            m_isHcmToRos ? "sent" : "received", (unsigned long long) m_frames,
            (unsigned long long) m_errorCount, (unsigned long long) m_poolExhausted );
    if ( ( QC_STATUS_OK == ret ) && ( 0 != m_errorCount ) )
    {
        ret = QC_STATUS_FAIL;
    }

    return ret;
}

QCStatus_e SampleIpcAdapter::Deinit()
{
    QCStatus_e ret = QC_STATUS_OK;

    if ( nullptr != m_node )
    {
        ret = m_node->DeInitialize();
        PrintState( "DeInitialize", ret );
        m_node.reset();
    }
    m_pool.Clear();

    return ret;
}

REGISTER_SAMPLE( IpcAdapter, SampleIpcAdapter );

}   // namespace sample
}   // namespace QC
