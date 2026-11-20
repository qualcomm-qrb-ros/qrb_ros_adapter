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
 * there; without, it sends a frame it fills itself (FillFrame(), UpdateFrame()) every 1/fps
 * second.
 * ROS to HCM (direction 1): it fetches the latest ROS message into a frame; with output_topic,
 * it publishes every frame fetched there, and without, it fetches every 1/fps second.
 *
 * It prints the node state after each lifecycle call, the configuration read back through
 * GetConfigurationIfs() after Initialize(), and the monitoring and frame counts after Stop().
 *
 * Attributes:
 *   adapter_config: (optional) a file with the Adapter's static config; it replaces the six
 *                   Flow attributes below
 *   contract_type : (optional) a Contract full name listed in kContracts, default
 *                   qrb.contract.sensor.v1.PointCloud2
 *   direction     : (optional) 0: HCM to ROS (default), 1: ROS to HCM
 *   ros_topic     : (optional) default the Contract's topic in kContracts
 *   reliability   : (optional) reliable or best_effort, default the ROS default
 *   durability    : (optional) volatile or transient_local, default the ROS default
 *   depth         : (optional) 1..32, default the ROS default
 *   input_topic   : (optional) direction 0: the topic of the frames to send
 *   output_topic  : (optional) direction 1: the topic to publish the fetched frames on
 *   pool_size     : (optional) frames to fetch into with output_topic, 1..32, default 4
 *   fps           : (optional) frames per second, 0 for no pause, default 30
 *   payload_bytes : (optional) DMA-BUF payload sent to ROS, default 1048576
 *   buffer_bytes  : (optional) DMA-BUF buffer received from ROS, default 33554432
 *
 * Inline Contracts ignore payload_bytes and buffer_bytes.
 */

#include "ContractFrame.hpp"
#include "QC/Node/Ifs/QCNodeFactory.hpp"
#include "QC/Node/NodeFrameDescriptor.hpp"
#include "QC/sample/DataBroker.hpp"
#include "QC/sample/SampleIF.hpp"
#include "QRB/Memory/Geometry/PoseDescriptor.hpp"
#include "QRB/Memory/Navigation/OdometryDescriptor.hpp"
#include "QRB/Memory/Sensor/PointCloudDescriptor.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_node.hpp"

#include <atomic>
#include <chrono>
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

    /**
     * The Contracts the sample can send or fetch, with their default ROS topics. To add one,
     * add it here and give FillFrame() / UpdateFrame() a case for its descriptor.
     */
    struct SampleContract
    {
        const char *type;
        const char *defaultTopic;
    };
    static constexpr const char *kPointCloudContract = "qrb.contract.sensor.v1.PointCloud2";
    static constexpr SampleContract kContracts[] = {
            { kPointCloudContract, "/qrb_ros_adapter/sample/points" },
            { "qrb.contract.geometry.v1.PoseWithCovarianceStamped",
              "/qrb_ros_adapter/sample/pose" },
            { "qrb.contract.nav.v1.Odometry", "/qrb_ros_adapter/sample/odom" },
    };
    static const SampleContract *FindContract( const std::string &type );

    static constexpr uint32_t kPointStep = 3 * sizeof( float );
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
    m_contract = node.Get<std::string>( "contract_type", "" );
    m_isHcmToRos = ( 0 == node.Get<uint32_t>( "direction", 0 ) );
    std::vector<DataTree> map;
    if ( ( QC_STATUS_OK == node.Get( "globalBufferIdMap", map ) ) && ( false == map.empty() ) )
    {
        m_bufferId = map[0].Get<uint32_t>( "id", 0 );
    }

    return QC_STATUS_OK;
}

const SampleIpcAdapter::SampleContract *SampleIpcAdapter::FindContract( const std::string &type )
{
    for ( const SampleContract &contract : kContracts )
    {
        if ( type == contract.type )
        {
            return &contract;
        }
    }
    return nullptr;
}

std::string SampleIpcAdapter::MakeAdapterConfig( SampleConfig_t &config )
{
    // Init() has checked that the Contract is in kContracts.
    std::string defaultTopic = FindContract( m_contract )->defaultTopic;
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
        if ( nullptr == FindContract( m_contract ) )
        {
            QC_ERROR( "unsupported contract_type: %s", m_contract.c_str() );
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

    uint32_t bytes = m_isHcmToRos ? m_payloadBytes : m_bufferBytes;
    if ( m_isPointCloud && m_isHcmToRos && ( bytes < kPointStep ) )
    {
        QC_ERROR( "payload_bytes must hold at least one point: %u", bytes );
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
    auto *pose = dynamic_cast<QRB::Memory::PoseDescriptor_t *>( &frame.Descriptor() );
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

    auto *pose = dynamic_cast<QRB::Memory::PoseDescriptor_t *>( &frame.Descriptor() );
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
