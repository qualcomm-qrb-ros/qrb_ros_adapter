// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/manager/ros_bridge_manager.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

int main( int argc, char **argv )
{
    rclcpp::init( argc, argv );
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto manager = std::make_shared<qrb::adapter::RosBridgeManager>( executor );
    auto componentManager = std::make_shared<rclcpp_components::ComponentManager>(
        executor, "qrb_component_manager" );
    executor->add_node( manager );
    executor->add_node( componentManager );
    manager->Start();
    executor->spin();
    manager->Stop();
    executor->remove_node( manager );
    executor->remove_node( componentManager );
    rclcpp::shutdown();
    return 0;
}
