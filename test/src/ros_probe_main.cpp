// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

//
// RosStreamProbeNode in a process of its own, reaching the ROS IPC Bridge
// through DDS. Runs until the probe's `duration`, SIGINT or SIGTERM.
//
// Exit code: 0 ok, 1 a frame failed verification or belonged to another
// stream, 2 the probe could not start, 3 bad parameters.
//
// The only file it writes is `log`: ROS file logging (~/.ros/log) is off,
// the ROS log still goes to the console and /rosout.
//

#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

#include "ros_probe_node.hpp"

int main(int argc, char ** argv)
{
  for (int i = 1; i < argc && std::strcmp(argv[i], "--ros-args") != 0; ++i) {
    if (std::strcmp(argv[i], "--help") != 0 && std::strcmp(argv[i], "-h") != 0)
      continue;
    std::printf(
        "usage: ros_stream_probe_node [--ros-args -p NAME:=VALUE ...]\n"
        "\n"
        "The ROS side of a probed stream, in a process of its own (DDS to the bridge).\n"
        "Parameters come from the built-in defaults, then params_file (default\n"
        "share/qrb_ros_adapter_stream_probe/probe_configs/stream_probe.json), then -p:\n"
        "  params_file     JSON file of the parameters below\n"
        "  adapter_config  the Adapter's static JSON\n"
        "                  (probe_configs/adapter_point_cloud_h2r.json); the probe follows its\n"
        "                  direction (0: HCM to ROS, this side receives;\n"
        "                  1: ROS to HCM, this side sends), contract_type, ros_topic, qos_profile\n"
        "  fps             frames per second of a sender, 0: unlimited (30)\n"
        "  payload_bytes   payload of each frame this side sends (1048576)\n"
        "  buffer_bytes    DMA buffer size of the HCM receiver (R2H); ignored on this side\n"
        "  duration        seconds to run, 0: until a signal (0)\n"
        "  warmup          seconds a receiver does not measure (0)\n"
        "  log             text log, empty: none: a line per frame sent or received,\n"
        "                  a monitor line every 5 s, events and the summary; the only file\n"
        "                  this process writes (no ~/.ros/log)\n"
        "\n"
        "The same node loads into the bridge's container (zero-copy):\n"
        "  ros2 component load /qrb_ros_adapter_container qrb_ros_adapter_stream_probe \\\n"
        "      qrb::adapter::stream_probe::RosStreamProbeNode -p NAME:=VALUE\n"
        "Exit code: 0 ok, 1 verification failed or another stream, 2 runtime error,\n"
        "3 bad parameters.\n");
    return 0;
  }

  std::vector<const char *> args(argv, argv + argc);
  args.insert(args.end(), {"--ros-args", "--disable-external-lib-logs"});
  rclcpp::init(static_cast<int>(args.size()), args.data());
  std::shared_ptr<qrb::adapter::stream_probe::RosStreamProbeNode> node;
  int exitCode = 0;
  try {
    node = std::make_shared<qrb::adapter::stream_probe::RosStreamProbeNode>(rclcpp::NodeOptions());
  } catch (const std::invalid_argument & error) {
    std::fprintf(stderr, "ros_stream_probe_node: %s\n", error.what());
    exitCode = 3;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "ros_stream_probe_node: %s\n", error.what());
    exitCode = 2;
  }

  if (node != nullptr) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->Done()) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    node->Finish("signal");
    executor.remove_node(node);
    exitCode = node->ExitCode();
    node.reset();
  }
  rclcpp::shutdown();
  return exitCode;
}
