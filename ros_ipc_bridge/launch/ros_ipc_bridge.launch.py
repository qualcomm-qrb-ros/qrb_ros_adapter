# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("qrb_ros_adapter"),
        "config",
        "ros_ipc_bridge.yaml",
    )
    return LaunchDescription([
        Node(
            package="qrb_ros_adapter",
            executable="ros_ipc_bridge",
            parameters=[config],
            output="screen",
        )
    ])
