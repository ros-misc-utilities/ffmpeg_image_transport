# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration as LaunchConfig


def launch_setup(context, *args, **kwargs):
    params_path = PathJoinSubstitution(
        [FindPackageShare("usb_cam"), "config", "params_1.yaml"]
    )
    node_name = "usb_cam"
    params = {
        "av1": {
            "image_raw.ffmpeg.qmax": 1,  # high quality
            # '.image_raw.ffmpeg.bit_rate': 1000000,
            "image_raw.ffmpeg.gop_size": 1,
            "image_raw.ffmpeg.encoder": "librav1e",  # very high CPU usage!!!
        },
        "hevc": {
            "image_raw.ffmpeg.qmax": 1,  # high quality
            # '.image_raw.ffmpeg.bit_rate': 1000000,
            "image_raw.ffmpeg.gop_size": 1,
            "image_raw.ffmpeg.encoder": "hevc_vaapi",
        },
        "h264": {
            "image_raw.ffmpeg.qmax": 1,  # high quality
            "image_raw.ffmpeg.bit_rate": 1000000,  # required!
            "image_raw.ffmpeg.gop_size": 1,
            "image_raw.ffmpeg.encoder": "libx264",
        },
    }
    node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output="screen",
        namespace="camera",
        name=node_name,
        parameters=[
            params_path,
            params[LaunchConfig("codec").perform(context)],
        ],
    )
    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                "codec",
                default_value=["h264"],
                description="name of libav (ffmpeg) encoder to use",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
