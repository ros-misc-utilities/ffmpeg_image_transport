# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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


import launch
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig


def launch_setup(context, *args, **kwargs):
    container = ComposableNodeContainer(
        name="republisher_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_transport",
                plugin="image_transport::Republisher",
                namespace=LaunchConfig("camera"),
                name="republisher",
                parameters=[
                    {
                        "in_transport": LaunchConfig("in_transport"),
                        "out_transport": LaunchConfig("out_transport"),
                        "image_raw.ffmpeg.decoders.h264": "h264,h264_cuvid,h264_qsv,h264_v4l2m2m",
                        "image_raw.ffmpeg.decoders.hevc": "hevc_cuvid,hevc_qsv,hevc_v4l2m2m,hevc",
                        "image_raw.ffmpeg.decoders.av1": "libaom-av1,av1_cuvid,av1,av1_qs,libdav1d",
                        "image_raw.ffmpeg.decoder_measure_performance": False,
                    }
                ],
                # remap the 'in' topic to the topic under which the
                # uncompressed image is published. The republisher
                # node will automatically append the in_transport
                # (here 'ffmpeg') to the topic.
                remappings=[
                    ("in", LaunchConfig("image_topic")),
                    ("out", LaunchConfig("image_topic").perform(context) + "/decoded"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="screen",
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            LaunchArg(
                "image_topic",
                default_value=["image_raw"],
                description="name of topic to republish "
                "(without the /camera/  prefix and the /ffmpeg suffix)",
            ),
            LaunchArg(
                "camera",
                default_value=["/camera"],
                description="name of camera (base of topics)",
            ),
            LaunchArg(
                "in_transport",
                default_value=["ffmpeg"],
                description="name of the input transport, e.g. ffmpeg, raw, compressed",
            ),
            LaunchArg(
                "out_transport",
                default_value=["raw"],
                description="name of the output transport, e.g. ffmpeg, raw, compressed",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
