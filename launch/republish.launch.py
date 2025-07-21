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


def launch_setup(context, *args, **kwargs):
    container = ComposableNodeContainer(
        name='republisher_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_transport',
                plugin='image_transport::Republisher',
                namespace='/camera',
                name='republisher',
                parameters=[
                    {
                        'in_transport': 'ffmpeg',
                        'out_transport': 'raw',
                        '.image_raw.ffmpeg.map.hevc': 'hevc_cuvid,hevc_qsv,hevc_v4l2m2m,hevc',
                        '.image_raw.ffmpeg.map.h264': 'h264_cuvid,h264_qsv,h264_v4l2m2m,h264',
                        '.image_raw.ffmpeg.map.hevc': 'hevc_cuvid,hevc_qsv,hevc_v4l2m2m,hevc',
                        '.image_raw.ffmpeg.map.av1': 'av1_cuvid,av1,av1_qs,libaom-av1,libdav1d',
                        '.image_raw.ffmpeg.measure_performance': False,
                    }
                ],
                # remap the 'in' topic to the topic under which the
                # uncompressed image is published. The republisher
                # node will automatically append the in_transport
                # (here 'ffmpeg') to the topic.
                remappings=[
                    ('in', '/camera/image_raw'),
                    ('out', '/camera/image_raw/decoded'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
