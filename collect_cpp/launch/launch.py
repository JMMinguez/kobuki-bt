# Copyright 2024 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('collect_cpp')

    convert_2d_3d = Node(package='camera',
      executable='detection_2d_to_3d_depth',
      output='screen',
      remappings=[
        ('input_depth', '/camera/depth/image_raw'),
        ('input_detection_2d', 'detection_2d'),
        ('camera_info', '/camera/camera_info'),
        ('output_detection_3d', 'detection_3d'),
      ]
    )

    yolo_cmd = Node(package='camera',
      executable='yolo_detection',
      output='screen',
      remappings=[
        ('output_detection_2d', 'detection_2d'),
        ('input_detection', '/yolo/detections'),
      ]
    )

    tf_cmd = Node(package='fp_cpp',
      executable='p2',
      output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(convert_2d_3d)
    ld.add_action(yolo_cmd)
    ld.add_action(tf_cmd)

    return ld
