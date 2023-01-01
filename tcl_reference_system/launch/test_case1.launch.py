# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import time
import os
import yaml
import sys

sys.path.append(get_package_share_directory('tcl_reference_system')+'/config')
from test_case1_config import *


def generate_launch_description():
    os.sched_setaffinity ( 0 , [11])

    node_characteristics_param_path = os.path.join(
        get_package_share_directory('tcl_reference_system'),
        'param',
        'test_case1.param.yaml'
    )
    
    with open(node_characteristics_param_path, "r") as f:
        node_characteristics_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    front_lidar_driver = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_some', 
            name='front_lidar_driver',
            output='screen', 
            parameters=[
                node_characteristics_param['front_lidar_driver'],
                get_tcl_sched_param('front_lidar_driver'),
                get_tcl_timing_param('front_lidar_driver')
            ],
        )
    
    rear_lidar_driver = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_some', 
            name='rear_lidar_driver',
            output='screen', 
            parameters=[
                node_characteristics_param['rear_lidar_driver'],
                get_tcl_sched_param('rear_lidar_driver'),
                get_tcl_timing_param('rear_lidar_driver')
            ],
        )
    
    point_cloud_fusion = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_main', 
            name='point_cloud_fusion',
            output='screen', 
            parameters=[
                node_characteristics_param['point_cloud_fusion'],
                get_tcl_sched_param('point_cloud_fusion'),
                get_tcl_timing_param('point_cloud_fusion')
            ],
    )
    
    voxel_grid_filter = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_main', 
            name='voxel_grid_filter',
            output='screen', 
            parameters=[
                node_characteristics_param['voxel_grid_filter'],
                get_tcl_sched_param('voxel_grid_filter'),
                get_tcl_timing_param('voxel_grid_filter')
            ],
        )
    
    ndt_localizer = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_main', 
            name='ndt_localizer',
            output='screen', 
            parameters=[
                node_characteristics_param['ndt_localizer'],
                get_tcl_sched_param('ndt_localizer'),
                get_tcl_timing_param('ndt_localizer')
            ],
        )

    ray_ground_filter = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_main', 
            name='ray_ground_filter',
            output='screen', 
            parameters=[
                node_characteristics_param['ray_ground_filter'],
                get_tcl_sched_param('ray_ground_filter'),
                get_tcl_timing_param('ray_ground_filter')
            ],
        )

    euclidean_clustering = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_main', 
            name='euclidean_clustering',
            output='screen', 
            parameters=[
                node_characteristics_param['euclidean_clustering'],
                get_tcl_sched_param('euclidean_clustering'),
                get_tcl_timing_param('euclidean_clustering')
            ],
        )


    return launch.LaunchDescription([
        point_cloud_fusion,
        front_lidar_driver,
        rear_lidar_driver,
        voxel_grid_filter,
        ray_ground_filter,
        ndt_localizer,
        euclidean_clustering,
    ])
