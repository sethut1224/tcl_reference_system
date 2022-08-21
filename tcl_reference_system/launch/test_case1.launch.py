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

margin = 0
reference_time_point = time.clock_gettime_ns(time.CLOCK_MONOTONIC) + margin

timing_observation_topics = ['/front_lidar/points_raw', '/front_lidar/downsampled_points', '/ndt_pose', '/odom_raw', '/vehicle_status', '/ekf_pose', '/trajectory']

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
                {
                    'tcl_sched_param.type': 0,
                    'tcl_sched_param.cpu' : 1,
                    'tcl_sched_param.rate' : 10,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 98,
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                }
            ],
        )

    """Generate launch description with multiple components."""
    front_lidar_container = ComposableNodeContainer(
        name='front_lidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        parameters=[
            {
                'tcl_sched_param.type': 1,
                'tcl_sched_param.cpu' : 1,
                'tcl_sched_param.rate' : 5,
                'tcl_sched_param.phase' : 0,
                'tcl_sched_param.priority' : 98,
                'tcl_sched_param.ref_time_point' : reference_time_point,
                'tcl_timing_param.timing_observation_topics' : timing_observation_topics
            }
        ],
        
        composable_node_descriptions=[
            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::TimerNode',
                name='front_lidar_driver',
                parameters=[
                    node_characteristics_param['front_lidar_driver'],
                    {
                        'tcl_sched_param.type': 0,
                        'tcl_sched_param.cpu' : 1,
                        'tcl_sched_param.rate' : 5,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                    }
                ], 
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::SpinNode',
                name='voxel_grid_filter',
                parameters=[
                    node_characteristics_param['voxel_grid_filter'],
                    {
                        'tcl_sched_param.type': 1,
                        'tcl_sched_param.cpu' : 1,
                        'tcl_sched_param.rate' : 5,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_sched_param.blocking_topics' : ['/front_lidar/points_raw'],
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::SpinNode',
                name='ndt_localizer',
                parameters=[
                    node_characteristics_param['ndt_localizer'],
                    {
                        'tcl_sched_param.type' : 1,
                        'tcl_sched_param.cpu' : 1,
                        'tcl_sched_param.rate' : 5,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_sched_param.blocking_topics' : ['/front_lidar/downsampled_points'],
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    rear_lidar_container = ComposableNodeContainer(
        name='rear_lidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        parameters=[
            {
                'tcl_sched_param.type': 0,
                'tcl_sched_param.cpu' : 2,
                'tcl_sched_param.rate' : 5,
                'tcl_sched_param.phase' : 0,
                'tcl_sched_param.priority' : 98,
                'tcl_sched_param.ref_time_point' : reference_time_point,
                'tcl_timing_param.timing_observation_topics' : timing_observation_topics
            }
        ],
        composable_node_descriptions=[
            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::TimerNode',
                name='rear_lidar_driver',
                parameters=[
                    node_characteristics_param['rear_lidar_driver'],
                    {
                        'tcl_sched_param.type': 1,
                        'tcl_sched_param.cpu' : 2,
                        'tcl_sched_param.rate' : 5,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            
            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::SpinNode',
                name='ray_ground_filter',
                parameters=[
                    node_characteristics_param['ray_ground_filter'],
                    {
                        'tcl_sched_param.type': 1,
                        'tcl_sched_param.cpu' : 2,
                        'tcl_sched_param.rate' : 5,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.blocking_topics' : ['/rear_lidar/points_raw'],
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::SpinNode',
                name='euclidean_clustering',
                parameters=[
                    node_characteristics_param['euclidean_clustering'],
                    {
                        'tcl_sched_param.type': 1,
                        'tcl_sched_param.cpu' : 2,
                        'tcl_sched_param.rate' : 5,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.blocking_topics' : ['/rear_lidar/no_ground_points'],
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )

    odom_container = ComposableNodeContainer(
        name='odom_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        parameters=[
            {
                'tcl_sched_param.type': 0,
                'tcl_sched_param.cpu' : 3,
                'tcl_sched_param.rate' : 20,
                'tcl_sched_param.phase' : 0,
                'tcl_sched_param.priority' : 98,
                'tcl_sched_param.ref_time_point' : reference_time_point,
                'tcl_timing_param.timing_observation_topics' : timing_observation_topics
            }
        ], 
        composable_node_descriptions=[
            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::TimerNode',
                name='odom_driver',
                parameters=[
                    node_characteristics_param['odom_driver'],
                    {
                        'tcl_sched_param.type': 0,
                        'tcl_sched_param.cpu' : 3,
                        'tcl_sched_param.rate' : 20,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                    }
                ], 
            ),
            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::SpinNode',
                name='vehicle_interface',
                parameters=[
                    node_characteristics_param['vehicle_interface'],
                    {
                        'tcl_sched_param.type': 1,
                        'tcl_sched_param.cpu' : 3,
                        'tcl_sched_param.rate' : 20,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_sched_param.blocking_topics' : ['/odom_raw'],
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                    }
                ], 
            ),
            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::SpinNode',
                name='ekf_localizer',
                parameters=[
                    node_characteristics_param['ekf_localizer'],
                    {
                        'tcl_sched_param.type' : 1,
                        'tcl_sched_param.cpu' : 3,
                        'tcl_sched_param.rate' : 20,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_sched_param.blocking_topics' : ['/vehicle_status'],
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                    }
                ]
            ),

            ComposableNode(
                package='tcl_reference_system',
                plugin='tcl_reference_system::SpinNode',
                name='behavior_planner',
                parameters=[
                    node_characteristics_param['behavior_planner'],
                    {
                        'tcl_sched_param.type' : 1,
                        'tcl_sched_param.cpu' : 3,
                        'tcl_sched_param.rate' : 20,
                        'tcl_sched_param.phase' : 0,
                        'tcl_sched_param.priority' : 98,
                        'tcl_sched_param.ref_time_point' : reference_time_point,
                        'tcl_sched_param.blocking_topics' : ['/ekf_pose'],
                        'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                    }
                ]
            ),
        ],
        output='screen',
    )
    # return launch.LaunchDescription([front_lidar_container] + [rear_lidar_container] + [odom_container])
    return launch.LaunchDescription([front_lidar_container])
