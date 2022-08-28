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

# margin = 1000000000
margin = 0
reference_time_point = time.clock_gettime_ns(time.CLOCK_REALTIME) + margin

#timing_observation_topics = ['/front_lidar/points_raw', '/rear_lidar/points_raw', '/lidars/fused_points', '/lidars/downsampled_points', '/lidars/no_ground_points', '/detected_objects', '/ndt_pose', '/odom_raw', '/vehicle_status', '/ekf_pose', '/trajectory']

timing_observation_topics=['']
def generate_launch_description():
    os.sched_setaffinity ( 0 , [11])

    node_characteristics_param_path = os.path.join(
        get_package_share_directory('tcl_reference_system'),
        'param',
        'test_case2.param.yaml'
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
                    'tcl_sched_param.rate' : 5,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 90,
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    # 'tcl_timing_param.sub_timing_observation_topics' : ['']
                    'tcl_timing_param.pub_timing_observation_topics' : ['/front_lidar/points_raw'],
                    'tcl_timing_param.enable_profile' : True
                }
            ],
        )
    
    rear_lidar_driver = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_some', 
            name='rear_lidar_driver',
            output='screen', 
            parameters=[
                node_characteristics_param['rear_lidar_driver'],
                {
                    'tcl_sched_param.type': 0,
                    'tcl_sched_param.cpu' : 2,
                    'tcl_sched_param.rate' : 5,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 90,
                    # 'tcl_timing_param.sub_timing_observation_topics' : ['']
                    'tcl_timing_param.pub_timing_observation_topics' : ['/rear_lidar/points_raw'],
                    'tcl_timing_param.enable_profile' : True
                }
            ],
        )
    
    point_cloud_fusion = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='point_cloud_fusion',
            output='screen', 
            parameters=[
                node_characteristics_param['point_cloud_fusion'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 2,
                    'tcl_sched_param.rate' : 5,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 90,
                    'tcl_sched_param.blocking_topics' : ['/front_lidar/points_raw', '/rear_lidar/points_raw'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.sub_timing_observation_topics' : ['/front_lidar_driver/tcl_timing', '/rear_lidar_driver/tcl_timing'],
                    'tcl_timing_param.pub_timing_observation_topics' : ['/lidars/fused_points'],
                    'tcl_timing_param.enable_profile' : True
                }
            ],
        )
    
    voxel_grid_filter = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='voxel_grid_filter',
            output='screen', 
            parameters=[
                node_characteristics_param['voxel_grid_filter'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 1,
                    'tcl_sched_param.rate' : 5,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 90,
                    'tcl_sched_param.blocking_topics' : ['/lidars/fused_points'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.sub_timing_observation_topics' : ['/point_cloud_fusion/timing'],
                    'tcl_timing_param.pub_timing_observation_topics' : ['/lidars/downsampled_points'],
                }
            ],
        )
    
    ndt_localizer = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='ndt_localizer',
            output='screen', 
            parameters=[
                node_characteristics_param['ndt_localizer'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 1,
                    'tcl_sched_param.rate' : 5,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 90,
                    'tcl_sched_param.blocking_topics' : ['/lidars/downsampled_points'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.sub_timing_observation_topics' : ['/voxel_grid_filter/timing'],
                    'tcl_timing_param.pub_timing_observation_topics' : ['/ndt_pose'],
                }
            ],
        )

    ray_ground_filter = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='ray_ground_filter',
            output='screen', 
            parameters=[
                node_characteristics_param['ray_ground_filter'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 2,
                    'tcl_sched_param.rate' : 5,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 90,
                    'tcl_sched_param.blocking_topics' : ['/lidars/fused_points'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.sub_timing_observation_topics' : ['/point_cloud_fusion/timing'],
                    'tcl_timing_param.pub_timing_observation_topics' : ['/lidars/no_ground_points'],
                }
            ],
        )

    euclidean_clustering = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='euclidean_clustering',
            output='screen', 
            parameters=[
                node_characteristics_param['euclidean_clustering'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 2,
                    'tcl_sched_param.rate' : 5,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 90,
                    'tcl_sched_param.blocking_topics' : ['/lidars/no_ground_points'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.sub_timing_observation_topics' : ['/ray_ground_filter/timing'],
                    'tcl_timing_param.pub_timing_observation_topics' : ['/detected_objects'],
                }
            ],
        )

    odom_driver = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_some', 
            name='odom_driver',
            output='screen', 
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
        )

    vehicle_interface = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='vehicle_interface',
            output='screen', 
            parameters=[
                node_characteristics_param['vehicle_interface'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 3,
                    'tcl_sched_param.rate' : 20,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 98,
                    'tcl_sched_param.blocking_topics' : ['/odom_raw'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                }
            ],
        )

    ekf_localizer = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='ekf_localizer',
            output='screen', 
            parameters=[
                node_characteristics_param['ekf_localizer'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 3,
                    'tcl_sched_param.rate' : 20,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 98,
                    'tcl_sched_param.blocking_topics' : ['/vehicle_status'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                }
            ],
        )

    behavior_planner = launch_ros.actions.Node(
            package='tcl_reference_system', 
            executable='spin_node', 
            name='behavior_planner',
            output='screen', 
            parameters=[
                node_characteristics_param['behavior_planner'],
                {
                    'tcl_sched_param.type': 1,
                    'tcl_sched_param.cpu' : 3,
                    'tcl_sched_param.rate' : 20,
                    'tcl_sched_param.phase' : 0,
                    'tcl_sched_param.priority' : 98,
                    'tcl_sched_param.blocking_topics' : ['/ekf_pose'],
                    'tcl_sched_param.ref_time_point' : reference_time_point,
                    'tcl_timing_param.timing_observation_topics' : timing_observation_topics,
                }
            ],
        )


    return launch.LaunchDescription([
        front_lidar_driver,
        rear_lidar_driver,
        point_cloud_fusion,
        # voxel_grid_filter,
        # ray_ground_filter,
        # ndt_localizer,
        # euclidean_clustering,
        # odom_driver,
        # vehicle_interface,
        # ekf_localizer,
        # behavior_planner
    ])
