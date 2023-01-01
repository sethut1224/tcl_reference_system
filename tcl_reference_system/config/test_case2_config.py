import time

global margin
global reference_time_point

tcl_sched_config = {
'front_lidar_driver'    : {'type': 0, 'rate': 10, 'cpu': 1, 'phase': 0,    'priority': 98, 'blocking_topics':['']},
'point_cloud_fusion'    : {'type': 1, 'rate': 10, 'cpu': 1, 'phase': 1000000, 'priority': 98, 'blocking_topics':['/front_lidar/points_raw', '/rear_lidar/points_raw']},
'voxel_grid_filter'     : {'type': 1, 'rate': 10, 'cpu': 1, 'phase': 2000000, 'priority': 98, 'blocking_topics':['/lidars/fused_points']},
'ndt_localizer'         : {'type': 1, 'rate': 10, 'cpu': 1, 'phase': 3000000, 'priority': 98, 'blocking_topics':['/lidars/downsampled_points']},

'rear_lidar_driver'     : {'type': 0, 'rate': 10, 'cpu': 2, 'phase': 0,    'priority': 98, 'blocking_topics':['']},
'ray_ground_filter'     : {'type': 1, 'rate': 10, 'cpu': 2, 'phase': 2000000, 'priority': 98, 'blocking_topics':['/lidars/fused_points']},
'euclidean_clustering'  : {'type': 1, 'rate': 10, 'cpu': 2, 'phase': 3000000, 'priority': 98, 'blocking_topics':['/lidars/no_ground_points']},
}

tcl_timing_config = {
'front_lidar_driver'    : {'enable_profile': True},
'point_cloud_fusion'    : {'enable_profile': True},
'voxel_grid_filter'     : {'enable_profile': True},
'ndt_localizer'         : {'enable_profile': True},

'rear_lidar_driver'     : {'enable_profile': True},
'ray_ground_filter'     : {'enable_profile': True},
'euclidean_clustering'  : {'enable_profile': True},
}

margin = len(tcl_sched_config.keys()) * 500000000
reference_time_point = time.clock_gettime_ns(time.CLOCK_REALTIME) + margin

def get_tcl_sched_param(node_name):
    parameters=dict()
    base='tcl_sched_param.'
    for key in tcl_sched_config[node_name]:
        parameters[base+key] = tcl_sched_config[node_name][key]

    parameters[base+'ref_time_point'] = reference_time_point
    return parameters

def get_tcl_timing_param(node_name):
    parameters=dict()
    base='tcl_timing_param.'
    for key in tcl_timing_config[node_name]:
        parameters[base+key] = tcl_timing_config[node_name][key]

    return parameters