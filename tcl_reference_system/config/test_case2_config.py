import time

global margin
global reference_time_point

tcl_sched_config = {
'virtual_driver_vlp16_front' :          {'type':0, 'rate':10, 'cpu':1, 'phase': 100000000,    'priority':98, 'blocking_topics':['']},
'point_cloud_fusion' :                  {'type':1, 'rate':10, 'cpu':1, 'phase': 0,    'priority':98, 'blocking_topics':['/lidar_front/points_raw', '/lidar_rear/points_raw']},
'voxel_grid' :                          {'type':1, 'rate':10, 'cpu':1, 'phase': 0,    'priority':98, 'blocking_topics':['/lidars/points_fused']},
'ndt_localizer' :                       {'type':1, 'rate':10, 'cpu':1, 'phase': 0,    'priority':98, 'blocking_topics':['/lidars/points_downsampled']},

'virtual_driver_vlp16_rear' :           {'type':0, 'rate':10, 'cpu':2, 'phase': 100000000,    'priority':98, 'blocking_topics':['']},
'ray_ground_classifier' :               {'type':1, 'rate':10, 'cpu':2, 'phase': 0,    'priority':98, 'blocking_topics':['/lidars/points_fused']},
'euclidean_clustering' :                {'type':1, 'rate':10, 'cpu':2, 'phase': 0,    'priority':98, 'blocking_topics':['/lidars/points_nonground']},
}

tcl_timing_config = {
'virtual_driver_vlp16_front'    : {'enable_profile': True},
'point_cloud_fusion'            : {'enable_profile': True},
'voxel_grid'                    : {'enable_profile': True},
'ndt_localizer'                 : {'enable_profile': True},

'virtual_driver_vlp16_rear'     : {'enable_profile': True},
'ray_ground_classifier'         : {'enable_profile': True},
'euclidean_clustering'          : {'enable_profile': True},
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