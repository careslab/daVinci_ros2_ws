import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch import LaunchContext, LaunchDescription, Substitution
from typing import Text
import numpy as np
from scipy.spatial.transform import Rotation as R

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    generation = LaunchConfiguration('generation')
    simulated = LaunchConfiguration('simulated', default = 'true')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    rate = LaunchConfiguration('rate', default = 50.0)

    # PSM transforms (PSM1 is treated as the base frame)
    
    # ECM static transform relative to PSM1
    ecm_tx_val = 0.16542699332099142
    ecm_ty_val = 0.07359911824517254
    ecm_tz_val = -0.03504927115331456
    ecm_roll = 0.8890035777338241
    ecm_pitch = 0.1393023305400441
    ecm_yaw = 2.5479808727935125
    
    ecm_rotation = R.from_euler('xyz', [ecm_roll, ecm_pitch, ecm_yaw], degrees=False)
    ecm_quat = ecm_rotation.as_quat()
    ecm_qx, ecm_qy, ecm_qz, ecm_qw = ecm_quat[0], ecm_quat[1], ecm_quat[2], ecm_quat[3]
    
    # PSM2 static transform relative to PSM1
    psm2_tx_val = 0.13592323
    psm2_ty_val = 0.15001368
    psm2_tz_val = -0.22357665
    roll = 1.54371639
    pitch = 0.6684686
    yaw = 2.33371598

    
    rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    quat = rotation.as_quat()
    psm2_qx, psm2_qy, psm2_qz, psm2_qw = quat[0], quat[1], quat[2], quat[3]
    
    print(quat)

    ld = LaunchDescription()


    # dVRK system
    system_json = [
        PathJoinSubstitution([FindPackageShare('dvrk_config'),
                              'system', '']),
        '/system-patient-cart-',
        generation,
        '-simulated.json'
    ]
    dvrk_node = Node(
        package = 'dvrk_robot',
        executable = 'dvrk_system',
        condition = IfCondition(simulated),
        arguments = ['-j', system_json],
        output = 'both',
    )
    ld.add_action(dvrk_node)


    # Note: SUJ nodes intentionally omitted in this copy

    # Publish PSM1 as base frame in the world
    psm1_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'PSM1_mounting_point'],
        output='both',
    )
    ld.add_action(psm1_base_tf)

    # Publish ECM mounting point relative to PSM1
    ecm_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[str(ecm_tx_val), str(ecm_ty_val), str(ecm_tz_val), str(ecm_qx), str(ecm_qy), str(ecm_qz), str(ecm_qw), 'world', 'ECM_mounting_point'],
        output='both',
    )
    ld.add_action(ecm_static_tf)

    # Publish PSM2 mounting point relative to PSM1
    psm2_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[str(psm2_tx_val), str(psm2_ty_val), str(psm2_tz_val), str(psm2_qx), str(psm2_qy), str(psm2_qz), str(psm2_qw), 'world', 'PSM2_mounting_point'],
        output='both',
    )
    ld.add_action(psm2_static_tf)

    # Arm joint/robot state publishers (attach arms to mounting points)
    for arm in ['ECM', 'PSM1', 'PSM2']:
        publisher_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('dvrk_model'),
                    'launch',
                    'arm_state_publishers.launch.py')),
            launch_arguments = {
                'arm': arm,
                'generation': generation,
                'use_sim_time': use_sim_time,
                'rate': rate,
                'suj': 'true'
            }.items()
        )
        ld.add_action(publisher_nodes)

    # RViz
    rviz_config_file = [
        PathJoinSubstitution([FindPackageShare('dvrk_model'),
                              'rviz', generation, '']),
        '/patient_cart.rviz'
    ]
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2_patient_cart',
        arguments = ['-d', rviz_config_file],
        output = 'both',
    )
    ld.add_action(rviz_node)

    return ld
