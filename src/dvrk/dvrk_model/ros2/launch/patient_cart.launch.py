import os
import math
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch import LaunchContext, LaunchDescription, Substitution
from typing import Text

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
    rate = LaunchConfiguration('rate', default = 50.0)  # Hz, default is 10 so we're increasing that a bit.  Funny enough joint and robot state publishers don't have the same name for that parameter :-(

    ld = LaunchDescription()


    # dVRK system
    system_json = [
        PathJoinSubstitution([FindPackageShare('dvrk_config'),
                              'system', '']),
        'system-patient-cart-',
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


    # SUJ
    model = PathJoinSubstitution([
        FindPackageShare('dvrk_model'),
        'urdf', 
        generation, 
        'SUJ.urdf.xacro'
    ])

    description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), 
            ' ', 
            model
        ]),
        value_type=str,
    )
    pi = math.pi
    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    namespace='SUJ',
    executable='joint_state_publisher',
    name='SUJ_joint_state_publisher',
    parameters=[{
        'use_sim_time': use_sim_time,
        'rate': rate,
        'zeros': {
            'SUJ_ECM_J0': 0.66,  
            'SUJ_ECM_J1': pi/4,
            'SUJ_ECM_J2': -pi/2,
            'SUJ_ECM_J3': -(3/4)*pi,
        },
    }],
    output='both',
)
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        namespace = 'SUJ',
        executable = 'robot_state_publisher',
        name = 'SUJ_robot_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'robot_description': description,
                       'publish_frequency': rate}],
        output = 'both',
    )
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)


    # Arm joint/robot state publishers
    for arm in ['ECM', 'PSM1', 'PSM2', 'PSM3']:
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
        '/dvrk_patient_cart.rviz'
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
