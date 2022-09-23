from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro
# from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    world = os.path.join(get_package_share_directory(
        'uthai_gazebo'), 'worlds', 'empty.world')

    DeclareLaunchArgument(
        'pause', default_value='false',
        description='Set "true" to start the server in a paused state.'
    ),
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world}.items()
    )

    # gazebo_custom_world = ExecuteProcess(
    #     cmd=['gazebo', world, '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so'],
    #     output='screen')

    package_path = os.path.join(
        get_package_share_directory('uthai_description')
    )

    xacro_file = os.path.join(package_path, 'urdf', 'uthai.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'uthai',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.764',
                   ],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # load_uthai_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'uthai_controller'],
    #     output='screen'
    # )

    # load_left_leg_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'uthai_l_leg_controller'],
    #     output='screen'
    # )

    # load_right_leg_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'uthai_r_leg_controller'],
    #     output='screen'
    # )

    individual_controllers = True

    if individual_controllers:
        controllers = ['l_hip_yaw_position', 'l_hip_roll_position', 'l_hip_pitch_position', 'l_knee_pitch_position', 'l_ankle_roll_position', 'l_ankle_pitch_position',
                       'r_hip_yaw_position', 'r_hip_roll_position', 'r_hip_pitch_position', 'r_knee_pitch_position', 'r_ankle_roll_position', 'r_ankle_pitch_position']
    else:
        controllers = ['uthai_controller']

    load_controllers = []
    for c in controllers:
        load_c = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 c],
            output='screen'
        )
        load_controllers.append(load_c)

    launch_nodes = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_controllers[0]],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ]

    if len(load_controllers) > 1:
        for i in range(1, len(load_controllers)-1):
            launch_nodes.append(RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_controllers[i-1],
                    on_exit=[load_controllers[i]],
                )
            ))
    return LaunchDescription([node for node in launch_nodes])
