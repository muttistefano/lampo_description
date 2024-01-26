import os
from os import environ
from os import pathsep
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import OpaqueFunction
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from scripts import GazeboRosPaths
import launch_ros.descriptions

import xacro



def generate_launch_description():

    declared_arguments = []
    # UR specific arguments

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur10",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_prefix",
            default_value="",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(

        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="lampo_description",
            description="mobile manip description",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="system.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value='true',
            description="use_mock_hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mm",
            default_value='true',
            description="mobile manipulators",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gz",
            default_value='true',
            description="sim_gz",
        )
    )

    # Initialize Arguments
    ur_type                  = LaunchConfiguration("ur_type")
    safety_limits            = LaunchConfiguration("safety_limits")
    safety_pos_margin        = LaunchConfiguration("safety_pos_margin")
    safety_k_position        = LaunchConfiguration("safety_k_position")
    # simulation_controllers   = LaunchConfiguration("simulation_controllers")
    
    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")
    prefix                   = LaunchConfiguration("prefix")
    frame_prefix             = LaunchConfiguration("frame_prefix")
    use_mock_hardware        = LaunchConfiguration("use_mock_hardware")
    mm                       = LaunchConfiguration("mm")
    sim_gz                   = LaunchConfiguration("sim_gz")

    initial_joint_controllers_1 = PathJoinSubstitution(
        [
            FindPackageShare("lampo_description"),
            "config",
            "ur_controllers_1.yaml",
        ]
    )

    initial_joint_controllers_2 = PathJoinSubstitution(
        [
            FindPackageShare("lampo_description"),
            "config",
            "ur_controllers_2.yaml",
        ]
    )

    initial_joint_controllers_3 = PathJoinSubstitution(
        [
            FindPackageShare("lampo_description"),
            "config",
            "ur_controllers_2.yaml",
        ]
    )


    robot_description_content_1 = launch_ros.descriptions.ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ","safety_limits:=",safety_limits,
            " ","safety_pos_margin:=",safety_pos_margin,
            " ","safety_k_position:=",safety_k_position,
            " ","name:=","mm1",
            " ","ur_type:=",ur_type,
            " ","prefix:=","sweepee_1/",
            " ","prefix_rc:=","sweepee_1",
            " ","simulation_controllers:=",initial_joint_controllers_1,
            " ","use_mock_hardware:=",use_mock_hardware,
            " ","mm:=",mm,
            " ","sim_gz:=",sim_gz,
        ]), value_type=str)

    robot_description_content_2 = launch_ros.descriptions.ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ","safety_limits:=",safety_limits,
            " ","safety_pos_margin:=",safety_pos_margin,
            " ","safety_k_position:=",safety_k_position,
            " ","name:=","mm2",
            " ","ur_type:=",ur_type,
            " ","prefix:=","sweepee_2/",
            " ","prefix_rc:=","sweepee_2",
            " ","simulation_controllers:=",initial_joint_controllers_2,
            " ","use_mock_hardware:=",use_mock_hardware,
            " ","mm:=",mm,
            " ","sim_gz:=",sim_gz,
        ]), value_type=str)
    
    robot_description_content_3 = launch_ros.descriptions.ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ","safety_limits:=",safety_limits,
            " ","safety_pos_margin:=",safety_pos_margin,
            " ","safety_k_position:=",safety_k_position,
            " ","name:=","mm3",
            " ","ur_type:=",ur_type,
            " ","prefix:=","sweepee_3/",
            " ","prefix_rc:=","sweepee_3",
            " ","simulation_controllers:=",initial_joint_controllers_3,
            " ","use_mock_hardware:=",use_mock_hardware,
            " ","mm:=",mm,
            " ","sim_gz:=",sim_gz,
        ]), value_type=str)
    

    robot_description_1  = {"robot_description": robot_description_content_1}
    robot_description_2  = {"robot_description": robot_description_content_2}
    robot_description_3  = {"robot_description": robot_description_content_3}



    frame_prefix_param_1 = {"frame_prefix": ""}
    frame_prefix_param_2 = {"frame_prefix": ""}
    frame_prefix_param_3 = {"frame_prefix": ""}
    
    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sweepee_1",
        output="screen",
        parameters=[robot_description_1,frame_prefix_param_1,{"use_sim_time": True}],
    )

    robot_state_publisher_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sweepee_2",
        output="screen",
        parameters=[robot_description_2,frame_prefix_param_2,{"use_sim_time": True}],
    )

    robot_state_publisher_node_3 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sweepee_3",
        output="screen",
        parameters=[robot_description_3,frame_prefix_param_3],
    )

    spawn_sweepee_1 = Node(
        name='spawn1',
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[ '-topic', 'sweepee_1/robot_description',
                   '-name', 'sweepee_1',
                   '-allow_renaming', 'true',
                   '-x', '-3.5',
                   '-y', '2.2',
                   '-Y', '0.3'],
        remappings=[('/sweepee', 'sweepee_1/robot_description')],
        parameters=[{"use_sim_time": True}],
    )

    spawn_sweepee_2 = Node(
        name='spawn2',
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'sweepee_2/robot_description',
                   '-name', 'sweepee_2',
                   '-allow_renaming', 'true',
                   '-x', '-3.0',
                   '-y', '-2.0'],
        remappings=[('/sweepee', 'sweepee_2/robot_description')],
        parameters=[{"use_sim_time": True}],
    )

########## CONTROLLERS

    sw1_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="sweepee_1",
        parameters=[robot_description_1, initial_joint_controllers_1,{"use_sim_time": True}],
        output="both",
    )

    sw2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="sweepee_2",
        parameters=[robot_description_2, initial_joint_controllers_2,{"use_sim_time": True}],
        output="both",
    )

    joint_state_broadcaster_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "sweepee_1/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    position_controller_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "sweepee_1/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "sweepee_2/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    position_controller_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "sweepee_2/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )



########## VISUALIZATION

    world_path = os.path.join(get_package_share_directory('lampo_description'),'worlds/warehouse_big.sdf')

    gazebo_server = GroupAction(
        actions=[
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments={
                'gz_args': [' -r -v 4 ' + world_path ],
                'gz_version': "8"
            }.items())
            ]
            )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("lampo_description"), "rviz", "config2.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_file],
    )


    tf_sw1 = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "sweepee_1/odom", "sweepee_1/base_footprint"],
            parameters=[{"use_sim_time": True}]
        )

    tf_sw2 = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "sweepee_2/odom", "sweepee_2/base_footprint"],
            parameters=[{"use_sim_time": True}]
        )

    tf_sw3 = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "sweepee_3/odom"]
        )

########## BRIDGE

    config_bridge = os.path.join(get_package_share_directory('lampo_description'),
                              'config', 'bridge.yaml')
    
    config_param  = {"config_file": config_bridge}

    gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen" ,
            parameters=[config_param,{"use_sim_time": True}],
        )

    ros_to_gz_1 = Node(
            package="lampo_description",
            executable="ros_to_gz_commands.py",
            namespace="sweepee_1",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )

    ros_to_gz_2 = Node(
            package="lampo_description",
            executable="ros_to_gz_commands.py",
            namespace="sweepee_2",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
    
########## LAUNCHING


    nodes_to_start = [
        gazebo_server,
        rviz_node,
        TimerAction(
            period=2.0,
            actions=[spawn_sweepee_1,robot_state_publisher_node_1],
        ),
        TimerAction(
            period=4.0,
            actions=[spawn_sweepee_2,robot_state_publisher_node_2],
        ),
        TimerAction(
            period=6.0,
            actions=[sw1_control_node,sw2_control_node,ros_to_gz_1,ros_to_gz_2]
        ),
        TimerAction(
            period=8.0,
            actions=[joint_state_broadcaster_spawner_1,
                     position_controller_spawner_1]
        ),
        TimerAction(
            period=10.0,
            actions=[joint_state_broadcaster_spawner_2,
                     position_controller_spawner_2]
        ),
        # TimerAction(
        #     period=18.0,
        #     actions=[joint_state_broadcaster_spawner_2,initial_joint_controller_spawner_started_2]
        # ),
        # TimerAction(
        #     period=7.0,
        #     actions=[tf_sw1,tf_sw2,tf_sw3],#rviz_node
        # ),
        # TimerAction(
        #     period=14.0,
        #     actions=[map_server,nav_sw1],
        # )
        TimerAction(
            period=7.0,
            actions=[gz_bridge]
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


