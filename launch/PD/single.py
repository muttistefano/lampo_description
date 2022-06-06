import os
from os import environ
from os import pathsep
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import OpaqueFunction
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths


def generate_launch_description():


    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {'GAZEBO_MODEL_PATH': model,
           'GAZEBO_PLUGIN_PATH': plugin,
           'GAZEBO_RESOURCE_PATH': media}

    print(env)
    # sys.exit()

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
            "simulation_controllers",
            default_value=os.path.join(get_package_share_directory('ur_bringup'),'config/ur_controllers.yaml'),
            description="ur controllers file",
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
            "use_fake_hardware",
            default_value='true',
            description="use_fake_hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value='true',
            description="sim_gazebo",
        )
    )


    # Initialize Arguments
    ur_type                  = LaunchConfiguration("ur_type")
    safety_limits            = LaunchConfiguration("safety_limits")
    safety_pos_margin        = LaunchConfiguration("safety_pos_margin")
    safety_k_position        = LaunchConfiguration("safety_k_position")
    simulation_controllers   = LaunchConfiguration("simulation_controllers")
    
    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")
    prefix                   = LaunchConfiguration("prefix")
    frame_prefix             = LaunchConfiguration("frame_prefix")
    use_fake_hardware        = LaunchConfiguration("use_fake_hardware")
    sim_gazebo               = LaunchConfiguration("sim_gazebo")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ","safety_limits:=",safety_limits,
            " ","safety_pos_margin:=",safety_pos_margin,
            " ","safety_k_position:=",safety_k_position,
            " ","name:=","ur",
            " ","ur_type:=",ur_type,
            " ","prefix:=",prefix,
            " ","simulation_controllers:=",simulation_controllers,
            " ","use_fake_hardware:=",use_fake_hardware,
            " ","sim_gazebo:=",sim_gazebo,
        ]
    )


    robot_description  = {"robot_description": robot_description_content}
    frame_prefix_param = {"frame_prefix": LaunchConfiguration("frame_prefix")}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description,frame_prefix_param],
    )

    spawn_sweepee_1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'sw1', "-topic", "robot_description",
                                    #    "-robot_namespace",frame_prefix,
                                       "-x"," 1"],
                            output='screen')#TODO frame_prefix

    initial_joint_controllers = PathJoinSubstitution(
        [os.path.join(get_package_share_directory('lampo_description'), "config/ur_controllers.yaml")]
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,initial_joint_controllers ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_trajectory_controller", "-c", "controller_manager"],
    )


    nodes_to_start = [
        TimerAction(
            period=2.0,
            actions=[spawn_sweepee_1,robot_state_publisher_node],
        ),
        TimerAction(
            period=4.0,
            actions=[control_node,joint_state_broadcaster_spawner,initial_joint_controller_spawner_started],
        )
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

