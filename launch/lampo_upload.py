# example.launch.py

import os
from os import environ
from os import pathsep

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from scripts import GazeboRosPaths


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
            default_value='false',
            description="sim_gazebo",
        )
    )

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


    gazebo_server = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('gazebo_ros'),
                        'launch/gazebo.launch.py')),
                        launch_arguments={'world': os.path.join(get_package_share_directory('lampo_description'),'worlds/world.world'),
                                          'verbose' : 'false' ,
                                          'pause' : 'true'}.items(),
            ),
        ]
    )
    



    sweepee_1_launch = GroupAction(
        actions=[
            PushRosNamespace('sweepee_1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('lampo_description'),
                        'launch/single.py')),
                        launch_arguments={'frame_prefix': 'sweepee_1/'}.items(),
            ),
        ]
    )

    # sweepee_2_launch = GroupAction(
    #     actions=[
    #         PushRosNamespace('sweepee_2'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('lampo_description'),
    #                     'launch/mobile_manip.py')),
    #                     launch_arguments={'frame_prefix': 'sweepee_2/'}.items(),
    #         ),
    #     ]
    # )


    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("lampo_description"), "rviz", "config.rviz"]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )


    return LaunchDescription([
        gazebo_server,
        sweepee_1_launch,
        # sweepee_2_launch,
        # rviz_node,
    ] + declared_arguments)