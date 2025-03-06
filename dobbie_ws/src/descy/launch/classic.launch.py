#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to your 'descy' package
    descy_dir = get_package_share_directory("descy")

    # Launch argument for the URDF model
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(descy_dir, "urdf", "dobbie.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    # Convert the xacro to a string for robot_description
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    # Joint state publisher GUI node (optional for manual joint control in RViz)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(descy_dir, "rviz", "display.rviz")]
    )

    # Your custom GUI node
    gui_node = Node(
        package="descy",
        executable="gui.py",
        name="gui",
        output="screen"
    )

    # Gazebo Classic launch setup
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py"
            )
        ),
        launch_arguments={
            "verbose": "true",
            "world": os.path.join(descy_dir, "worlds", "empty.world")
        }.items()
    )

    # Include Gazebo client (GUI) if needed
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py"
            )
        )
    )

    # Spawn the robot in Gazebo Classic using spawn_entity.py from gazebo_ros
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "dobbie",
            "-topic", "robot_description"
        ],
        output="screen"
    )

    # Return the LaunchDescription with all these actions
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,  # Optional, can be removed
        rviz_node,
        gui_node,
        gazebo_launch,
        gazebo_client,  # Optional, can be removed if no GUI needed
        spawn_entity
    ])

if __name__ == '__main__':
    generate_launch_description()
