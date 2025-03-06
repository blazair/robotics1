import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    descy_package = get_package_share_directory("descy")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(descy_package, "urdf", "dobbie.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(descy_package).parent.resolve())]
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    rviz_config_file = os.path.join(descy_package, "rviz", "display.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf "])
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "dobbie"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    gui_node = Node(
        package="descy",
        executable="gui.py",
        name="gui",
        output="screen"
    )

    # ROS2 Control Setup:
    control_yaml = os.path.join(descy_package, "config", "controller.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, control_yaml],
        output="screen"
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller"],
        output="screen"
    )

    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dobot_trajectory_controller"],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        gui_node,
        ros2_control_node,
        joint_state_spawner,
        trajectory_spawner,
    ])


if __name__ == '__main__':
    generate_launch_description()
