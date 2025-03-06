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
    # Get the package share directory for descy
    descy_share = get_package_share_directory("descy")

    # Declare a launch argument for the robot's Xacro file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(descy_share, "urdf", "dobbie.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can locate resources if needed
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(descy_share).parent.resolve())]
    )

    # Process the Xacro file to generate the robot description
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Launch robot_state_publisher (using simulation time)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Launch the joint_state_publisher_gui (optional but helpful for modifying joints)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    # Include the Gazebo Sim launch file (empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ]),
        launch_arguments={"gz_args": " -v 4 -r empty.sdf "}.items()
    )

    # Spawn the robot into Gazebo using the robot description from the parameter server
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "dobbie"]
    )

    # Bridge the /clock topic so that ROS2 receives simulation time from Gazebo
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    # Launch RViz2 with your preconfigured display file (adjust path if necessary)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(descy_share, "rviz", "display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        rviz_node
    ])

