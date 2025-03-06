#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tesseract_robotics.tesseract_environment as tesseract_env
import tesseract_robotics.tesseract_planning as tesseract_planning
import tesseract_robotics.tesseract_command_language as tesseract_cmd
import tesseract_robotics.tesseract_rosutils as tesseract_rosutils
import numpy as np
import os
import time

class TesseractPlanner(Node):
    def __init__(self):
        super().__init__('tesseract_planner')

        # Get paths
        workspace_path = os.path.expanduser("~/workspaces/dobbie_ws")
        urdf_path = os.path.join(workspace_path, "src/descy/urdf/dobbie.urdf.xacro")
        srdf_path = os.path.join(workspace_path, "src/dobbie_moveit_config/config/dobbie.srdf")

        # Load Tesseract Environment
        self.env = tesseract_env.Environment()
        if not self.env.init(urdf_path, srdf_path):
            self.get_logger().error("Failed to initialize Tesseract environment!")
            return
        
        self.get_logger().info("Tesseract environment initialized!")

        # Create a publisher for /joint_states
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Plan and execute motion
        self.plan_and_execute()

    def plan_and_execute(self):
        """ Plans a motion from 'home' to a target joint state and executes it. """
        # Home position (from SRDF)
        home_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # [joint1, joint2, joint3, joint4, joint5]

        # Target joint state (example goal)
        target_state = np.array([0.5, -0.3, 0.2, -0.4, 0.3])  # Change values to test other goals

        # Define start and goal states
        start_wp = tesseract_cmd.JointWaypoint(home_state, "control")  # SRDF group name
        goal_wp = tesseract_cmd.JointWaypoint(target_state, "control")

        # Create a joint trajectory
        program = tesseract_cmd.CompositeInstruction()
        program.append(tesseract_cmd.PlanInstruction(start_wp, tesseract_cmd.PlanInstructionType.START))
        program.append(tesseract_cmd.PlanInstruction(goal_wp, tesseract_cmd.PlanInstructionType.LINEAR))

        # Configure TrajOpt Planner
        request = tesseract_planning.PlanningRequest()
        request.instructions = program

        # Initialize TrajOpt planner
        planner = tesseract_planning.TrajOptMotionPlanner()
        response = planner.solve(request)

        if response.status.value != 0:
            self.get_logger().error("Trajectory planning failed!")
            return

        self.get_logger().info("Trajectory planned successfully! Executing...")

        # Execute the trajectory
        self.execute_trajectory(response.results)

    def execute_trajectory(self, trajectory):
        """ Publishes joint states to execute the trajectory smoothly. """
        rate = self.create_rate(10)  # 10Hz update rate
        for state in trajectory:
            joint_msg = JointState()
            joint_msg.header = Header()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
            joint_msg.position = state.position  # Extract joint positions

            self.joint_pub.publish(joint_msg)
            rate.sleep()

        self.get_logger().info("Trajectory execution complete!")


def main(args=None):
    rclpy.init(args=args)
    node = TesseractPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
