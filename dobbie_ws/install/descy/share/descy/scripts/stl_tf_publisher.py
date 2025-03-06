#!/home/blazar/envs/gaussianprocess/bin/python3


import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
import numpy as np

class STLTransformPublisher(Node):
    def __init__(self):
        super().__init__('stl_tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Declare parameters for live tuning
        self.declare_parameter('base_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('base_rpy', [0.0, 0.0, 0.0])

        self.declare_parameter('first_xyz', [0.0, 0.0, 0.3])
        self.declare_parameter('first_rpy', [0.0, 0.0, 0.0])

        self.timer = self.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        # Read parameter values
        base_xyz = self.get_parameter('base_xyz').get_parameter_value().double_array_value
        base_rpy = self.get_parameter('base_rpy').get_parameter_value().double_array_value

        first_xyz = self.get_parameter('first_xyz').get_parameter_value().double_array_value
        first_rpy = self.get_parameter('first_rpy').get_parameter_value().double_array_value

        # Publish TF transforms
        self.publish_tf("world", "base", base_xyz, base_rpy)
        self.publish_tf("base", "first", first_xyz, first_rpy)

    def publish_tf(self, parent, child, xyz, rpy):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]

        # Convert Euler rpy to quaternion
        q = tf2_ros.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = STLTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
