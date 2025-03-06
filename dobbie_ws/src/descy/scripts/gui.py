#!/usr/bin/env python3
import math
import numpy as np
import tkinter as tk
import sympy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

# Robot parameters (in mm)
a1 = 150.0
a2 = 150.0
a3 = 90.0

def rotation_z(theta):
    """Rotation about z-axis."""
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])

def H12(t1):
    # H12 defined as in original code.
    return np.array([[0, math.cos(t1), -math.sin(t1), 0],
                     [0, math.sin(t1),  math.cos(t1), 0],
                     [1,          0,           0,  0],
                     [0,          0,           0,  1]])

def H23(t2):
    return np.array([[math.cos(t2), -math.sin(t2), 0, a1*math.cos(t2)],
                     [math.sin(t2),  math.cos(t2), 0, a1*math.sin(t2)],
                     [0,             0,            1, 0],
                     [0,             0,            0, 1]])

def H34(t3):
    return np.array([[math.cos(t3), -math.sin(t3), 0, -a2*math.sin(t3)],
                     [math.sin(t3),  math.cos(t3), 0,  a2*math.cos(t3)],
                     [0,             0,            1, 0],
                     [0,             0,            0, 1]])

def H45(t4):
    # t4 is fixed (non‐controllable); computed as -th3.
    return np.array([[-math.sin(t4), 0, math.cos(t4), -a3*math.sin(t4)],
                     [ math.cos(t4), 0, math.sin(t4),  a3*math.cos(t4)],
                     [0,             1,          0,  0],
                     [0,             0,          0,  1]])

def H56(t5):
    return np.array([[math.cos(t5), -math.sin(t5), 0, 0],
                     [math.sin(t5),  math.cos(t5), 0, 0],
                     [0,             0,            1, 0],
                     [0,             0,            0, 1]])

def forward_kinematics(th1_deg, th2_deg, th3_deg, th5_deg):
    """
    Compute the forward kinematics.
    Inputs are joint angles (in degrees).
    Effective angles (in radians) are computed as:
       t1 = th1
       t2 = th2
       t3 = -th2 + th3
       t4 = -th3  (fixed, non-controllable)
       t5 = th5
    """
    th1 = math.radians(th1_deg)
    th2 = math.radians(th2_deg)
    th3 = math.radians(th3_deg)
    th5 = math.radians(th5_deg)
    
    t1 = th1
    t2 = th2
    t3_ = -th2 + th3
    t4 = -th3
    t5_ = th5

    H_12 = H12(t1)
    H_23 = H23(t2)
    H_34 = H34(t3_)
    H_45 = H45(t4)
    H_56 = H56(t5_)

    H16 = H_12 @ H_23 @ H_34 @ H_45 @ H_56
    position = H16[0:3, 3]
    return H16, position

def dobot_lite_ik(xd, yd, zd, guess=(10, 10, 10)):
    """
    Compute inverse kinematics using a symbolic approach.
    The solver finds joint angles (in degrees) th1, th2, and th3 that satisfy:
       H16(x, y, z) = [xd, yd, zd]
    The effective angles are defined as:
       t1 = th1 * pi/180
       t2 = th2 * pi/180
       t3 = (-th2 + th3) * pi/180
       t4 = -th3 * pi/180  (hence Theta4 = -th3 in degrees)
       t5 = 0  (fixed wrist rotation)
    Link lengths (mm) are fixed: a1 = 150, a2 = 150, a3 = 90.
    Returns [Theta1, Theta2, Theta3, Theta4, Theta5] in degrees.
    """
    # Define symbolic variables for th1, th2, th3 (in degrees)
    th1, th2, th3 = sympy.symbols('th1 th2 th3', real=True)
    # Fixed wrist rotation (Theta5) is 0 degrees.
    th4_val = 0  # This value is used for t5 (wrist rotation) in the forward kinematics.
    
    # Compute effective joint angles (in radians)
    t1 = th1 * sympy.pi / 180
    t2 = th2 * sympy.pi / 180
    t3_eff = (-th2 + th3) * sympy.pi / 180
    t4 = -th3 * sympy.pi / 180
    t5 = th4_val * sympy.pi / 180  # wrist rotation

    # Link lengths (mm)
    a1_sym = 150
    a2_sym = 150
    a3_sym = 90
    
    # H12: From base to joint 2
    H12_sym = sympy.Matrix([
        [0,           sympy.cos(t1), -sympy.sin(t1),  0],
        [0,           sympy.sin(t1),  sympy.cos(t1),  0],
        [1,                    0,              0,    0],
        [0,                    0,              0,    1]
    ])
    
    # H23: Joint 2 to joint 3
    H23_sym = sympy.Matrix([
        [sympy.cos(t2), -sympy.sin(t2),  0, a1_sym * sympy.cos(t2)],
        [sympy.sin(t2),  sympy.cos(t2),  0, a1_sym * sympy.sin(t2)],
        [0,                     0,       1,                0],
        [0,                     0,       0,                1]
    ])
    
    # H34: Joint 3 to joint 4 (elbow)
    H34_sym = sympy.Matrix([
        [sympy.cos(t3_eff), -sympy.sin(t3_eff),  0, -a2_sym * sympy.sin(t3_eff)],
        [sympy.sin(t3_eff),  sympy.cos(t3_eff),  0,  a2_sym * sympy.cos(t3_eff)],
        [0,                     0,       1,                0],
        [0,                     0,       0,                1]
    ])
    
    # H45: Joint 4 (wrist pitch)
    H45_sym = sympy.Matrix([
        [-sympy.sin(t4),  0, sympy.cos(t4), -a3_sym * sympy.sin(t4)],
        [ sympy.cos(t4),  0, sympy.sin(t4),  a3_sym * sympy.cos(t4)],
        [           0,   1,            0,                 0],
        [           0,   0,            0,                 1]
    ])
    
    # H56: Wrist rotation (fixed)
    H56_sym = sympy.Matrix([
        [sympy.cos(t5), -sympy.sin(t5), 0, 0],
        [sympy.sin(t5),  sympy.cos(t5), 0, 0],
        [0,                     0,      1, 0],
        [0,                     0,      0, 1]
    ])
    
    # Compute full transformation matrix
    H16_sym = sympy.simplify(H12_sym * H23_sym * H34_sym * H45_sym * H56_sym)
    # Extract end-effector position (Px, Py, Pz)
    Px = H16_sym[0, 3]
    Py = H16_sym[1, 3]
    Pz = H16_sym[2, 3]
    
    # Set up equations for the target coordinates
    eq1 = sympy.Eq(Px, xd)
    eq2 = sympy.Eq(Py, yd)
    eq3 = sympy.Eq(Pz, zd)
    
    # Use numerical solver (nsolve) for the three equations
    sol = sympy.nsolve((eq1, eq2, eq3), (th1, th2, th3), guess)
    th1_sol = float(sol[0])
    th2_sol = float(sol[1])
    th3_sol = float(sol[2])
    # As in the forward kinematics, Theta4 is fixed as -Theta3 and Theta5 is 0.
    th4_sol = -th3_sol
    th5_sol = 0.0
    
    return [th1_sol, th2_sol, th3_sol, th4_sol, th5_sol]

class KinematicsGUI(Node):
    def __init__(self):
        super().__init__('kinematics_gui')
        
        # Publisher for JointState messages on /joint_states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber to get the current end-effector pose (optional)
        self.pose_sub = self.create_subscription(Pose, 'end_effector_pose', self.pose_callback, 10)
        
        # Store the current joint positions (in radians) to publish continuously
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Timer to continuously publish the current joint states (10 Hz)
        self.publish_timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Build the Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Robot Kinematics GUI")
        
        # ----- Forward Kinematics Section -----
        fk_frame = tk.LabelFrame(self.root, text="Forward Kinematics (Input Joint Angles)")
        fk_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        self.fk_entries = {}
        for idx, label in enumerate(["Theta1", "Theta2", "Theta3", "Theta4", "Theta5"]):
            tk.Label(fk_frame, text=label + " (deg):").grid(row=idx, column=0, padx=5, pady=5)
            entry = tk.Entry(fk_frame)
            entry.grid(row=idx, column=1, padx=5, pady=5)
            self.fk_entries[label] = entry
        
        self.fk_button = tk.Button(fk_frame, text="Compute FK & Publish", command=self.handle_fk)
        self.fk_button.grid(row=5, column=0, columnspan=2, pady=10)
        
        self.fk_result = tk.Label(fk_frame, text="FK Pose: Not computed")
        self.fk_result.grid(row=6, column=0, columnspan=2, pady=5)
        
        # ----- Inverse Kinematics Section -----
        ik_frame = tk.LabelFrame(self.root, text="Inverse Kinematics (Input End-Effector Coordinates)")
        ik_frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        
        self.ik_entries = {}
        for idx, label in enumerate(["X", "Y", "Z"]):
            tk.Label(ik_frame, text=label + ":").grid(row=idx, column=0, padx=5, pady=5)
            entry = tk.Entry(ik_frame)
            entry.grid(row=idx, column=1, padx=5, pady=5)
            self.ik_entries[label] = entry
        
        self.ik_button = tk.Button(ik_frame, text="Compute IK", command=self.handle_ik)
        self.ik_button.grid(row=3, column=0, columnspan=2, pady=10)
        
        self.ik_result = tk.Label(ik_frame, text="IK Joints: Not computed")
        self.ik_result.grid(row=4, column=0, columnspan=2, pady=5)
        
        # ROS timer to update the Tkinter loop
        self.gui_timer = self.create_timer(0.1, self.tk_update)
        
        # Robot link lengths (in mm) for GUI IK (can be different from FK parameters if needed)
        self.a1 = 135.0  # Length of link 1
        self.a2 = 147.0  # Length of link 2
        self.a3 = 60.0   # Length of end effector

        # Joint limits in degrees (not enforced in the new IK solver)
        self.joint_limits = {
            "Theta1": (-135, 135),
            "Theta2": (-5, 80),
            "Theta3": (-10, 85),
            "Theta4": (-145, 145)
        }
    
    def handle_fk(self):
        """Compute FK from input joint angles, update display, and update self.current_joint_positions."""
        try:
            th1 = float(self.fk_entries["Theta1"].get())
            th2 = float(self.fk_entries["Theta2"].get())
            th3 = float(self.fk_entries["Theta3"].get())
            th4 = float(self.fk_entries["Theta4"].get())
        except ValueError as e:
            self.get_logger().error(f"Invalid FK input: {e}")
            return
        
        # Check joint limits
        for label, angle in zip(["Theta1", "Theta2", "Theta3", "Theta4"], [th1, th2, th3, th4]):
            min_limit, max_limit = self.joint_limits[label]
            if not (min_limit <= angle <= max_limit):
                self.fk_result.config(text=f"Error: {label} out of limits ({min_limit} to {max_limit} deg)")
                return

        try:
            th5 = float(self.fk_entries["Theta5"].get())
        except ValueError as e:
            self.get_logger().error(f"Invalid FK input for Theta5: {e}")
            return
        
        # Compute FK
        H16, pos = forward_kinematics(th1, th2, th3, th5)
        pose_text = f"X: {pos[0]:.2f}, Y: {pos[1]:.2f}, Z: {pos[2]:.2f}"
        self.fk_result.config(text="FK Pose: " + pose_text)
        
        # Convert angles to radians and store them for continuous publishing
        j1 = math.radians(th1)
        j2 = math.radians(th2)
        j3_ = math.radians(th3)
        j4 = math.radians(th4)
        j5_ = math.radians(th5)
        
        self.current_joint_positions = [j1, j2, j3_, j4, j5_]
        self.get_logger().info(f"handle_fk updated joint positions to: {self.current_joint_positions}")
    
    def handle_ik(self):
        """Compute IK from input coordinates using the symbolic solver, update display, and update joint positions."""
        try:
            x = float(self.ik_entries["X"].get())
            y = float(self.ik_entries["Y"].get())
            z = float(self.ik_entries["Z"].get())
        except ValueError as e:
            self.get_logger().error(f"Invalid IK input: {e}")
            return
        
        try:
            # Use the new IK solver with an initial guess.
            joint_angles = dobot_lite_ik(x, y, z, guess=(10, 10, 10))
        except Exception as e:
            self.ik_result.config(text="IK Joints: Position unreachable or error")
            self.get_logger().warn(f"IK error: {e}")
            return
        
        result_text = (f"Theta1: {joint_angles[0]:.2f}, Theta2: {joint_angles[1]:.2f}, "
                       f"Theta3: {joint_angles[2]:.2f}, Theta4: {joint_angles[3]:.2f}, "
                       f"Theta5: {joint_angles[4]:.2f}")
        self.ik_result.config(text="IK Joints: " + result_text)
        
        # Convert angles to radians and store them
        j1 = math.radians(joint_angles[0])
        j2 = math.radians(joint_angles[1])
        j3_ = math.radians(joint_angles[2])
        j4 = math.radians(joint_angles[3])
        j5_ = math.radians(joint_angles[4])
        
        self.current_joint_positions = [j1, j2, j3_, j4, j5_]
        self.get_logger().info(f"handle_ik updated joint positions to: {self.current_joint_positions}")
    
    def pose_callback(self, msg):
        """Update display based on received end-effector pose (if some other node publishes Pose)."""
        pos = msg.position
        ori = msg.orientation
        text = f"Received Pose -> Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})\n"
        text += f"Ori: ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})"
        self.fk_result.config(text="FK Pose: " + text)
    
    def publish_joint_states(self):
        """Continuously publish the current joint positions to /joint_states at 10 Hz."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        msg.position = self.current_joint_positions
        self.joint_pub.publish(msg)
    
    def tk_update(self):
        """Regularly update the Tkinter GUI event loop."""
        try:
            self.root.update()
        except tk.TclError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.root.destroy()

if __name__ == '__main__':
    main()
