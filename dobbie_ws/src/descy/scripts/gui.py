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
a1 = 150.0  # link 1 length
a2 = 150.0  # link 2 length
a3 = 90.0   # link 3 (end effector) length

def rotation_z(theta):
    """Rotation about z-axis."""
    # simple rotation matrix about Z, nothing fancy
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])

def H12(t1):
    # Transformation from frame 1 to frame 2 using t1 angle
    return np.array([[0, math.cos(t1), -math.sin(t1), 0],
                     [0, math.sin(t1),  math.cos(t1), 0],
                     [1,          0,           0,  0],
                     [0,          0,           0,  1]])

def H23(t2):
    # Transformation from frame 2 to frame 3 using t2
    return np.array([[math.cos(t2), -math.sin(t2), 0, a1*math.cos(t2)],
                     [math.sin(t2),  math.cos(t2), 0, a1*math.sin(t2)],
                     [0,             0,            1, 0],
                     [0,             0,            0, 1]])

def H34(t3):
    # Transformation from frame 3 to frame 4 (elbow) using t3
    return np.array([[math.cos(t3), -math.sin(t3), 0, -a2*math.sin(t3)],
                     [math.sin(t3),  math.cos(t3), 0,  a2*math.cos(t3)],
                     [0,             0,            1, 0],
                     [0,             0,            0, 1]])

def H45(t4):
    # Transformation from frame 4 to frame 5, where t4 is fixed as -theta3
    return np.array([[-math.sin(t4), 0, math.cos(t4), -a3*math.sin(t4)],
                     [ math.cos(t4), 0, math.sin(t4),  a3*math.cos(t4)],
                     [0,             1,          0,  0],
                     [0,             0,          0,  1]])

def H56(t5):
    # Transformation from frame 5 to frame 6 (wrist rotation)
    return np.array([[math.cos(t5), -math.sin(t5), 0, 0],
                     [math.sin(t5),  math.cos(t5), 0, 0],
                     [0,             0,            1, 0],
                     [0,             0,            0, 1]])

def forward_kinematics(th1_deg, th2_deg, th3_deg, th5_deg):
    """
    Compute the forward kinematics.
    Inputs are joint angles (in degrees).
    We compute effective angles:
      t1 = th1, t2 = th2, t3 = -th2 + th3, t4 = -th3 (fixed), t5 = th5.
    """
    # Convert degrees to radians for math functions
    th1 = math.radians(th1_deg)
    th2 = math.radians(th2_deg)
    th3 = math.radians(th3_deg)
    th5 = math.radians(th5_deg)
    
    # Set up our effective angles
    t1 = th1
    t2 = th2
    t3_ = -th2 + th3  # effective t3
    t4 = -th3        # fixed joint angle for t4
    t5_ = th5

    # Calculate the homogeneous transforms
    H_12 = H12(t1)
    H_23 = H23(t2)
    H_34 = H34(t3_)
    H_45 = H45(t4)
    H_56 = H56(t5_)

    # Multiply them all together to get the full transform from base to end-effector
    H16 = H_12 @ H_23 @ H_34 @ H_45 @ H_56
    position = H16[0:3, 3]  # extract x,y,z position
    return H16, position

def dobot_lite_ik(xd, yd, zd, guess=(10, 10, 10)):
    """
    Compute inverse kinematics using a symbolic approach.
    Finds joint angles th1, th2, and th3 (in degrees) such that the end-effector
    position matches (xd, yd, zd). Note t4 is -th3 and t5 is fixed (0).
    """
    # Create symbolic variables for the joint angles
    th1, th2, th3 = sympy.symbols('th1 th2 th3', real=True)
    th4_val = 0  # fixed wrist rotation

    # Convert the angles to radians symbolically
    t1 = th1 * sympy.pi / 180
    t2 = th2 * sympy.pi / 180
    t3_eff = (-th2 + th3) * sympy.pi / 180
    t4 = -th3 * sympy.pi / 180
    t5 = th4_val * sympy.pi / 180  # t5 is 0

    # Link lengths
    a1_sym = 150
    a2_sym = 150
    a3_sym = 90

    # Build each homogeneous transform symbolically
    H12_sym = sympy.Matrix([
        [0, sympy.cos(t1), -sympy.sin(t1), 0],
        [0, sympy.sin(t1),  sympy.cos(t1), 0],
        [1,           0,            0,  0],
        [0,           0,            0,  1]
    ])

    H23_sym = sympy.Matrix([
        [sympy.cos(t2), -sympy.sin(t2), 0, a1_sym * sympy.cos(t2)],
        [sympy.sin(t2),  sympy.cos(t2), 0, a1_sym * sympy.sin(t2)],
        [0,                    0,  1,                0],
        [0,                    0,  0,                1]
    ])

    H34_sym = sympy.Matrix([
        [sympy.cos(t3_eff), -sympy.sin(t3_eff), 0, -a2_sym * sympy.sin(t3_eff)],
        [sympy.sin(t3_eff),  sympy.cos(t3_eff), 0,  a2_sym * sympy.cos(t3_eff)],
        [0,                    0,  1,                0],
        [0,                    0,  0,                1]
    ])

    H45_sym = sympy.Matrix([
        [-sympy.sin(t4), 0, sympy.cos(t4), -a3_sym * sympy.sin(t4)],
        [ sympy.cos(t4), 0, sympy.sin(t4),  a3_sym * sympy.cos(t4)],
        [           0, 1,           0,                 0],
        [           0, 0,           0,                 1]
    ])

    H56_sym = sympy.Matrix([
        [sympy.cos(t5), -sympy.sin(t5), 0, 0],
        [sympy.sin(t5),  sympy.cos(t5), 0, 0],
        [0,                    0, 1, 0],
        [0,                    0, 0, 1]
    ])

    # Multiply all the transforms together
    H16_sym = sympy.simplify(H12_sym * H23_sym * H34_sym * H45_sym * H56_sym)
    # Get the position from the final transform
    Px = H16_sym[0, 3]
    Py = H16_sym[1, 3]
    Pz = H16_sym[2, 3]

    # Set up equations to match desired position
    eq1 = sympy.Eq(Px, xd)
    eq2 = sympy.Eq(Py, yd)
    eq3 = sympy.Eq(Pz, zd)

    # Solve the equations using a numerical solver with a guess
    sol = sympy.nsolve((eq1, eq2, eq3), (th1, th2, th3), guess)
    th1_sol = float(sol[0])
    th2_sol = float(sol[1])
    th3_sol = float(sol[2])
    # As per our design, t4 = -th3 and t5 = 0.
    th4_sol = -th3_sol
    th5_sol = 0.0

    return [th1_sol, th2_sol, th3_sol, th4_sol, th5_sol]

class KinematicsGUI(Node):
    def __init__(self):
        super().__init__('kinematics_gui')
        
        # Set up the publisher for joint states (publishes to /joint_states)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Set up a subscriber to get the current end-effector pose (if available)
        self.pose_sub = self.create_subscription(Pose, 'end_effector_pose', self.pose_callback, 10)
        
        # Current and target joint positions (in radians) for smooth motion
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_joint_positions = self.current_joint_positions.copy()
        
        # Timer to publish joint states at 10Hz
        self.publish_timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Build the Tkinter GUI window
        self.root = tk.Tk()
        self.root.title("Robot Kinematics GUI")
        
        # ----- Forward Kinematics Section -----
        fk_frame = tk.LabelFrame(self.root, text="Forward Kinematics (Input Joint Angles)")
        fk_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        # Create entry boxes for each joint angle (in degrees)
        self.fk_entries = {}
        for idx, label in enumerate(["Theta1", "Theta2", "Theta3", "Theta4", "Theta5"]):
            tk.Label(fk_frame, text=label + " (deg):").grid(row=idx, column=0, padx=5, pady=5)
            entry = tk.Entry(fk_frame)
            entry.grid(row=idx, column=1, padx=5, pady=5)
            self.fk_entries[label] = entry
        
        # Button to compute FK and publish joint states
        self.fk_button = tk.Button(fk_frame, text="Compute FK & Publish", command=self.handle_fk)
        self.fk_button.grid(row=5, column=0, columnspan=2, pady=10)
        
        # Label to show the computed FK pose
        self.fk_result = tk.Label(fk_frame, text="FK Pose: Not computed")
        self.fk_result.grid(row=6, column=0, columnspan=2, pady=5)
        
        # ----- Inverse Kinematics Section -----
        ik_frame = tk.LabelFrame(self.root, text="Inverse Kinematics (Input End-Effector Coordinates)")
        ik_frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        
        # Create entry boxes for x, y, z of the end-effector
        self.ik_entries = {}
        for idx, label in enumerate(["X", "Y", "Z"]):
            tk.Label(ik_frame, text=label + ":").grid(row=idx, column=0, padx=5, pady=5)
            entry = tk.Entry(ik_frame)
            entry.grid(row=idx, column=1, padx=5, pady=5)
            self.ik_entries[label] = entry
        
        # Button to compute IK based on the given coordinates
        self.ik_button = tk.Button(ik_frame, text="Compute IK", command=self.handle_ik)
        self.ik_button.grid(row=3, column=0, columnspan=2, pady=10)
        
        # Label to show the computed IK joint angles
        self.ik_result = tk.Label(ik_frame, text="IK Joints: Not computed")
        self.ik_result.grid(row=4, column=0, columnspan=2, pady=5)
        
        # ----- Motion Parameters Section -----
        motion_frame = tk.LabelFrame(self.root, text="Speed")
        motion_frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        tk.Label(motion_frame, text="Speed (deg/s):").grid(row=0, column=0, padx=5, pady=5)
        self.speed_entry = tk.Entry(motion_frame)
        self.speed_entry.grid(row=0, column=1, padx=5, pady=5)
        self.speed_entry.insert(0, "20")  # default speed set to 20 deg/s
        
        # ----- Live Joint Sliders Section -----
        sliders_frame = tk.LabelFrame(self.root, text="Live Joint Sliders")
        sliders_frame.grid(row=3, column=0, padx=10, pady=10, sticky="ew")
        
        # Create slider variables and sliders for each joint.
        self.slider_vars = {}
        self.sliders = {}
        joint_order = ["Theta1", "Theta2", "Theta3", "Theta4", "Theta5"]
        # Use joint limits for Theta1-Theta4 and manual limits for Theta5
        limits = {
            "Theta1": self.joint_limits["Theta1"] if hasattr(self, 'joint_limits') else (-135, 135),
            "Theta2": self.joint_limits["Theta2"] if hasattr(self, 'joint_limits') else (-5, 80),
            "Theta3": self.joint_limits["Theta3"] if hasattr(self, 'joint_limits') else (-10, 85),
            "Theta4": self.joint_limits["Theta4"] if hasattr(self, 'joint_limits') else (-145, 145),
            "Theta5": (-180, 180)
        }
        for idx, joint in enumerate(joint_order):
            self.slider_vars[joint] = tk.DoubleVar()
            self.slider_vars[joint].set(0)  # start all sliders at 0 deg
            tk.Label(sliders_frame, text=joint).grid(row=idx, column=0, padx=5, pady=5)
            self.sliders[joint] = tk.Scale(sliders_frame, variable=self.slider_vars[joint],
                                           from_=limits[joint][0], to=limits[joint][1],
                                           orient=tk.HORIZONTAL, resolution=0.5,
                                           command=self.update_from_sliders)
            self.sliders[joint].grid(row=idx, column=1, padx=5, pady=5)
        
        # ROS timer to update the Tkinter GUI loop every 0.1 sec
        self.gui_timer = self.create_timer(0.1, self.tk_update)
        
        # Robot link lengths (can be different from FK parameters if needed)
        self.a1 = 135.0  # link 1 length for GUI IK
        self.a2 = 147.0  # link 2 length for GUI IK
        self.a3 = 60.0   # end effector length for GUI IK

        # Joint limits for FK/IK input checking (only for Theta1-Theta4 here)
        self.joint_limits = {
            "Theta1": (-135, 135),
            "Theta2": (-5, 80),
            "Theta3": (-10, 85),
            "Theta4": (-145, 145)
        }
    
    def update_from_sliders(self, event=None):
        """
        Callback when any slider moves.
        Reads slider values (in degrees), converts to radians, and updates target positions.
        """
        self.target_joint_positions = [
            math.radians(self.slider_vars["Theta1"].get()),
            math.radians(self.slider_vars["Theta2"].get()),
            math.radians(self.slider_vars["Theta3"].get()),
            math.radians(self.slider_vars["Theta4"].get()),
            math.radians(self.slider_vars["Theta5"].get())
        ]
        # Optionally update the FK entry fields with slider values
        for joint in ["Theta1", "Theta2", "Theta3", "Theta4", "Theta5"]:
            if joint in self.fk_entries:
                self.fk_entries[joint].delete(0, tk.END)
                self.fk_entries[joint].insert(0, f"{self.slider_vars[joint].get():.2f}")
        self.get_logger().info(f"Sliders updated target joint positions to: {self.target_joint_positions}")
    
    def handle_fk(self):
        """Compute FK using entry box angles and update target joint positions."""
        try:
            th1 = float(self.fk_entries["Theta1"].get())
            th2 = float(self.fk_entries["Theta2"].get())
            th3 = float(self.fk_entries["Theta3"].get())
            th4 = float(self.fk_entries["Theta4"].get())
        except ValueError as e:
            self.get_logger().error(f"Invalid FK input: {e}")
            return
        
        # Check if angles are within limits
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
        
        # Compute FK and update displayed pose
        H16, pos = forward_kinematics(th1, th2, th3, th5)
        pose_text = f"X: {pos[0]:.2f}, Y: {pos[1]:.2f}, Z: {pos[2]:.2f}"
        self.fk_result.config(text="FK Pose: " + pose_text)
        
        # Update target joint positions based on FK entry values
        j1 = math.radians(th1)
        j2 = math.radians(th2)
        j3_ = math.radians(th3)
        j4 = math.radians(th4)
        j5_ = math.radians(th5)
        
        self.target_joint_positions = [j1, j2, j3_, j4, j5_]
        self.get_logger().info(f"handle_fk updated target joint positions to: {self.target_joint_positions}")
    
    def handle_ik(self):
        """Compute IK from input coordinates and update target joint positions."""
        try:
            x = float(self.ik_entries["X"].get())
            y = float(self.ik_entries["Y"].get())
            z = float(self.ik_entries["Z"].get())
        except ValueError as e:
            self.get_logger().error(f"Invalid IK input: {e}")
            return
        
        try:
            # Compute IK using the symbolic solver
            joint_angles = dobot_lite_ik(x, y, z, guess=(10, 10, 10))
        except Exception as e:
            self.ik_result.config(text="IK Joints: Position unreachable or error")
            self.get_logger().warn(f"IK error: {e}")
            return
        
        result_text = (f"Theta1: {joint_angles[0]:.2f}, Theta2: {joint_angles[1]:.2f}, "
                       f"Theta3: {joint_angles[2]:.2f}, Theta4: {joint_angles[3]:.2f}, "
                       f"Theta5: {joint_angles[4]:.2f}")
        self.ik_result.config(text="IK Joints: " + result_text)
        
        # Update target joint positions from IK result
        j1 = math.radians(joint_angles[0])
        j2 = math.radians(joint_angles[1])
        j3_ = math.radians(joint_angles[2])
        j4 = math.radians(joint_angles[3])
        j5_ = math.radians(joint_angles[4])
        
        self.target_joint_positions = [j1, j2, j3_, j4, j5_]
        self.get_logger().info(f"handle_ik updated target joint positions to: {self.target_joint_positions}")
    
    def pose_callback(self, msg):
        """When a Pose msg is received, update the FK result display."""
        pos = msg.position
        ori = msg.orientation
        text = f"Received Pose -> Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})\n"
        text += f"Ori: ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})"
        self.fk_result.config(text="FK Pose: " + text)
    
    def publish_joint_states(self):
        """
        Smoothly update the current joint positions toward the target positions 
        (based on the speed set in the GUI) and publish them.
        """
        try:
            speed_deg = float(self.speed_entry.get())  # read speed in deg/s from GUI
        except ValueError:
            speed_deg = 20.0  # default speed if invalid input
        dt = 0.1  # time interval for updates (10 Hz)
        step = math.radians(speed_deg) * dt  # step size in radians per update

        # Gradually adjust each joint's current position towards its target
        for i in range(len(self.current_joint_positions)):
            diff = self.target_joint_positions[i] - self.current_joint_positions[i]
            if abs(diff) > step:
                self.current_joint_positions[i] += step * (1 if diff > 0 else -1)
            else:
                self.current_joint_positions[i] = self.target_joint_positions[i]
        
        # Build and publish the JointState message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        msg.position = self.current_joint_positions
        self.joint_pub.publish(msg)
    
    def tk_update(self):
        """Regularly update the Tkinter GUI loop (so the window stays responsive)."""
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
