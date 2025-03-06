#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
from ikpy.chain import Chain

# Load the URDF file
URDF_FILE = "/home/blazar/workspaces/dobbie_ws/src/descy/urdf/dobbie.urdf"

def main():
    # Load the kinematic chain from URDF
    chain = Chain.from_urdf_file(URDF_FILE, base_elements=["base"])

    # 🔹 Define the target position (convert mm to meters)
    target_position = [311.7681 / 1000.0, -74.8789 / 1000.0, -7.9137 / 1000.0]

    # 🔹 Define the target orientation (using given R value)
    roll_angle = np.radians(101.3233)  # Convert R (Roll) to radians
    rotation_matrix = R.from_euler('xyz', [roll_angle, 0, 0]).as_matrix()  # Convert to 3x3 rotation matrix

    # 🔹 Compute the inverse kinematics with position & orientation
    ik_solution = chain.inverse_kinematics(target_position, target_orientation=rotation_matrix)

    # 🔹 Output joint angles (converted to degrees for readability)
    print("\n🔹 IK Solution (Joint Angles)")
    for i, angle in enumerate(ik_solution):
        print(f"  Joint {i}: {angle:.4f} rad ({np.degrees(angle):.2f} deg)")

if __name__ == "__main__":
    main()
