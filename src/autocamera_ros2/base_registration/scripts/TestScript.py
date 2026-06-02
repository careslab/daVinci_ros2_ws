from itertools import chain
import math
import threading
import csv

import scipy
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import subprocess
import os
from ament_index_python.packages import get_package_share_directory
import xacro
import PyKDL
from kdl_parser_py import urdf
from sensor_msgs.msg import JointState
from urdf_kdl_utils import forward_kinematics, to_homo_mat, to_pos_euler

def arm_callback(msg, arm_kin, arm_name):
    angles = list(msg.position)
    if len(angles) > 0:
        xyz, _ = forward_kinematics(arm_kin, angles)
        print(f"{arm_name} xyz: [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]")
        print(f"dist {arm_name}: {math.sqrt(xyz[0]**2 + xyz[1]**2 + xyz[2]**2):.6f}")

def main():
    model_dir = os.path.join(get_package_share_directory('dvrk_model'), 'urdf')
    
    # Ask user which arm to display
    print("Select which arm to display:")
    print("1) PSM1")
    print("2) PSM2")
    print("3) ECM")
    choice = input("Enter choice (1/2/3): ").strip()
    
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    
    if choice == "1":
        # Load PSM1
        psm1_urdf_path = os.path.join(model_dir, 'Classic', 'psm.urdf.xacro')
        print(f"Loading PSM1 from: {psm1_urdf_path}")
        xml_string = xacro.process_file(psm1_urdf_path, mappings={'arm': 'psm1'}).toxml()
        ok, psm1_tree = urdf.treeFromString(xml_string)
        psm1_kin = psm1_tree.getChain("world", "psm1_tool_tip_link")
        
        node = Node('psm1_subscriber')
        node.create_subscription(
            JointState,
            '/PSM1/measured_js',
            lambda msg: arm_callback(msg, psm1_kin, "PSM1"),
            10
        )
        print("Listening for PSM1 joint state updates...\n")
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        
    elif choice == "2":
        # Load PSM2
        psm2_urdf_path = os.path.join(model_dir, 'Classic', 'psm.urdf.xacro')
        print(f"Loading PSM2 from: {psm2_urdf_path}")
        xml_string = xacro.process_file(psm2_urdf_path, mappings={'arm': 'psm2'}).toxml()
        ok, psm2_tree = urdf.treeFromString(xml_string)
        psm2_kin = psm2_tree.getChain("world", "psm2_tool_tip_link")
        
        node = Node('psm2_subscriber')
        node.create_subscription(
            JointState,
            '/PSM2/measured_js',
            lambda msg: arm_callback(msg, psm2_kin, "PSM2"),
            10
        )
        print("Listening for PSM2 joint state updates...\n")
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        
    elif choice == "3":
        # Load ECM
        ecm_urdf_path = os.path.join(model_dir, 'Classic', 'ecm.urdf.xacro')
        print(f"Loading ECM from: {ecm_urdf_path}")
        xml_string = xacro.process_file(ecm_urdf_path, mappings={'arm': 'ecm'}).toxml()
        ok, ecm_tree = urdf.treeFromString(xml_string)
        ecm_kin = ecm_tree.getChain("ecm_base_link", "ecm_end_link")
        
        node = Node('ecm_subscriber')
        node.create_subscription(
            JointState,
            '/ECM/measured_js',
            lambda msg: arm_callback(msg, ecm_kin, "ECM"),
            10
        )
        print("Listening for ECM joint state updates...\n")
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
    else:
        print("Invalid choice")
        rclpy.shutdown()
        return
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()