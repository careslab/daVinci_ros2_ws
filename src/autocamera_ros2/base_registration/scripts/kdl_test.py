#!/usr/bin/env python3

import os
import sys
import time
import threading
import xacro
import PyKDL
from rclpy.node import Node
from kdl_parser_py import urdf
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
import rclpy


class KDLTester(Node):

    def __init__(self):
        super().__init__('kdl_tester')
        self.psm1_read = False
        self.psm2_read = False
        self.ecm_read = False
        self.chain = None
        self.psm2_chain = None
        self.ecm_chain = None
        self.create_subscription(JointState, '/PSM1/measured_js', self.psm1_read_js, 10)
        self.create_subscription(JointState, '/PSM2/measured_js', self.psm2_read_js, 10)
        self.create_subscription(JointState, '/ECM/measured_js', self.ecm_read_js, 10)

        
    def print_tree_structure(self,segment, prefix="", parent_name=""):
        """Recursively print the tree structure"""
        print(f"{prefix}├─ {segment.getName()}")
        for i in range(segment.getNrOfChildren()):
            child = segment.getChild(i)
            self.print_tree_structure(child, prefix + "│  ", segment.getName())

    def test_psm1_chain(self):
        # Load PSM1 URDF
        model_dir = os.path.join(get_package_share_directory('dvrk_model'), 'urdf')
        psm1_urdf_path = os.path.join(model_dir, 'Classic', 'PSM1.urdf.xacro')
        
        print(f"Loading PSM1 from: {psm1_urdf_path}")
        print(f"File exists: {os.path.exists(psm1_urdf_path)}\n")
        
        # Process URDF with xacro
        xml_string = xacro.process_file(psm1_urdf_path).toxml()
        ok, tree = urdf.treeFromString(xml_string)
        
        if not ok:
            print("✗ Failed to parse URDF")
            return
        
        print("✓ URDF parsed successfully\n")
        
        # Try common link name patterns
        print("Attempting to create chains with common link name patterns:\n")
        
        link_pairs = [
            ("world", "PSM1_tool_tip_link"),
            ("world", "PSM1_tool_wrist_caudier_ee_link"),
            ("PSM1_base", "PSM1_tool_tip_link"),
            ("PSM1_base", "PSM1_tool_wrist_caudier_ee_link"),
            ("PSM1_psm_base_link", "PSM1_tool_tip_link"),
            ("PSM1_psm_base_link", "PSM1_tool_wrist_caudier_ee_link"),
        ]
        
        for start_link, end_link in link_pairs:
            try:
                self.chain = tree.getChain(start_link, end_link)
                n_segments = self.chain.getNrOfSegments()
                n_joints = self.chain.getNrOfJoints()
                
                if n_segments > 0:
                    print(f"✓ Chain from '{start_link}' to '{end_link}':")
                    print(f"    Segments: {n_segments}, Joints: {n_joints}")
                    for i in range(n_segments):
                        seg = self.chain.getSegment(i)
                        print(f"      {i}: {seg.getName()}")
                else:
                    print(f"✗ Chain from '{start_link}' to '{end_link}': 0 segments")
            except Exception as e:
                print(f"✗ Chain from '{start_link}' to '{end_link}': Error - {e}")
            print()
            
        print("Computing forward kinematics chain from world to tool tip")
        self.chain = tree.getChain("world", "PSM1_tool_tip_link")
        
        # Create zero joint angles
        n_joints = self.chain.getNrOfJoints()
        print(f"Number of joints in chain: {n_joints}")
                
        self.psm1_read=True
        
        input("Press Enter when done")    

    def test_psm2_chain(self):
        # Load PSM2 URDF
        model_dir = os.path.join(get_package_share_directory('dvrk_model'), 'urdf')
        psm2_urdf_path = os.path.join(model_dir, 'Classic', 'PSM2.urdf.xacro')
        
        print(f"Loading PSM2 from: {psm2_urdf_path}")
        print(f"File exists: {os.path.exists(psm2_urdf_path)}\n")
        
        # Process URDF with xacro
        xml_string = xacro.process_file(psm2_urdf_path).toxml()
        ok, tree = urdf.treeFromString(xml_string)
        
        if not ok:
            print("✗ Failed to parse URDF")
            return
        
        print("✓ URDF parsed successfully\n")
        
        # Try common link name patterns for PSM2
        print("Attempting to create chains with common link name patterns:\n")
        
        link_pairs = [
            ("world", "PSM2_tool_tip_link"),
            ("PSM2_psm_base_link", "PSM2_tool_wrist_caudier_link_shaft"),
            ("world", "PSM2_tool_wrist_caudier_ee_link"),
            ("PSM2_base", "PSM2_tool_tip_link"),
            ("PSM2_psm_base_link", "PSM2_tool_tip_link"),
        ]
        
        for start_link, end_link in link_pairs:
            try:
                self.psm2_chain = tree.getChain(start_link, end_link)
                n_segments = self.psm2_chain.getNrOfSegments()
                n_joints = self.psm2_chain.getNrOfJoints()
                
                if n_segments > 0:
                    print(f"✓ Chain from '{start_link}' to '{end_link}':")
                    print(f"    Segments: {n_segments}, Joints: {n_joints}")
                    for i in range(n_segments):
                        seg = self.psm2_chain.getSegment(i)
                        print(f"      {i}: {seg.getName()}")
                else:
                    print(f"✗ Chain from '{start_link}' to '{end_link}': 0 segments")
            except Exception as e:
                print(f"✗ Chain from '{start_link}' to '{end_link}': Error - {e}")
            print()
            
        print("Computing forward kinematics chain from world to tool tip")
        self.psm2_chain = tree.getChain("world", "PSM2_tool_tip_link")
        
        # Create zero joint angles
        n_joints = self.psm2_chain.getNrOfJoints()
        print(f"Number of joints in chain: {n_joints}")
                
        self.psm2_read=True
        
        input("Press Enter when done")    

    def test_ecm_chain(self):
        # Load ECM URDF
        model_dir = os.path.join(get_package_share_directory('dvrk_model'), 'urdf')
        ecm_urdf_path = os.path.join(model_dir, 'Classic', 'ECM.urdf.xacro')
        
        print(f"Loading ECM from: {ecm_urdf_path}")
        print(f"File exists: {os.path.exists(ecm_urdf_path)}\n")
        
        # Process URDF with xacro
        xml_string = xacro.process_file(ecm_urdf_path).toxml()
        print(xml_string)
        ok, tree = urdf.treeFromString(xml_string)
        
        if not ok:
            print("✗ Failed to parse URDF")
            return
        
        print("✓ URDF parsed successfully\n")
        
        # Try common link name patterns for ECM
        print("Attempting to create chains with common link name patterns:\n")
        
        link_pairs = [
            ("world", "ECM_tool_link"),
            ("world", "ECM_end_link"),
            ("ECM_base_link", "ECM_tool_link"),
            ("ECM_base_link", "ECM_end_link"),
        ]
        
        for start_link, end_link in link_pairs:
            try:
                self.ecm_chain = tree.getChain(start_link, end_link)
                n_segments = self.ecm_chain.getNrOfSegments()
                n_joints = self.ecm_chain.getNrOfJoints()
                
                if n_segments > 0:
                    print(f"✓ Chain from '{start_link}' to '{end_link}':")
                    print(f"    Segments: {n_segments}, Joints: {n_joints}")
                    for i in range(n_segments):
                        seg = self.ecm_chain.getSegment(i)
                        print(f"      {i}: {seg.getName()}")
                else:
                    print(f"✗ Chain from '{start_link}' to '{end_link}': 0 segments")
            except Exception as e:
                print(f"✗ Chain from '{start_link}' to '{end_link}': Error - {e}")
            print()
            
        print("Computing forward kinematics chain from world to tool tip")
        self.ecm_chain = tree.getChain("world", "ECM_tool_link")
        
        # Create zero joint angles
        n_joints = self.ecm_chain.getNrOfJoints()
        print(f"Number of joints in chain: {n_joints}")
                
        self.ecm_read=True
        
        input("Press Enter when done")    

    def psm1_read_js(self, msg):
        if self.psm1_read == True:
            self.get_logger().info("psm1 joint angles collected")
            self.psm1_read = False
            
            # Skip if we don't have enough joint positions
            if len(msg.position) < 6:
                return
            
            n_joints = self.chain.getNrOfJoints()
            q = PyKDL.JntArray(n_joints)
            for i in range(min(n_joints, len(msg.position))):
                q[i] = float(msg.position[i])
            
            # Compute forward kinematics
            fksolver = PyKDL.ChainFkSolverPos_recursive(self.chain)
            end_effector_frame = PyKDL.Frame()
            status = fksolver.JntToCart(q, end_effector_frame)
    
            if status >= 0:
                print(f"✓ Forward kinematics computed successfully")
                print(f"  End effector position: {end_effector_frame.p}")
                print(f"  End effector rotation:\n {end_effector_frame.M}")
            else:
                print(f"✗ Forward kinematics failed with status: {status}")                        
            time.sleep(0.5)

    def psm2_read_js(self, msg):
        if self.psm2_read == True:
            self.get_logger().info("psm2 joint angles collected")
            self.psm2_read = False
            # Skip if we don't have enough joint positions
            if len(msg.position) < 6:
                print(f"Skipping - got {len(msg.position)} positions, need 6")
                return
            
            n_joints = self.psm2_chain.getNrOfJoints()
            q = PyKDL.JntArray(n_joints)
            for i in range(min(n_joints, len(msg.position))):
                q[i] = float(msg.position[i])
            
            # Compute forward kinematics
            fksolver = PyKDL.ChainFkSolverPos_recursive(self.psm2_chain)
            end_effector_frame = PyKDL.Frame()
            status = fksolver.JntToCart(q, end_effector_frame)
    
            if status >= 0:
                print(f"✓ Forward kinematics computed successfully")
                print(f"  End effector position: {end_effector_frame.p}")
                print(f"  End effector rotation:\n {end_effector_frame.M}")
            else:
                print(f"✗ Forward kinematics failed with status: {status}")                        
            time.sleep(0.5)

    def ecm_read_js(self, msg):
        if self.ecm_read == True:
            self.get_logger().info("ecm joint angles collected")
            self.ecm_read = False
            # Skip if we don't have enough joint positions
            if len(msg.position) < 1:
                print("No joint positions received")
                return
            
            n_joints = self.ecm_chain.getNrOfJoints()
            print(f"  Number of joints in message: {len(msg.position)}, chain expects: {n_joints}")
            
            q = PyKDL.JntArray(n_joints)
            for i in range(min(n_joints, len(msg.position))):
                q[i] = float(msg.position[i])
            
            # Compute forward kinematics
            fksolver = PyKDL.ChainFkSolverPos_recursive(self.ecm_chain)
            end_effector_frame = PyKDL.Frame()
            status = fksolver.JntToCart(q, end_effector_frame)
    
            if status >= 0:
                print(f"✓ Forward kinematics computed successfully")
                print(f"  End effector position: {end_effector_frame.p}")
                print(f"  End effector rotation:\n {end_effector_frame.M}")
            else:
                print(f"✗ Forward kinematics failed with status: {status}")                        
            time.sleep(0.5)
            
def main(args=None):
    rclpy.init(args=args)
    node = KDLTester()
    # 1. Start ROS 2 spinning in a background thread
    # This allows callbacks (psm1_read_cb, etc.) to run while we use input()
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    
    while True:
        print("\nSelect arm to test:")
        print("(1) PSM1")
        print("(2) PSM2")
        print("(3) ECM")
        print("(q) Quit")
        choice = input("Selection: ")
        
        if choice == "1":
            node.test_psm1_chain()
        elif choice == "2":
            node.test_psm2_chain()
        elif choice == "3":
            node.test_ecm_chain()
        elif choice == "q":
            break
        else:
            print("Invalid selection")
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()