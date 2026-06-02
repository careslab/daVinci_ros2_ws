from itertools import chain
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
from urdf_kdl_utils import forward_kinematics, pos_rot_to_homo_mat, to_homo_mat, to_pos_euler

class coregistrator(Node):
    """
        The coregistrator class collects the joint angles from the da vinci platform and 
        optimizes the relative transformation of their bases. We have to touch the tips 
        of two pairs of arms together and record the data a few times. Then run the 
        optimization algorithm. We need to do it twice one for psm1 and psm2, and another
        one for psm1 and ecm.
    """
    
    class REGISTRATION_MODE:
        """
            The registration mode. 
        """
        PSM1_PSM2 = 'psm1_psm2'
        PSM1_ECM = 'psm1_ecm'
    
    # Static variables
    mode = REGISTRATION_MODE.PSM1_PSM2
    
    psm1_data = None
    psm2_data = None
    ecm_data = None
    
    def __init__(self):
        super().__init__('psm_optimization_data_collector')
        
        self.psm1_tree = None
        self.psm1_kin = None
        
        self.psm2_tree = None
        self.psm2_kin = None
        
        self.ecm_tree = None
        self.ecm_kin = None
    
        self.psm1_read_cb_save = False
        self.psm1_read_cb_count = 0
        
        self.psm2_read_cb_save = False
        self.psm2_read_cb_count = 0
        
        self.ecm_read_cb_save = False
        self.ecm_read_cb_count = 0
        
        self.collected_joint_angles = {}
        
        # Find the kinematic model of the arms from base to the end-effector
        # The base for psm1 and psm2 is link 1 but for ecm is link 3
        # The first link is the world
        model_dir = os.path.join(get_package_share_directory('dvrk_model'), 'urdf')
        
        if self.psm1_tree is None:
            psm1_urdf_path = os.path.join(model_dir, 'Classic', 'psm.urdf.xacro')
            print(f"Loading PSM1 from: {psm1_urdf_path}")
            xml_string = xacro.process_file(psm1_urdf_path, mappings={'arm': 'psm1'}).toxml()
            ok, self.psm1_tree = urdf.treeFromString(xml_string)
            self.psm1_kin = self.psm1_tree.getChain("world", "psm1_tool_tip_link")                
                
        if self.psm2_tree is None:
            psm2_urdf_path = os.path.join(model_dir, 'Classic', 'psm.urdf.xacro')
            print(f"Loading PSM2 from: {psm2_urdf_path}")
            xml_string = xacro.process_file(psm2_urdf_path, mappings={'arm': 'psm2'}).toxml()
            ok, self.psm2_tree = urdf.treeFromString(xml_string)
            self.psm2_kin = self.psm2_tree.getChain("world", "psm2_tool_tip_link")
            
        if self.ecm_tree is None:
            ecm_urdf_path = os.path.join(model_dir, 'Classic', 'ecm.urdf.xacro')
            print(f"Loading ECM from: {ecm_urdf_path}")
            xml_string = xacro.process_file(ecm_urdf_path, mappings={'arm': 'ecm'}).toxml()
            ok, self.ecm_tree = urdf.treeFromString(xml_string)
            self.ecm_kin = self.ecm_tree.getChain("world", "ecm_end_link")
    
        self.__init_nodes()
        
    def __init_nodes(self):
        # Subscribe to joint state topics
        self.create_subscription(JointState, '/PSM1/measured_js', self.psm1_read_cb, 10)
        self.create_subscription(JointState, '/PSM2/measured_js', self.psm2_read_cb, 10)
        self.create_subscription(JointState, '/ECM/measured_js', self.ecm_read_cb, 10)
    
    # Callback to the psm1 subscriber
    def psm1_read_cb(self, msg):
        if self.psm1_read_cb_save == True:
            # Prevent situation where zero joint angles are collected due to race condition   
            if len(msg.position) == 0:
                return                
            self.get_logger().info("psm1 joint angles collected")
            self.psm1_read_cb_save = False
                            
            if self.psm1_read_cb_count == 0 and 'psm1_angles' not in self.collected_joint_angles:
                self.collected_joint_angles["psm1_angles"] = []
            self.collected_joint_angles["psm1_angles"].append(msg.position)
            
            self.psm1_read_cb_count += 1   
                
    # callback to the psm2 subscriber
    def psm2_read_cb(self, msg):
        if self.psm2_read_cb_save == True:
            # Prevent situation where zero joint angles are collected due to race condition   
            if len(msg.position) == 0:
                return      
            self.get_logger().info("psm2 joint angles collected")
            self.psm2_read_cb_save = False
                        
            if self.psm2_read_cb_count == 0 and 'psm2_angles' not in self.collected_joint_angles:
                self.collected_joint_angles["psm2_angles"] = []
            
            self.collected_joint_angles["psm2_angles"].append(msg.position)
            
            self.psm2_read_cb_count += 1
    
    # ecm callback
    def ecm_read_cb(self, msg):
        if self.ecm_read_cb_save == True:
            # Prevent situation where zero joint angles are collected due to race condition   
            if len(msg.position) == 0:
                return      
            self.get_logger().info("ecm joint angles collected")
            self.ecm_read_cb_save = False
            
            if self.ecm_read_cb_count == 0 and 'ecm_angles' not in self.collected_joint_angles:
                self.collected_joint_angles["ecm_angles"] = []
                
            self.collected_joint_angles["ecm_angles"].append(msg.position)
           
            self.ecm_read_cb_count += 1    
        
    def collect(self):
        while True:
            print("save now? ")
            print("(y) yes\n(n) no\n(r) read from CSV\n(q) quit")
            r = input(" : ")
            
            if r == "q":
                return
            if r == "y":
                self.psm1_read_cb_save = True
                self.psm2_read_cb_save = True
                self.ecm_read_cb_save = True
            if r == "r":
                csv_path = os.path.join(os.getcwd(), 'calibration_data.csv')
                print(f"Reading from {csv_path}...")
                self.read_from_csv(csv_path)
                print("data read from csv")
                return
    
    def save_to_csv(self, csv_name):
        with open(csv_name, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(['arm', 'joint_angles', 'end_effector_position'])
            # Write data for each arm
            for arm in ['psm1', 'psm2', 'ecm']:
                angles_key = f'{arm}_angles'
                if angles_key in self.collected_joint_angles:
                    # Compute end-effector positions using forward kinematics
                    xyz_positions = self.compute_fk(arm)
                    for angles, xyz in zip(self.collected_joint_angles[angles_key], xyz_positions):
                        writer.writerow([arm, list(angles), list(xyz)])
    
    def read_from_csv(self, csv_name):
        import ast
        self.collected_joint_angles = {}
        with open(csv_name, 'r', newline='') as csvfile:
            csvfile.seek(0)  
            reader = csv.DictReader(csvfile)
            for row in reader:
                arm = row['arm']
                angles_key = f'{arm}_angles'
                # Parse the joint_angles string back to tuple
                angles = tuple(ast.literal_eval(row['joint_angles']))
                if angles_key not in self.collected_joint_angles:
                    self.collected_joint_angles[angles_key] = []
                self.collected_joint_angles[angles_key].append(angles)
                        
    def compute_fk(self, name):
        if name == "psm1":
            psm1_joint_angles = self.collected_joint_angles['psm1_angles']
            data = []
            for angles in psm1_joint_angles:
                xyz,_ = forward_kinematics(self.psm1_kin, angles)
                data.append(xyz)
            return data
        if name == "psm2":
            psm2_joint_angles = self.collected_joint_angles['psm2_angles']
            data = []
            for angles in psm2_joint_angles:
                xyz,_ = forward_kinematics(self.psm2_kin, angles)
                data.append(xyz)
            return data
        if name == "ecm":
            ecm_joint_angles = self.collected_joint_angles['ecm_angles']
            data = []
            for angles in ecm_joint_angles:                
                xyz,_ = forward_kinematics(self.ecm_kin, angles)
                data.append(xyz)
            return data

    def dist(self, a,b):
        return np.sqrt( sum([ (i-j)**2 for i,j in zip(a,b)]))
    
    # This function is supposed to be fed to scipy
    def objective_function(self, xyzrpy):
        if len(xyzrpy) > 2:
            xyzrpy = [ tuple(xyzrpy[0:3]), tuple(xyzrpy[3:])]
        if self.psm1_data == None:
            self.psm1_data = self.compute_fk('psm1')
        if self.psm2_data == None:
            self.psm2_data = self.compute_fk('psm2')
        if self.ecm_data == None:
            self.ecm_data = self.compute_fk('ecm')
        
        T = to_homo_mat(xyzrpy)

        if self.mode == self.REGISTRATION_MODE.PSM1_ECM:
            pos_in_psm1rf = []
            for ecm_xyz in self.ecm_data:
                ecm_xyz = np.insert(ecm_xyz,3,1)
                ecm_xyz = ecm_xyz.reshape(4, 1)
                temp = T @ ecm_xyz
                # Extract only the position (first 3 elements) from homogeneous coordinates
                pos = temp[0:3, 0]
                pos_in_psm1rf.append(pos)
        else:
            pos_in_psm1rf = []
            for psm2_xyz in self.psm2_data:
                
                psm2_xyz = np.insert(psm2_xyz,3,1)
                psm2_xyz = psm2_xyz.reshape(4, 1)
                temp = T @ psm2_xyz
                # Extract only the position (first 3 elements) from homogeneous coordinates
                pos = temp[0:3, 0]
                pos_in_psm1rf.append(pos)
        
        diff = []

        for a,b in zip(self.psm1_data, pos_in_psm1rf):          
            diff.append(self.dist(a,b))
        return sum(diff)
        
    
    def find_everything_related_to_world(self, arm_name, xyzrpy):
        if len(xyzrpy) > 2:
            xyzrpy = [ tuple(xyzrpy[0:3]), tuple(xyzrpy[3:])]
        
        # Get world to PSM1 base chain using kdl_parser_py
        psm1_kin_world_to_base = self.psm1_tree.getChain("world", "psm1_psm_base_link")
        Twp1pos , Twp1Rotation = forward_kinematics(psm1_kin_world_to_base, np.zeros(psm1_kin_world_to_base.getNrOfJoints()))
                
        Twp1 = pos_rot_to_homo_mat(Twp1pos, Twp1Rotation)
                
        if arm_name == 'psm2':
            Tp12 = to_homo_mat(xyzrpy)
            
            Twp2 = Twp1 @ Tp12
            Twp2_euler = to_pos_euler(Twp2)
            return Twp2_euler
        
        if arm_name == 'ecm':
            # Get chain from second joint to ECM base using kdl_parser_py
            ecm_kin_sj_to_base = self.ecm_tree.getChain("world", "ECM_ecm_base_link")
            Tse_pos, Tse_rot = forward_kinematics(ecm_kin_sj_to_base, np.zeros(ecm_kin_sj_to_base.getNrOfJoints()))
            Tse = pos_rot_to_homo_mat(Tse_pos, Tse_rot)
            Tp1E = to_homo_mat(xyzrpy)
            Tws = Twp1 @ Tp1E @ np.linalg.inv(Tse)
            Tws_euler = to_pos_euler(Tws)
            
            return Tws_euler
    
    def optimize_bases(self):
        # Test with zero transformation first
        # print("\n--- Testing objective function with zero transformation ---")
        # zero_error = self.objective_function([0, 0, 0, 0, 0, 0])
        # print(f"Error with zero transformation: {zero_error}")
        print("--- Starting optimization ---\n")
        if self.mode == self.REGISTRATION_MODE.PSM1_PSM2:
            # [-0.08201892  0.11983251 -0.01110584] [-0.95928483  1.24202957  1.28026907]
            initial_guess = np.array([ (-0.08201892,  0.11983251, -0.01110584), (-0.95928483,  1.24202957,  1.28026907)]).flatten()
        elif self.mode == self.REGISTRATION_MODE.PSM1_ECM:
            initial_guess = np.array([ (0.14003942500299224, 0.06195379762116297, -0.03353172441490421), (0.6234358306665349, 0.15600848673307866, 2.6228419950738715)]).flatten()
        
        res = minimize(self.objective_function, initial_guess, method='nelder-mead', options={'xatol':1e-12, 'fatol':1e-12, 'disp':False, 'maxiter': 100000, 'maxfev':100000})
        print(res)
        print(f"Location of {self.mode} relative to PMS1: {res.x}")
        print(f"Final error: {res.fun}")
        print(self.mode)   
        if self.mode == self.REGISTRATION_MODE.PSM1_PSM2:
            print('psm2 relative to world: ')
            v = self.find_everything_related_to_world('psm2', res.x)
     #       print("""xyz="{} {} {}" rpy="{} {} {}" """.format(v[0], v[1]) )
            print("""xyz="{0} {1} {2}" rpy="{3} {4} {5}" """.format(v[0][0],v[0][1],v[0][2],v[1][0],v[1][1],v[1][2]))
        if self.mode == self.REGISTRATION_MODE.PSM1_ECM:
            print('ecm relative to world: ')
            v = self.find_everything_related_to_world('ecm', res.x)
            print("""xyz="{0} {1} {2}" rpy="{3} {4} {5}" """.format(v[0][0],v[0][1],v[0][2],v[1][0],v[1][1],v[1][2]))
    

def main(args=None):    
    rclpy.init(args=args)
    node = coregistrator()
    
    # 1. Start ROS 2 spinning in a background thread
    # This allows callbacks (psm1_read_cb, etc.) to run while we use input()
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    try:
        # 2. Run your manual collection loop
        node.collect()
        
        # 3. Save after the loop finishes
        csv_path = os.path.join(os.getcwd(), 'calibration_data.csv')
        print(f"Saving collected data to {csv_path}...")
        node.save_to_csv(csv_path)
        
        print("\n\n\nPSM1_ECM \n")
        node.mode = node.REGISTRATION_MODE.PSM1_ECM
        node.optimize_bases()
        
        print("\n\n\nPSM1_PSM2 \n")
        node.mode = node.REGISTRATION_MODE.PSM1_PSM2
        node.optimize_bases()
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
