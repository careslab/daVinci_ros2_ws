from scipy.spatial.transform import Rotation
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
import subprocess
import os

def to_homo_mat(xyzrpy):
    """Convert xyz position and rpy angles to homogeneous transformation matrix using PyKDL Eigen"""
    if isinstance(xyzrpy, (list, tuple)) and len(xyzrpy) == 2:
        pos, rpy = xyzrpy
    else:
        pos = xyzrpy[:3]
        rpy = xyzrpy[3:]
    
    # Use PyKDL's Eigen-based rotation from RPY
    rotation = PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2])
    
    # Create PyKDL Frame (Eigen-based transformation matrix)
    frame = PyKDL.Frame(rotation, PyKDL.Vector(pos[0], pos[1], pos[2]))
    
    # Convert to numpy 4x4 matrix for compatibility with downstream code
    T = np.eye(4)
    T[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                          [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                          [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
    T[:3, 3] = [frame.p.x(), frame.p.y(), frame.p.z()]
    return T

def to_pos_euler(T):
    """Convert homogeneous transformation matrix to xyz position and rpy angles using PyKDL Eigen"""
    # Extract position
    pos = T[:3, 3]
    
    # Create PyKDL Rotation matrix using Eigen
    rotation_kdl = PyKDL.Rotation(
        T[0, 0], T[0, 1], T[0, 2],
        T[1, 0], T[1, 1], T[1, 2],
        T[2, 0], T[2, 1], T[2, 2]
    )
    
    # Convert from RPY using PyKDL's Eigen-based method
    roll, pitch, yaw = rotation_kdl.GetRPY()
    rpy = np.array([roll, pitch, yaw])
    
    return (pos, rpy)

def pos_rot_to_homo_mat(pos, rot):
    """Convert position vector and rotation matrix to 4x4 homogeneous transformation matrix.
    
    Args:
        pos: 3-element position vector (x, y, z)
        rot: 3x3 rotation matrix (or PyKDL.Rotation object)
    
    Returns:
        4x4 numpy transformation matrix
    """
    T = np.eye(4)
    
    # Handle both numpy arrays and PyKDL Rotation objects
    if isinstance(rot, PyKDL.Rotation):
        # Convert PyKDL Rotation to numpy array
        T[:3, :3] = np.array([[rot[0, 0], rot[0, 1], rot[0, 2]],
                              [rot[1, 0], rot[1, 1], rot[1, 2]],
                              [rot[2, 0], rot[2, 1], rot[2, 2]]])
    else:
        # Assume it's a numpy array or list-like
        T[:3, :3] = np.array(rot)
    
    # Set position (handle both 1D array and column vector)
    T[:3, 3] = np.array(pos).flatten()[:3]
    
    return T

def forward_kinematics(chain, joint_angles):
    """Compute forward kinematics using PyKDL"""
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    nr_joints = chain.getNrOfJoints()

    # ECM chain includes mimic joints in KDL. Expand 4 independent angles
    # [yaw, pitch, insertion, roll] to KDL's 6-joint ordering.
    # This is because the ECM's URDF defines mimic joints for the pitch, which KDL treats as separate joints.
    if nr_joints == 6 and len(joint_angles) == 4:
        yaw, pitch, insertion, roll = joint_angles
        joint_angles = [yaw, pitch, -pitch, pitch, insertion, roll]

    q_kdl = PyKDL.JntArray(nr_joints)
    for i in range(0, min(len(joint_angles), nr_joints)):
        q_kdl[i] = joint_angles[i]
    
    frame = PyKDL.Frame()
    status = fk_solver.JntToCart(q_kdl, frame)
    if status < 0:
        raise RuntimeError("FK computation failed")
    
    # Extract position and rotation from frame
    pos = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
    rot = frame.M  
    
    return pos, rot


def load_urdf_from_file(file_path):
    """Load URDF from a file, processing XACRO if necessary"""
    try:
        if file_path.endswith('.xacro'):
            # Process XACRO file to URDF using xacro command
            # Set up environment for xacro to find package dependencies
            env = os.environ.copy()
            
            # Add common ROS package paths to ROS_PACKAGE_PATH
            ros_paths = [
                '/home/careslab/ros2_ws/install',
                '/home/careslab/ros2_ws/install/dvrk_model/share',
                '/opt/ros/jazzy/share',
            ]
            existing_path = env.get('ROS_PACKAGE_PATH', '')
            if existing_path:
                ros_paths.append(existing_path)
            env['ROS_PACKAGE_PATH'] = os.pathsep.join(ros_paths)
            
            try:
                print(f"Running xacro with ROS_PACKAGE_PATH: {env['ROS_PACKAGE_PATH']}")
                result = subprocess.run(['xacro', file_path], 
                                      env=env, 
                                      capture_output=True, 
                                      text=True,
                                      check=True)
                urdf_string = result.stdout
            except subprocess.CalledProcessError as e:
                raise RuntimeError(f"xacro processing failed for {file_path}\nStderr: {e.stderr}")
            except FileNotFoundError:
                raise RuntimeError(f"xacro command not found. Install with: sudo apt install ros-jazzy-xacro")
        else:
            # Load plain URDF file
            with open(file_path, 'r') as f:
                urdf_string = f.read()
        
        return URDF.from_xml_string(urdf_string)
    except Exception as e:
        raise RuntimeError(f"Failed to load URDF from file {file_path}: {e}")