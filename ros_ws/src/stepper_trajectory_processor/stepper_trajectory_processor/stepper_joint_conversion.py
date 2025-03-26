import yaml
import os
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from stepper_msgs.msg import StepperTrajectory, StepperPoint
from array import array
import numpy as np

def load_config(filename, local=False):
    """
    Loads motor yaml config file
    """
    package_share_directory = get_package_share_directory('stepper_trajectory_processor')
    if local:

        config_file_path = os.path.join('ros_ws','src', 'stepper_trajectory_processor', 'config', filename)
    else:
        config_file_path = os.path.join(package_share_directory, 'config', filename)
    with open(config_file_path, 'r') as file:
        motor_config = yaml.safe_load(file)
        return motor_config

def make_tf(config: dict):
    """
    returns: the transform and the inverse transform
    """
    motors = config["motors"]
    tf = np.zeros((config["num_joints"],len(motors)))
    for i, k in enumerate(motors):
        val = motors[k]
        act_ratio = 1.0 / (val["gear_ratio"]*val["step_resolution"])
        for joint in val["joints"]:
            tf[joint["index"], i] = 2*np.pi*act_ratio*joint["ratio"]
    return tf, np.linalg.pinv(tf)

def convert_joint_to_steps(positions: array, inv_tf: np.array):
    steps = inv_tf@positions
    return array('i', steps.astype(int).tolist())

def convert_steps_to_joint(steps: array, tf: np.array):
    joints = tf@steps
    return array('d', joints.tolist())

def convert_joint_to_stepper_trajectory(joint_trajectory: JointTrajectory, config):
    """
    Converts a JointTrajectory to a StepperTrajectory.

    Args:
        joint_trajectory (JointTrajectory): Input trajectory from MoveIt.
        config (dict): Motor configuration loaded from YAML.

    Returns:
        StepperTrajectory: Converted trajectory for stepper motors.
    """
    # Create the StepperTrajectory message
    stepper_trajectory = StepperTrajectory()
    stepper_trajectory.header = joint_trajectory.header
    stepper_trajectory.motor_names = list(config['motors'].keys())
    
    prev_point = None
    
    for point in joint_trajectory.points:
        point: JointTrajectoryPoint
        if not prev_point:
            prev_point = point
            continue
        stepper_point = StepperPoint()

        # Calculate delta time
        delta_sec = point.time_from_start.sec - prev_point.time_from_start.sec
        delta_nanosec = point.time_from_start.nanosec - prev_point.time_from_start.nanosec
        stepper_point.duration_us = int(1e6*delta_sec + delta_nanosec*1e-3)
        if stepper_point.duration_us == 0: continue

        # Calculate delta movement
        joint_delta = [0] * len(point.positions)
        for i in len(point.positions):
            joint_delta[i] = point.positions[joint_index] - prev_point.positions[joint_index]

        # Initialize motor steps
        stepper_steps = [0] * len(config['motors'])

        for motor_name, motor_config in config['motors'].items():
            motor_index = stepper_trajectory.motor_names.index(motor_name)
            gear_ratio = motor_config['gear_ratio']
            step_resolution = motor_config['step_resolution']
            joint_contributions = motor_config['joints']

            # Compute motor position and velocity for the current point
            motor_steps = 0
            for joint in joint_contributions:
                joint_index = joint['index']
                joint_ratio = joint['ratio']

                # Compute contribution of joint
                motor_steps += rad2turns(joint_delta[joint_index]) * gear_ratio * joint_ratio * step_resolution

            # Assign computed values to the arrays
            stepper_steps[motor_index] = int(motor_steps)

        # Populate StepperTrajectoryPoint
        stepper_point.steps = stepper_steps

        # Append to the trajectory
        stepper_trajectory.points.append(stepper_point)
        
        # Update previous point
        prev_point = point
        
    return stepper_trajectory

if __name__ == '__main__':
    config = load_config("motors.yaml", local=True)
    tf, inv_tf = make_tf(config)
    joints = array('d', [1.0]*6)
    steps = convert_joint_to_steps(joints, inv_tf)
    re_joints = convert_steps_to_joint(steps, tf)
    print(joints)
    print(steps)
    print(re_joints)