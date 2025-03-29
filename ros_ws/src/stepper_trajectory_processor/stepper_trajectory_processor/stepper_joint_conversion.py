import yaml
import os
from ament_index_python.packages import get_package_share_directory
from array import array
import numpy as np

def load_config(filename, local=False):
    """
    Loads motor yaml config file
    """
    motor_config: dict
    package_share_directory = get_package_share_directory('stepper_trajectory_processor')
    if local:

        config_file_path = os.path.join('src', 'stepper_trajectory_processor', 'config', filename)
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

if __name__ == '__main__':
    config = load_config("motors.yaml", local=True)
    tf, inv_tf = make_tf(config)
    joints = array('d', [1.0]*6)
    steps = convert_joint_to_steps(joints, inv_tf)
    re_joints = convert_steps_to_joint(steps, tf)
    print(joints)
    print(steps)
    print(re_joints)