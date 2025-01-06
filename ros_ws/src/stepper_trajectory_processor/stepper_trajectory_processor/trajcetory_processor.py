from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from stepper_msgs.msg import StepperTrajectory, StepperTrajectoryPoint
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def rad2turns(rad):
    """
    converts angle in radians to number of turns
    """
    return rad/(2*3.14159265359)

def load_config(config_filename: str):
    # Get the package share directory path
    package_share_directory = get_package_share_directory('stepper_trajectory_processor')
    
    # Build the full path to the config file
    config_file_path = os.path.join(package_share_directory, 'config', config_filename)
    
    # Load the YAML configuration
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)

    return config

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
        if not prev_point:
            prev_point = point

        point: JointTrajectoryPoint
        stepper_point = StepperTrajectoryPoint()

        # Initialize motor steps and velocities
        stepper_steps = [0] * len(config['motors'])
        stepper_velocities = [0.0] * len(config['motors'])

        for motor_name, motor_config in config['motors'].items():
            motor_index = stepper_trajectory.motor_names.index(motor_name)
            gear_ratio = motor_config['gear_ratio']
            step_resolution = motor_config['step_resolution']
            joint_contributions = motor_config['joints']

            # Compute motor position and velocity for the current point
            motor_steps = 0
            motor_velocity = 0
            for joint in joint_contributions:
                joint_index = joint['index']
                joint_ratio = joint['ratio']

                # Compute position contribution
                joint_delta = point.positions[joint_index] - prev_point.positions[joint_index]
                motor_steps += rad2turns(joint_delta) * gear_ratio * joint_ratio * step_resolution

                # Compute velocity contribution if available
                if point.velocities:
                    joint_velocity = point.velocities[joint_index]
                    motor_velocity += rad2turns(joint_velocity) * gear_ratio * joint_ratio * step_resolution

            # Assign computed values to the arrays
            stepper_steps[motor_index] = int(round(motor_steps))
            stepper_velocities[motor_index] = int(round(motor_velocity))

        # Populate StepperTrajectoryPoint
        stepper_point.steps = stepper_steps
        stepper_point.steps_per_sec = stepper_velocities
        stepper_point.time_from_start_us = point.time_from_start.sec * 1e6 + point.time_from_start.nanosec/1e3

        # Append to the trajectory
        stepper_trajectory.points.append(stepper_point)
        
        # Update previous point
        prev_point = point

    return stepper_trajectory