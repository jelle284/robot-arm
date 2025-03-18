from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stepper_trajectory_processor',
            executable='stepper_trajectory_processor',
            name='stepper_trajectory_processor',
            output='screen',
        ),
        Node(
            package='stepper_trajectory_processor',
            executable='stepper_test_node',
            name='stepper_test_node',
            output='screen',
        ),
        Node(
            package='stepper_trajectory_processor',
            executable='stepper_joint_states_node',
            name='stepper_joint_states_node',
            output='screen',
        ),
    ])