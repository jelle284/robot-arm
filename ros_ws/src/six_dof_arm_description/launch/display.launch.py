import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'six_dof_arm_description'
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'six_dof_arm.urdf'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'six_dof_arm.rviz'
    )
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file).read()
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_map_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
