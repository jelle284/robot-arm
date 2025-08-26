from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_stepper_controller"),
            "config",
            "six_dof_arm_controllers.yaml",
        ]
    )


    # Get URDF via xacro
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_stepper_controller"),
                    "urdf",
                    "robot.urdf.xacro",
                ]
            ),
        ]
    )
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_controllers
                ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        ),
    ])
