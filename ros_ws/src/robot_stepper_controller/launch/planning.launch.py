from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_stepper_controller"),
            "config",
            "six_dof_arm_controllers.yaml",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("six_dof_arm_moveit_config"),
            "config",
            "moveit.rviz",
        ]
    )
    moveit_config = (
        MoveItConfigsBuilder("six_dof_arm")
        .robot_description(file_path="config/six_dof_arm.urdf")
        .robot_description_semantic(file_path="config/six_dof_arm.srdf")
        .to_moveit_configs()
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
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        ),
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
