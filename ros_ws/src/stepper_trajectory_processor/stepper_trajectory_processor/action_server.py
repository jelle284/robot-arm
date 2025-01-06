import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from stepper_msgs.msg import StepperTrajectory, StepperTrajectoryPoint
import json
import os
import trajcetory_processor as proc
import viz_trajectory as viz

class ProcessorActionServer(Node):
    def __init__(self):
        super().__init__("StepperTrajectoryProcessor")
        
        # Initialize the action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'stepper_trajectory_processor/follow_joint_trajectory',
            self.execute_callback
        )

        # Publisher for the last trajectory point
        self._point_publisher = self.create_publisher(
            JointTrajectoryPoint,
            '/cmd_point',
            10  # QoS depth
        )

        # Publisher for joint states
        self._joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10  # QoS depth
        )

        # Load initial joint states from file
        self.joint_state_file = "joint_states.json"
        self.current_joint_states = self.load_joint_states()
        # Periodic publishing of joint states
        self.timer = self.create_timer(1.0, self.publish_joint_states)  # Publish every 1 second

        # Load motor configuration file
        self.motor_config = proc.load_config("motors.yaml")

    def load_joint_states(self):
        """Load joint states from a JSON file."""
        if os.path.exists(self.joint_state_file):
            with open(self.joint_state_file, "r") as file:
                data = json.load(file)
                self.get_logger().info(f"Loaded joint states: {data}")
                return data
        else:
            self.get_logger().warn(f"File {self.joint_state_file} not found. Initializing default joint states.")
            return { "positions": [0.0] * 6 }  # Default to 6 joints at position 0

    def save_joint_states(self):
        """Save the current joint states to a JSON file."""
        with open(self.joint_state_file, "w") as file:
            json.dump(self.current_joint_states, file)
        self.get_logger().info(f"Saved joint states: {self.current_joint_states}")

    def publish_joint_states(self):
        """Publish the current joint states periodically."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"j{i}" for i in range(len(self.current_joint_states["positions"]))]
        joint_state_msg.position = self.current_joint_states["positions"]
        self._joint_state_publisher.publish(joint_state_msg)
    
    def publish_delta_move(self, trajectory):
        # Publish the last JointTrajectoryPoint to /cmd_point with position difference
        if trajectory.points:
            last_point = trajectory.points[-1]
            current_positions = self.current_joint_states["positions"]
            position_difference = [p - c for p, c in zip(last_point.positions, current_positions)]
            
            diff_point = JointTrajectoryPoint()
            diff_point.positions = position_difference
            diff_point.time_from_start = last_point.time_from_start
            
            self._point_publisher.publish(diff_point)
            self.get_logger().info("Published the position difference to /cmd_point")
    
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        trajectory = goal_handle.request.trajectory
        
        # Temporary solution
        self.publish_delta_move(trajectory)

        # TODO: publish proper stepper instructions
        stepper_trajectory: StepperTrajectory = proc.convert_joint_to_stepper_trajectory(trajectory, self.motor_config)
        viz.plot_trajectory(stepper_trajectory, trajectory, self.motor_config)

        # TODO: Feedback
        feedback = FollowJointTrajectory.Feedback()
        goal_handle.publish_feedback(feedback)

        # Update the joint state variable and save to file
        if trajectory.points:
             self.current_joint_states["positions"] = list(trajectory.points[-1].positions)
             self.save_joint_states()

        # Simulate processing and mark the goal as succeeded
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        self.get_logger().info("Goal execution complete")
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ProcessorActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
