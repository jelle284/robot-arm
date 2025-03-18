import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from stepper_msgs.msg import StepperTrajectory
from std_msgs.msg import String, Int32MultiArray
import time
import stepper_joint_conversion as cvt
    
class StepperTrajectoryProcessor(Node):
    def __init__(self):
        super().__init__("StepperTrajectoryProcessor")

        # Initialize the action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'stepper_trajectory_processor/follow_joint_trajectory',
            self.execute_callback
        )

        # Publisher for stepper commands
        self._stepper_publisher = self.create_publisher(
            StepperTrajectory,
            '/stepper_cmd',
            10  # QoS depth
        )
        # Subscriber for stepper status
        self._stepper_sts_subscriber = self.create_subscription(
            String,
            '/stepper_sts',
            self.stepper_sts_callback,
            10)

        # Subscriber for stepper step counting
        self._stepper_cnt_subscriber = self.create_subscription(
            Int32MultiArray,
            '/stepper_cnt',
            self.stepper_cnt_callback,
            10)

        # Load motor configuration file
        self.motor_config = cvt.load_config("motors.yaml")

        # Variables to track acknowledgment and step counts
        self.ack_received = False
        self.executed_points = 0

    def stepper_sts_callback(self, msg):
        """Handle acknowledgment from the ESP32."""
        if msg.data == "ACK":
            self.ack_received = True
            self.get_logger().info("Received acknowledgment from ESP32")

    def stepper_cnt_callback(self, msg: Int32MultiArray):
        """Accumulate step counts and provide feedback."""
        self.get_logger().info(f"Stepper count {msg.data}")
        self.executed_points += 1

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        trajectory: JointTrajectory = goal_handle.request.trajectory
        self.get_logger().info("Goal handle recieved")

        # Convert and publish stepper instructions
        stepper_trajectory: StepperTrajectory = cvt.convert_joint_to_stepper_trajectory(trajectory, self.motor_config)
        self._stepper_publisher.publish(stepper_trajectory)

        # Reset state variables
        self.ack_received = False
        self.executed_points = 0

        # Wait for acknowledgment with timeout
        start_time = time.time()
        while not self.ack_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 5.0:  # 5 seconds timeout
                self.get_logger().error("Timeout waiting for ACK")
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result

        # Wait for all steps to be executed with timeout
        start_time = time.time()
        while self.executed_points < len(stepper_trajectory.points):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 30.0:  # 30 seconds timeout
                self.get_logger().error("Timeout waiting for step execution")
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                return result
            #TODO: Feedback

        # Update the joint state variable and save to file
        if trajectory.points:
            self.current_joint_states["positions"] = list(trajectory.points[-1].positions)
            self.save_joint_states()

        # Mark the goal as succeeded
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        self.get_logger().info("Goal execution complete")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = StepperTrajectoryProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
