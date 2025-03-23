import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from stepper_msgs.msg import StepperTrajectoryPoint
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray
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
            StepperTrajectoryPoint,
            #'/stepper_cmd',
            '/cmd_point',
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
        result = FollowJointTrajectory.Result()
        trajectory: JointTrajectory = goal_handle.request.trajectory
        self.get_logger().info("Goal handle recieved")

        # Process trajectory points
        prev_point = None
        
        for point in trajectory.points:
            point: JointTrajectoryPoint
            N = len(point.positions)
            if not prev_point:
                prev_point = point
                continue
            stepper_point = StepperTrajectoryPoint()
            stepper_point.steps = [0]*N
            # Calculate delta time
            delta_sec = point.time_from_start.sec - prev_point.time_from_start.sec
            delta_nanosec = point.time_from_start.nanosec - prev_point.time_from_start.nanosec
            stepper_point.duration_us = int(1e6*delta_sec + delta_nanosec*1e-3)
            if stepper_point.duration_us == 0: continue

            # Calculate delta movement
            joint_delta = Float64MultiArray()
            joint_delta.data = [0] * N
            for i in range(N):
                joint_delta.data[i] = point.positions[i] - prev_point.positions[i]
            steps = cvt.convert_joint_to_steps(joint_delta, self.motor_config)
            stepper_point.steps = steps.data
            self._stepper_publisher.publish(stepper_point)
            prev_point = point
        # Mark the goal as succeeded
        goal_handle.succeed()
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
