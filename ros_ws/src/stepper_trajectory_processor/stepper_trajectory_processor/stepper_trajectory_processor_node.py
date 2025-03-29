import rclpy
from rclpy.action import ActionServer
import rclpy.executors
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from stepper_msgs.msg import StepperTrajectory, StepperPoint, StepperFeedback
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import stepper_joint_conversion as cvt

import json
import os
from array import array
import time

QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

MAX_POINTS=32

class StepperTrajectoryProcessor(Node):
    def __init__(self):
        super().__init__("StepperTrajectoryProcessor")
        
        # Load motor configuration file
        motor_conf = cvt.load_config("motors.yaml")
        self.motor_conf = motor_conf

        # create transformation matrices
        tf, inv_tf = cvt.make_tf(motor_conf)
        self.tf = tf
        self.inv_tf = inv_tf

        # Count transmissions
        self.transmission_num = 0

        # Load stored joint states
        self.joint_state_file = "joint_states.json"
        joint_states = self.load_joint_states()
        if joint_states == None:
            joint_states = { "positions": [0.0] * motor_conf["num_joints"] }
        self.current_joint_states = joint_states

        # Initialize the action server
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'stepper_trajectory_processor/follow_joint_trajectory',
            self.execute_callback
        )

        # Publisher for stepper commands
        self.stepper_cmd_publisher = self.create_publisher(
            StepperTrajectory,
            '/stepper_cmd',
            #QOS_BEST_EFFORT
            10
        )

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Periodic publishing of joint states
        self.timer = self.create_timer(1.0, self.publish_joint_states)  # Publish every 1 second

        # Subscriber for stepper feedback
        self.stepper_fb_subscriber = self.create_subscription(
            StepperFeedback,
            '/stepper_fb',
            self.stepper_fb_callback,
            #QOS_BEST_EFFORT
            10
            )
        
        self.get_logger().info(f"Succesfully initialized stepper trajectory processor.")
        
    def load_joint_states(self):
        if os.path.exists(self.joint_state_file):
            with open(self.joint_state_file, "r") as file:
                data = json.load(file)
                self.get_logger().info(f"Loaded joint states: {data}")
                return data
        else:
            self.get_logger().warn(f"File {self.joint_state_file} not found.")
            return None
    
    def save_joint_states(self):
        with open(self.joint_state_file, "w") as file:
            json.dump(self.current_joint_states, file)
        self.get_logger().info(f"Saved joint states: {self.current_joint_states}")

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"j{i}" for i in range(len(self.current_joint_states["positions"]))]
        joint_state_msg.position = self.current_joint_states["positions"]
        self.joint_state_publisher.publish(joint_state_msg)

    def stepper_fb_callback(self, msg: StepperFeedback):
        print(msg)

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        result = FollowJointTrajectory.Result()
        trajectory: JointTrajectory = goal_handle.request.trajectory
        self.get_logger().info(f"Goal handle recieved. Trajectory has {len(trajectory.points)} points")
        # initialize stepper trajectory
        stepper_cmd = StepperTrajectory()
        stepper_cmd.transmission_id = self.transmission_num
        stepper_cmd.motor_names = list(self.motor_conf["motors"].keys())
        stepper_points = []

        # Process trajectory points
        prev_point = None

        for point in trajectory.points:
            point: JointTrajectoryPoint
            if not prev_point:
                prev_point = point
                continue
            stepper_point = StepperPoint()

            # Calculate delta time
            delta_sec = point.time_from_start.sec - prev_point.time_from_start.sec
            delta_nanosec = point.time_from_start.nanosec - prev_point.time_from_start.nanosec
            stepper_point.duration_ms = int(1e3*delta_sec + delta_nanosec*1e-6)
            if stepper_point.duration_ms == 0: continue

            # Calculate steps to move
            joint_delta = array('d', [p1 - p2 for p1, p2 in zip(point.positions, prev_point.positions)])
            stepper_point.steps = cvt.convert_joint_to_steps(joint_delta, self.inv_tf)
            stepper_points.append(stepper_point)
            prev_point = point

        # Publish the trajectory
        stepper_cmd.points = stepper_points
        self.stepper_cmd_publisher.publish(stepper_cmd)
        self.transmission_num += 1

        # Monitor execution and submit feedback
        #TODO

        # Mark the goal as succeeded
        goal_handle.succeed()
        self.get_logger().info("Goal execution complete")
        return result

def main(args=None):
    rclpy.init(args=args)
    mtx = rclpy.executors.MultiThreadedExecutor(4)
    node = StepperTrajectoryProcessor()
    rclpy.spin(node, executor=mtx)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
