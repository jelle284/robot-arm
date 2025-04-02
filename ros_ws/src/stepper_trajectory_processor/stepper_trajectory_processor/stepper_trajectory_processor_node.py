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

from threading import Event
import time
import json
import os
from array import array

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

MAX_PARTIAL_SIZE    = 32
MAX_TOTAL_SIZE      = 256

def process_points(points: list, tx_id):
    msg_list = []
    N = len(points)
    num_tx = (N // MAX_PARTIAL_SIZE)
    if N % MAX_PARTIAL_SIZE > 0:
        num_tx += 1
    first = 0
    for n in range(num_tx):
        last = min(first+MAX_PARTIAL_SIZE, N)
        msg = StepperTrajectory()
        msg.points = points[first:last]
        msg.transmission_id = tx_id
        msg.partial_total=num_tx
        msg.partial_count=n+1
        msg_list.append(msg)
        first = last
    return msg_list

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

        # Create an event for when feedback is recieved
        self.stepper_fb_event = Event()
        self.stepper_fb_msg = StepperFeedback()

        # Load stored joint states
        self.joint_state_file = "joint_states.json"
        joint_states = self.load_joint_states()
        if joint_states == None:
            joint_states = [0.0] * motor_conf["num_joints"]
        self.joint_states_now = array('d', joint_states["positions"])
        self.joint_states_prev = self.joint_states_now

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
            QOS_PROFILE
        )

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Periodic publishing of joint states
        self.joint_state_timer = self.create_timer(1.0, self.publish_joint_states)  # Publish every 1 second

        # Subscriber for stepper feedback
        self.stepper_fb_subscriber = self.create_subscription(
            StepperFeedback,
            '/stepper_fb',
            self.stepper_fb_callback,
            QOS_PROFILE
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
            json.dump({"positions": self.joint_states_now.tolist()}, file)
        self.get_logger().info(f"Saved joint states: {self.joint_states_now}")

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"j{i}" for i in range(len(self.joint_states_now))]
        joint_state_msg.position = self.joint_states_now
        self.joint_state_publisher.publish(joint_state_msg)
    
    def stepper_fb_callback(self, msg: StepperFeedback):
        self.stepper_fb_msg = msg
        self.stepper_fb_event.set()

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        result = FollowJointTrajectory.Result()
        trajectory: JointTrajectory = goal_handle.request.trajectory
        self.get_logger().info(f"Goal handle recieved. Trajectory has {len(trajectory.points)} points")
        
        if len(trajectory.points) >= MAX_TOTAL_SIZE:
            self.get_logger().error(f"Too many points in trajectory: {len(trajectory.points)}.")
            return result
        
        # Process trajectory points
        prev_point = None
        stepper_points = []
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
        self.get_logger().info(f"Transmission {self.transmission_num} starting.")
        msgs = process_points(stepper_points, self.transmission_num)
        for msg in msgs:
            self.stepper_cmd_publisher.publish(msg)
            time.sleep(0.020)

        # Monitor execution and submit feedback
        while 1:
            self.stepper_fb_event.wait()
            self.stepper_fb_event.clear()
            msg = self.stepper_fb_msg
            joints = cvt.convert_steps_to_joint(msg.steps_executed, self.tf)
            self.joint_states_now = array('d', [i+j for i,j in zip(self.joint_states_prev, joints)])
            try:
                desired_point: JointTrajectoryPoint = trajectory.points[msg.current_point]
                action_feedback = FollowJointTrajectory.Feedback()
                action_feedback.actual.positions = self.joint_states_now
                action_feedback.desired = desired_point
                action_feedback.error.positions = array(
                    'd',
                    [i-j for i,j in zip(self.joint_states_now, desired_point.positions)])
                goal_handle.publish_feedback(action_feedback)
            except IndexError:
                self.get_logger().error(f"Index error in trajectory at point {msg.current_point}")
            if msg.current_point == msg.total_points:
                self.joint_states_prev = self.joint_states_now
                self.save_joint_states()
                self.tx_done = True
                self.get_logger().info(f"Transmission {msg.transmission_id} done.")
                break

        # Mark the goal as succeeded
        goal_handle.succeed()
        self.transmission_num += 1
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
