import rclpy
from rclpy.action import ActionServer, ActionClient
import rclpy.executors
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from stepper_msgs.action import FollowStepperTrajectory
from stepper_msgs.msg import StepperTrajectoryPoint

from sensor_msgs.msg import JointState

import stepper_joint_conversion as cvt

import time
import json
import os
from array import array

MAX_TOTAL_SIZE = 128

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

        # Load stored joint states
        self.joint_state_file = "joint_states.json"
        joint_states = self.load_joint_states()
        if joint_states == None:
            joint_states = [0.0] * motor_conf["num_joints"]
        self.joint_states_now = array('d', joint_states["positions"])
        self.joint_states_prev = self.joint_states_now

        # Initialize action server
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'stepper_trajectory_processor/follow_joint_trajectory',
            self.execute_callback
        )

        # Initialize action client
        self.client = ActionClient(
            self, 
            FollowStepperTrajectory, 
            '/follow_stepper_trajectory')

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Periodic publishing of joint states
        self.joint_state_timer = self.create_timer(1.0, self.publish_joint_states)  # Publish every 1 second

        # Store server goal handle
        self.current_goal_handle = None
        
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

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        result = FollowJointTrajectory.Result()
        server_req: FollowJointTrajectory.Goal = goal_handle.request
        trajectory = server_req.trajectory

        self.get_logger().info(f"Goal handle recieved. Trajectory has {len(trajectory.points)} points")
        
        if len(trajectory.points) >= MAX_TOTAL_SIZE:
            self.get_logger().error(f"Too many points in trajectory: {len(trajectory.points)}.")
            goal_handle.abort()
            return result
        self.current_goal_handle = goal_handle

        # Process trajectory points
        first_point = None
        stepper_points = []
        for point in trajectory.points:
            point: JointTrajectoryPoint
            
            if not first_point:
                first_point = point
                
            stepper_point = StepperTrajectoryPoint()

            # Calculate time in micro seconds
            stepper_point.time_from_start = int(1e6*point.time_from_start.sec + point.time_from_start.nanosec*1e-3)

            # Calculate steps to move in microsteps
            joint_delta = array('d', [1e6*(p1 - p2) for p1, p2 in zip(point.positions, first_point.positions)])
            stepper_point.positions = cvt.convert_joint_to_steps(joint_delta, self.inv_tf)

            # Calculate velocity in microsteps per millisecond
            joint_vel = array('d', [v*1e3 for v in point.velocities])
            stepper_point.velocities = cvt.convert_joint_to_steps(joint_vel, self.inv_tf)

            # Calculate acceleration in microsteps per millisecond squared
            stepper_point.accelerations = cvt.convert_joint_to_steps(point.accelerations, self.inv_tf)

            stepper_points.append(stepper_point)

        # Publish the trajectory
        client_req = FollowStepperTrajectory.Goal()
        client_req.trajectory.points = stepper_points
        send_goal_future = self.client.send_goal_async(client_req, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"Published trajectory to client")

        # Mark the goal as succeeded
        goal_handle.succeed()
        self.get_logger().info("Goal execution complete")
        return result
    
    def goal_response_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future: Future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: FollowStepperTrajectory.Result = future.result().result
        if result.error_code == 0:
            self.joint_states_prev = self.joint_states_now
            #self.save_joint_states() 
        self.current_goal_handle = None
        self.get_logger().info(f'Result: {result}')

    def feedback_callback(self, feedback_msg):
        client_feedback : FollowStepperTrajectory.Feedback = feedback_msg.feedback
        server_feedback = FollowJointTrajectory.Feedback()
        steps = array('q', [p//(1000*1000) for p in client_feedback.current_positions])
        joints = cvt.convert_steps_to_joint(steps, self.tf)
        self.joint_states_now = array('d', [i+j for i,j in zip(self.joint_states_prev, joints)])
        # Calculate feedback
        server_feedback.actual.positions = self.joint_states_now
        server_feedback.desired.positions = self.joint_states_now #TODO: use trajectory values calculated to current time
        server_feedback.error.positions = array('d', [0]*len(self.joint_states_now))
        self.current_goal_handle.publish_feedback(server_feedback)
        
def main(args=None):
    rclpy.init(args=args)
    mtx = rclpy.executors.MultiThreadedExecutor(4)
    node = StepperTrajectoryProcessor()
    rclpy.spin(node, executor=mtx)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
