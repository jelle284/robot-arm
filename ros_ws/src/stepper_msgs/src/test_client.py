import rclpy
from rclpy.action import ActionClient

from rclpy.node import Node
from stepper_msgs.action import FollowStepperTrajectory
from stepper_msgs.msg import StepperTrajectoryPoint

import matplotlib.pyplot as plt

class StepperActionClient(Node):

    def __init__(self):
        super().__init__('stepper_action_client')
        self._action_client = ActionClient(self, FollowStepperTrajectory, '/follow_stepper_trajectory')
        self.X = [0]
        self.T = [0]
    def send_goal(self):
        goal_msg = FollowStepperTrajectory.Goal()
        accel = StepperTrajectoryPoint()
        accel.accelerations = [4000]
        accel.velocities = [0]
        accel.positions = [0]
        accel.time_from_start = 0
        cruise = StepperTrajectoryPoint()
        cruise.accelerations = [0]
        cruise.velocities = [4000*1000]
        cruise.positions = [2000*1000*1000]
        cruise.time_from_start = 1*1000*1000
        decel = StepperTrajectoryPoint()
        decel.accelerations = [-4000]
        decel.velocities = [4000*1000]
        decel.positions = [6000*1000*1000]
        decel.time_from_start = 2*1000*1000
        ending = StepperTrajectoryPoint()
        ending.accelerations = [0]
        ending.velocities = [0]
        ending.positions = [8000*1000*1000]
        ending.time_from_start = 3*1000*1000

        goal_msg.trajectory.points = [accel, cruise, decel, ending]

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback : FollowStepperTrajectory.Feedback = feedback_msg.feedback
        self.X.append(feedback.current_positions[0])
        self.T.append(feedback.time_elapsed)
        

def main(args=None):
    rclpy.init(args=args)

    action_client = StepperActionClient()

    action_client.send_goal()
    rclpy.spin(action_client)

    fig, ax = plt.subplots()
    ax.plot(action_client.T, action_client.X)
    ax.grid()
    ax.set_title("Client feedback")
    plt.show()
if __name__ == '__main__':
    main()
