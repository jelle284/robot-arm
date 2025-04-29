import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from stepper_msgs.action import FollowStepperTrajectory
from stepper_msgs.msg import StepperTrajectoryPoint

import matplotlib.pyplot as plt

class StepperActionServer(Node):

    def __init__(self):
        super().__init__('stepper_action_server')
        self._action_server = ActionServer(
            self,
            FollowStepperTrajectory,
            '/follow_stepper_trajectory',
            self.execute_callback)
        
    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing goal...')
        VMIN = 1000*1000 # 1000 usteps pr msec
        goal_request: FollowStepperTrajectory.Goal = goal_handle.request
        points = goal_request.trajectory.points
        N = len(points)
        X = []
        T = []
        for i in range(N-1):
            current_point: StepperTrajectoryPoint = points[i]
            next_point: StepperTrajectoryPoint = points[i+1]
            a0 = current_point.accelerations[0]
            v0 = current_point.velocities[0]
            x0 = current_point.positions[0]
            ts = next_point.time_from_start - current_point.time_from_start
            t = 0
            x = x0
            v = v0
            while t < ts:
                if v > VMIN or v < -VMIN:
                    dt = (1000*1000*1000)//abs(v) # microseconds
                else:
                    dt = (1000*1000*1000)//VMIN # microseconds
                t += dt
                v = (a0*t)//1000 + v0
                x = (a0*t*t)//(2*1000*1000) + (v0*t)//1000 + x0
                X.append(x)
                T.append(t + current_point.time_from_start)
            # publish feedback
            feedback_msg = FollowStepperTrajectory.Feedback()
            feedback_msg.current_positions = [x]
            feedback_msg.time_elapsed = t + current_point.time_from_start
            goal_handle.publish_feedback(feedback_msg)
        
        # plot the steps
        fig, ax = plt.subplots()
        ax.plot(T, X)
        ax.grid()
        ax.set_title("Server steps")
        plt.show()

        # Submit result
        result = FollowStepperTrajectory.Result()
        result.error_code = 0

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    stepper_action_server = StepperActionServer()
    while 1:
        rclpy.spin_once(stepper_action_server)

if __name__ == '__main__':
    main()
