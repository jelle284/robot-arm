import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from stepper_msgs.action import FollowStepperTrajectory
from stepper_msgs.msg import StepperTrajectoryPoint

import matplotlib.pyplot as plt

class StepperPlotter:
    def __init__(self):
        self.Z = [0]
        self.z = 0
        self.T = [0]
        self.t = 0
    def step(self, direction, duration, target):
        z = self.z + direction
        t = self.t + duration
        self.Z.append(z)
        self.T.append(t)
        self.z = z
        self.t = t
    def plot(self, ax):
        ax.plot(self.T, self.Z)
        ax.grid()

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
        fig, ax = plt.subplots()
        for i in range(N-1):
            current_point: StepperTrajectoryPoint = points[i]
            next_point: StepperTrajectoryPoint = points[i+1]
            ts = next_point.time_from_start - current_point.time_from_start
            colors = ['b', 'g', 'r', 'c', 'm', 'y']
            fb = []
            for a0, v0, x0 in zip(
                                current_point.accelerations,
                                current_point.velocities,
                                current_point.positions):
                t = 0
                X = [x0]
                T = [current_point.time_from_start]
                while t < ts:
                    v = (a0*t)//1000 + v0
                    x = (a0*t*t)//(2*1000*1000) + (v0*t)//1000 + x0
                    if v > VMIN or v < -VMIN:
                        dt = (1000*1000*1000)//abs(v) # microseconds
                    else:
                        dt = (1000*1000*1000)//VMIN # microseconds
                    if (t+dt) > ts:
                        dt -= (t-ts)
                    t += dt
                    X.append(x)
                    T.append(t + current_point.time_from_start)
                fb.append(x)
                color = colors.pop(0)
                ax.plot(T, X, color)
            # publish feedback
            feedback_msg = FollowStepperTrajectory.Feedback()
            feedback_msg.current_positions = fb
            feedback_msg.time_elapsed = t + current_point.time_from_start
            goal_handle.publish_feedback(feedback_msg)
        
        # plot the steps
        ax.grid()
        ax.set_title("Server steps")
        plt.show()

        # Submit result
        result = FollowStepperTrajectory.Result()
        result.error_code = 0

        goal_handle.succeed()
        return result

def main(args=None):
    plt.ioff()
    rclpy.init(args=args)
    stepper_action_server = StepperActionServer()
    rclpy.spin(stepper_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
