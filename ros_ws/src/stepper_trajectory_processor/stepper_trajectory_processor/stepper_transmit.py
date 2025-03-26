import rclpy
import rclpy.executors
from rclpy.node import Node
from stepper_msgs.msg import StepperPoint, StepperFeedback, StepperTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

AXIS_NUM = 6

class MinimalPublisher(Node):
    def __init__(self, publish_count):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.point_publisher_ = self.create_publisher(StepperTrajectory, '/stepper_point', qos_profile)
        self.feedback_subscriber = self.create_subscription(StepperFeedback, '/stepper_fb', self.sub_callback, qos_profile)
        self.fb_count = [0] * AXIS_NUM
        self.cmd_count = [0] * AXIS_NUM
        self.wait_for_feedback = False
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def sub_callback(self, msg: StepperFeedback):
        self.get_logger().info(f'Feedback: {msg}')
        
    def timer_callback(self):
        points = []
        seq = [200]*20
        for s in seq:
            point = StepperPoint()
            point.steps = [0] * AXIS_NUM
            point.steps[0] = s
            point.duration_ms = 100
            points.append(point)

        msg = StepperTrajectory()
        msg.motor_names = [f"motor{n+1}" for n in range(AXIS_NUM)]
        msg.points = points
        self.point_publisher_.publish(msg)

        self.get_logger().info(f'Finished publishing. Sent {self.cmd_count}, got {self.fb_count}. Exiting...')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    publish_count = 10  # Set the number of times to publish
    minimal_publisher = MinimalPublisher(publish_count)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    rclpy.spin(minimal_publisher, executor=executor)

if __name__ == '__main__':
    main()
