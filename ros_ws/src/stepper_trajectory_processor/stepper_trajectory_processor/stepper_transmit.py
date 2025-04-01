import rclpy
import rclpy.executors
from rclpy.node import Node
from stepper_msgs.msg import StepperPoint, StepperFeedback, StepperTrajectory

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

AXIS_NUM = 6
MAX_PARTIAL_SIZE = 32

class MinimalPublisher(Node):
    def __init__(self, publish_count):
        super().__init__('minimal_publisher')

        self.publisher = self.create_publisher(
            StepperTrajectory,
            '/stepper_cmd', 
            QOS)
        self.finished = False
        self.create_trajectory()
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_trajectory(self):
        points = []
        seq = [100]*45+[-200]*6
        for s in seq:
            point = StepperPoint()
            point.steps = [0] * AXIS_NUM
            point.steps[0] = s
            point.duration_ms = 100
            points.append(point)
        self.pub_list = self.process_points(points)

    def timer_callback(self):
        try:
            msg = self.pub_list.pop(0)
            self.publisher.publish(msg)
        except IndexError:
            self.finished = True
            self.get_logger().info(f'Finished publishing. Exiting...')
    
    def process_points(self, points: list):
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
            msg.transmission_id = 2
            msg.partial_total=num_tx
            msg.partial_count=n+1
            msg_list.append(msg)
            first = last
        return msg_list

def main(args=None):
    rclpy.init(args=args)
    publish_count = 10  # Set the number of times to publish
    minimal_publisher = MinimalPublisher(publish_count)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    while not minimal_publisher.finished:
        rclpy.spin_once(minimal_publisher, executor=executor, timeout_sec=0.1)
    executor.shutdown()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
