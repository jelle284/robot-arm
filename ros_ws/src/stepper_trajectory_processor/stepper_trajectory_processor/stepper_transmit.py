import rclpy
from rclpy.node import Node
from stepper_msgs.msg import StepperTrajectoryPoint

class MinimalPublisher(Node):
    def __init__(self, publish_count):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(StepperTrajectoryPoint, '/cmd_point', 10)
        self.publish_count = publish_count
        self.current_count = 0
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.current_count < self.publish_count:
            for i in range(4):
                msg = StepperTrajectoryPoint()
                msg.steps = [20, 0, 0, 0, 0, 0]  # Sequence length of six
                msg.duration_us = 100*1000  # Example duration
                self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: 4x "{msg.steps}, {msg.duration_us}" ({self.current_count + 1}/{self.publish_count})')
            self.current_count += 1
        else:
            self.get_logger().info('Finished publishing. Exiting...')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    publish_count = 3  # Set the number of times to publish
    minimal_publisher = MinimalPublisher(publish_count)
    rclpy.spin(minimal_publisher)
    

if __name__ == '__main__':
    main()
