import rclpy
from rclpy.node import Node
from stepper_msgs.msg import StepperPoint
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self, publish_count):
        super().__init__('minimal_publisher')
        self.point_publisher_ = self.create_publisher(StepperPoint, '/stepper_point', 10)
        self.publish_count = publish_count
        self.current_count = 0
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.current_count < self.publish_count:
            # Publish StepperTrajectoryPoint messages
            seq = [4, 8, 2]
            for s in seq:
                msg = StepperPoint()
                msg.command = StepperPoint.COMMAND_LOAD
                msg.steps = [s, 0, 0, 0, 0, 0]  # Sequence length of six
                msg.duration_us = 8 * 1000  # Example duration
                self.point_publisher_.publish(msg)
                self.get_logger().info(f'Publishing point: "{msg.steps}, {msg.duration_us}" ({self.current_count + 1}/{self.publish_count})')

            # Publish execution command
            
            msg = StepperPoint()
            msg.steps = [0] * 6
            msg.duration_us = 0
            msg.command = StepperPoint.COMMAND_EXECUTE
            self.point_publisher_.publish(msg)
            self.get_logger().info(f'Publishing execute')
            self.current_count += 1
        else:
            self.get_logger().info('Finished publishing. Exiting...')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    publish_count = 1  # Set the number of times to publish
    minimal_publisher = MinimalPublisher(publish_count)
    rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()
