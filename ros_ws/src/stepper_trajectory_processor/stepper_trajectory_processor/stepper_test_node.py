import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from stepper_msgs.msg import StepperTrajectory, StepperTrajectoryPoint
import time

class StepperTest(Node):
    def __init__(self):
        super().__init__("StepperTest")

        # Subscriber for stepper commands
        self._stepper_cmd_subscriber = self.create_subscription(
            StepperTrajectory,
            '/stepper_cmd',
            self.stepper_cmd_callback,
            10
        )

        # Publisher for stepper status (acknowledgment)
        self._stepper_sts_publisher = self.create_publisher(
            String,
            '/stepper_sts',
            10
        )

        # Publisher for stepper step count
        self._stepper_cnt_publisher = self.create_publisher(
            Int32MultiArray,
            '/stepper_cnt',
            10
        )

    def stepper_cmd_callback(self, msg: StepperTrajectory):
        """Handle incoming stepper commands and simulate execution."""
        self.get_logger().info("Received stepper command")

        # Publish acknowledgment
        ack_msg = String()
        ack_msg.data = "ACK"
        self._stepper_sts_publisher.publish(ack_msg)
        self.get_logger().info("Published acknowledgment")

        # Publish step counts
        for point in msg.points:
            point: StepperTrajectoryPoint
            step_cnt = Int32MultiArray()
            step_cnt.data = point.steps
            self._stepper_cnt_publisher.publish(step_cnt)
            self.get_logger().info(f"Published step count: {step_cnt.data}")

            # Simulate delay
            time.sleep(point.duration_us / 1e6)

        # Publish Done
        ack_msg = String()
        ack_msg.data = "DN"
        self._stepper_sts_publisher.publish(ack_msg)
        self.get_logger().info("Published execution done")


def main(args=None):
    rclpy.init(args=args)
    node = StepperTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
