import rclpy
from rclpy.node import Node
from stepper_msgs.msg import StepperCommand, StepperState
from rclpy.qos import qos_profile_sensor_data
import time
AXIS_COUNT = 6
MOVE_VELOCITY = 1000 # Hz
MOVE_DURATION = 2 # seconds
class TestFirmware(Node):
    def __init__(self):
        super().__init__('test_firmware_node')
        self.get_logger().info('Test Firmware Node has been started.')
        self.command_publisher = self.create_publisher(StepperCommand, 'stepper_command', qos_profile_sensor_data)
        self.state_subscriber = self.create_subscription(StepperState, 'stepper_state', self.status_callback, qos_profile_sensor_data)
        self.state = StepperState()
        self.completed = False
    def status_callback(self, msg):
        self.state = msg
    def run_test_sequence(self):
        self.get_logger().info('Running test sequence...')
        for i in range(AXIS_COUNT):
            cmd = StepperCommand()
            cmd.velocity = [0] * AXIS_COUNT
            cmd.velocity[i] = MOVE_VELOCITY
            self.command_publisher.publish(cmd)
            self.get_logger().info(f'Sent command to stepper {i+1}')
            time.sleep(MOVE_DURATION)
        cmd = StepperCommand()
        cmd.velocity = [0] * AXIS_COUNT
        self.command_publisher.publish(cmd)
        self.get_logger().info('Test sequence completed.')
        for i in range(AXIS_COUNT):
            try:
                self.get_logger().info(f'Stepper {i+1} position: {self.state.position[i]}')
            except IndexError:
                self.get_logger().error(f'Stepper {i+1} position data not available.')
        self.completed = True

if __name__ == '__main__':
    rclpy.init()
    node = TestFirmware()
    node.run_test_sequence()
    while rclpy.ok() and not node.completed:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()