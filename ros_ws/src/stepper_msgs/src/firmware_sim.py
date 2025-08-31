import rclpy
from rclpy.node import Node
from stepper_msgs.msg import StepperCommand, StepperState
from rclpy.qos import qos_profile_sensor_data

AXIS_COUNT = 6
UPDATE_RATE = 0.01
MAX_VEL = 5000

class TestFirmware(Node):
    def __init__(self):
        super().__init__('firmware_simulator_node')
        self.get_logger().info('Test Firmware Node has been started.')
        self.state_publisher = self.create_publisher(StepperState, 'stepper_state', qos_profile_sensor_data)
        self.command_subscriber = self.create_subscription(StepperCommand, 'stepper_command', self.status_callback, qos_profile_sensor_data)
        self.cmd = StepperCommand()
        self.cmd.position = [0]*AXIS_COUNT
        self.state = StepperState()
        self.pos_float = [0.0]*AXIS_COUNT
        self.state.position = [0]*AXIS_COUNT
        self.state.velocity = [0]*AXIS_COUNT
        self.completed = False
        self.create_timer(UPDATE_RATE, self.timer_callback, autostart=True)

    def status_callback(self, msg):
        self.cmd = msg

    def inital_position_callback(self, msg: StepperCommand):
        self.initial_position = msg

    def timer_callback(self):
        Kp = 1.0
        for i in range(AXIS_COUNT):
            error = (self.cmd.position[i] - self.state.position[i])
            vel = Kp * error
            vel_clamped = vel
            if vel > MAX_VEL: vel_clamped = MAX_VEL
            if vel < -MAX_VEL: vel_clamped = -MAX_VEL
            self.pos_float[i] += vel_clamped * UPDATE_RATE
            self.state.velocity[i] = int(vel_clamped)
            self.state.position[i] = int(self.pos_float[i])
        self.state_publisher.publish(self.state)

if __name__ == '__main__':
    rclpy.init()
    node = TestFirmware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()