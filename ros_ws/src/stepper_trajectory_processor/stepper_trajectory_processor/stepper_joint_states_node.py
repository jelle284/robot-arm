import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from stepper_msgs.msg import StepperTrajectory, StepperTrajectoryPoint
from sensor_msgs.msg import JointState
import json
import os
import stepper_joint_conversion

class StepperJointStates(Node):
    def __init__(self):
        super().__init__("StepperJointStates")

        # Subscriber for stepper count
        self._stepper_cnt_subscriber = self.create_subscription(
            Int32MultiArray,
            '/stepper_cnt',
            self.stepper_cnt_callback,
            10
        )

        # Publisher for joint states
        self._joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10  # QoS depth
        )

        # Load motor configuration file
        self.motor_config = stepper_joint_conversion.load_config("motors.yaml")

        # Load initial joint states from file
        self.joint_state_file = "joint_states.json"
        self.current_joint_states = self.load_joint_states()
        # Periodic publishing of joint states
        self.timer = self.create_timer(1.0, self.publish_joint_states)  # Publish every 1 second

    def stepper_cnt_callback(self, msg: Int32MultiArray):
        joint_delta = stepper_joint_conversion.convert_steps_to_joint(msg, self.motor_config)
        for i in range(min(
            [len(joint_delta),len(self.current_joint_states)]
            )):
            self.current_joint_states[i] += joint_delta[i]

    def load_joint_states(self):
        """Load joint states from a JSON file."""
        if os.path.exists(self.joint_state_file):
            with open(self.joint_state_file, "r") as file:
                data = json.load(file)
                self.get_logger().info(f"Loaded joint states: {data}")
                return data
        else:
            self.get_logger().warn(f"File {self.joint_state_file} not found. Initializing default joint states.")
            return { "positions": [0.0] * 6 }  # Default to 6 joints at position 0

    def save_joint_states(self):
        """Save the current joint states to a JSON file."""
        with open(self.joint_state_file, "w") as file:
            json.dump(self.current_joint_states, file)
        self.get_logger().info(f"Saved joint states: {self.current_joint_states}")

    def publish_joint_states(self):
        """Publish the current joint states periodically."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"j{i}" for i in range(len(self.current_joint_states["positions"]))]
        joint_state_msg.position = self.current_joint_states["positions"]
        self._joint_state_publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StepperJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
