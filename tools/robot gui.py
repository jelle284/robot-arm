import tkinter as tk
import time
import paho.mqtt.client as mqtt
import json

# MQTT broker information
MQTT_BROKER = "192.168.0.10"
MQTT_PORT = 1883
MQTT_TOPIC = "/robot/position"

# Set up MQTT client
client = mqtt.Client()

# Connect to the MQTT broker
def connect_mqtt():
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()  # Start the MQTT client loop in the background

# Function to send a movement command via MQTT
def send_move(axis, angle, duration, movement_type):
    move_command = {
        "axis_angles": [
            {"axis": axis, "angle": angle}
        ],
        "type": movement_type,
        "duration": duration
    }
    # Convert dictionary to JSON string
    move_json = json.dumps(move_command)
    print(f"Sending move: {move_json}")
    
    # Publish the move to the MQTT topic
    client.publish(MQTT_TOPIC, move_json)

# Placeholder function to calculate the current end-effector position based on joint angles
def calculate_end_effector_position(joint_angles):
    # Example implementation: replace with actual forward kinematics logic
    return [sum(joint_angles), sum(joint_angles) / 2, sum(joint_angles) / 3]

# Placeholder function to calculate the required joint angles to reach a desired position
def calculate_joint_angles(desired_position):
    # Example implementation: replace with actual inverse kinematics logic
    return [desired_position[0] / 6, desired_position[1] / 6, desired_position[2] / 6, 0, 0, 0]

# GUI window for controlling the robot
class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Axis Control")

        # Move settings
        self.duration = tk.DoubleVar(value=4.0)
        self.move_amounts = [tk.DoubleVar(value=0) for _ in range(6)]
        self.current_angles = [tk.DoubleVar(value=0) for _ in range(6)]
        self.desired_position = [tk.DoubleVar(value=0) for _ in range(3)]
        self.calculated_effector_position = [tk.StringVar(value="0") for _ in range(3)]
        self.movement_type = "delta"

        # Update effector position whenever current angles change
        for angle_var in self.current_angles:
            angle_var.trace("w", self.update_effector_position)

        # Axis controls
        axis_frame = tk.Frame(root)
        axis_frame.pack(pady=10)

        tk.Label(axis_frame, text="Axis").grid(row=0, column=0)
        tk.Label(axis_frame, text="Move Amount").grid(row=0, column=1)

        for i in range(6):
            tk.Label(axis_frame, text=f"Axis {i}").grid(row=i+1, column=0)
            tk.Entry(axis_frame, textvariable=self.move_amounts[i], width=10).grid(row=i+1, column=1)

        # Duration input
        duration_frame = tk.Frame(root)
        duration_frame.pack(pady=10)

        tk.Label(duration_frame, text="Move Time (seconds):").pack(side=tk.LEFT)
        tk.Entry(duration_frame, textvariable=self.duration, width=10).pack(side=tk.LEFT)

        # Current joint angles
        angles_frame = tk.Frame(root)
        angles_frame.pack(pady=10)

        tk.Label(angles_frame, text="Current Angles").grid(row=0, column=0)
        for i in range(6):
            tk.Entry(angles_frame, textvariable=self.current_angles[i], width=10).grid(row=i+1, column=0)

        # Desired end-effector position
        position_frame = tk.Frame(root)
        position_frame.pack(pady=10)

        tk.Label(position_frame, text="Desired End-Effector Position").grid(row=0, column=0)
        for i, label in enumerate(["X", "Y", "Z"]):
            tk.Label(position_frame, text=label).grid(row=0, column=i+1)
            tk.Entry(position_frame, textvariable=self.desired_position[i], width=10).grid(row=1, column=i+1)

        # Current calculated end-effector position
        calculated_frame = tk.Frame(root)
        calculated_frame.pack(pady=10)

        tk.Label(calculated_frame, text="Current Effector Position").grid(row=0, column=0)
        for i, label in enumerate(["X", "Y", "Z"]):
            tk.Label(calculated_frame, text=label).grid(row=0, column=i+1)
            tk.Label(calculated_frame, textvariable=self.calculated_effector_position[i]).grid(row=1, column=i+1)

        # Control buttons
        button_frame = tk.Frame(root)
        button_frame.pack(pady=10)

        tk.Button(button_frame, text="Calculate Move", command=self.calculate_move).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Move All", command=self.move_all).pack(side=tk.LEFT, padx=5)

    # Automatically update current end-effector position
    def update_effector_position(self, *args):
        try:
            joint_angles = []
            for angle in self.current_angles:
                try:
                    joint_angles.append(angle.get())
                except tk.TclError:  # Handle invalid or empty inputs
                    joint_angles.append(0.0)
            position = calculate_end_effector_position(joint_angles)
            for i in range(3):
                self.calculated_effector_position[i].set(f"{position[i]:.2f}")
        except Exception as e:
            # Handle unexpected errors gracefully
            for i in range(3):
                self.calculated_effector_position[i].set("Error")
            print(f"Error in update_effector_position: {e}")


    # Function to calculate move amounts
    def calculate_move(self):
        joint_angles = [angle.get() for angle in self.current_angles]
        desired_position = [pos.get() for pos in self.desired_position]
        required_angles = calculate_joint_angles(desired_position)
        for i in range(6):
            self.move_amounts[i].set(required_angles[i] - joint_angles[i])

    # Function to move all axes based on user input and reset the amounts
    def move_all(self):
        for i in range(6):
            amount = self.move_amounts[i].get()
            if amount != 0:  # Skip axes with no movement specified
                send_move(i, amount, self.duration.get(), self.movement_type)
            self.move_amounts[i].set(0)  # Reset the move amount to zero


# Main function to start the GUI and MQTT
def main():
    connect_mqtt()

    # Create the tkinter window
    root = tk.Tk()
    app = RobotControlGUI(root)

    # Start the GUI event loop
    root.mainloop()

    # Stop the MQTT client loop when GUI is closed
    client.loop_stop()

if __name__ == "__main__":
    main()
