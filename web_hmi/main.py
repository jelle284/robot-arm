import asyncio
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel
from typing import List

# Import af ROS2 biblioteker
import rclpy
from rclpy.node import Node
from stepper_msgs.msg import StepperCommand, StepperState

app = FastAPI(title="Robot Arm Web HMI")
templates = Jinja2Templates(directory="templates")

# Globale variabler til at holde styr på tilstande og noden
latest_state = {"velocity": [0]*6, "position": [0]*6}
ros_node = None

class PositionCommand(BaseModel):
    positions: List[int]

class ROS2Interface(Node):
    def __init__(self):
        super().__init__('web_hmi_node')
        self.publisher_ = self.create_publisher(StepperCommand, 'stepper_command', 10)
        self.subscription = self.create_subscription(
            StepperState,
            'stepper_state',
            self.state_callback,
            10
        )
        self.get_logger().info("ROS2 Web HMI Node started.")

    def state_callback(self, msg: StepperState):
        global latest_state
        latest_state["velocity"] = list(msg.velocity)
        latest_state["position"] = list(msg.position)

    def send_positions(self, positions: List[int]):
        msg = StepperCommand()
        msg.position = positions
        self.publisher_.publish(msg)

async def ros2_spin_loop():
    global ros_node
    while rclpy.ok():
        rclpy.spin_once(ros_node, timeout_sec=0.05)
        await asyncio.sleep(0.01)

@app.on_event("startup")
async def startup_event():
    global ros_node
    rclpy.init()
    ros_node = ROS2Interface()
    asyncio.create_task(ros2_spin_loop())

@app.on_event("shutdown")
def shutdown_event():
    global ros_node
    if ros_node:
        ros_node.destroy_node()
    rclpy.shutdown()

@app.get("/", response_class=HTMLResponse)
async def get_hmi(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api/state")
async def get_state():
    return latest_state

@app.post("/api/command")
async def send_command(command: PositionCommand):
    global ros_node
    if ros_node and len(command.positions) == 6:
        ros_node.send_positions(command.positions)
        return {"status": "success"}
    return {"status": "error", "message": "Invalid node or payload"}
