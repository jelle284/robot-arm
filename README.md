# Robot Arm Project - Build & Run Guide

This project contains both the ESP-IDF firmware and the ROS2 Jazzy workspace. To ensure a consistent build environment, Podman is used.

## 1. Build the Podman Image
First, build the custom container image containing ESP-IDF v6.1 and the required ROS2/Colcon build tools. Run this command from the **root directory** of the project (`robot-arm/`):

```bash
podman build -t robot-arm-idf:latest .
```

## 2. Compile the Firmware (ESP-IDF)
To compile the firmware, you must start the container from the **root directory**. This ensures that the relative symlink inside `robot-controller-firmware/extra_ros_packages` pointing to `ros_ws/src/stepper_msgs` resolves correctly inside the container.

### Start the container interactively:
```bash
podman run -it --rm \
    --device=/dev/ttyUSB0 \
    -v "\$(pwd)":/workspace \
    -w /workspace/robot-controller-firmware \
    robot-arm-idf:latest
```

*Note: We mount the entire project root (`-v "$(pwd)":/workspace`), but set the working directory directly to the firmware folder (`-w /workspace/robot-controller-firmware`).*

### Inside the container, run:
```bash
idf.py build
```
*(Optional) To flash and monitor your ESP32 directly from the container:*
```bash
idf.py flash monitor
```

## 3. Run the micro-ROS Agent on a Raspberry Pi Server (UDP)
To run the micro-ROS agent as a permanent background service on your Raspberry Pi, use the `-d` (detached) flag and set a restart policy. 

### Option A: Use Host Networking (Recommended)
This allows the agent to use the Raspberry Pi's network interface directly, ensuring the ESP32 can easily find it over UDP:
```bash
podman run -d \
    --name microros-agent \
    --restart unless-stopped \
    --net=host \
    docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888
```

### Option B: Isolated Port Forwarding
If you prefer to isolate the container and only expose the specific UDP port:
```bash
podman run -d \
    --name microros-agent \
    --restart unless-stopped \
    -p 8888:8888/udp \
    docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888
```

### Useful Management Commands for the Server:
* **View logs (check connection):** `podman logs -f microros-agent`
* **Stop the agent:** `podman stop microros-agent`
* **Start it again:** `podman start microros-agent`

## 4. Run the Web HMI Container on Raspberry Pi
To run the Web HMI in the background so it can communicate with the micro-ROS agent over the ROS2 network:

### Build the HMI Image:
Run from the project **root directory**:
```bash
podman build -f Dockerfile.hmi -t robot-web-hmi:latest .
```

### Run the Container (Detached & Shared Network):
*Crucial: We use `--net=host` so this container shares the ROS2 middleware domain seamlessly with your micro-ROS agent container.*

```bash
podman run -d \
    --name robot-hmi \
    --restart unless-stopped \
    --net=host \
    robot-web-hmi:latest
```

The interface will now be accessible at `http://<your-raspberry-pi-ip>:8000`.
