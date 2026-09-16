# Robot Arm Project - Build & Run Guide

This project contains both the ESP-IDF firmware and the ROS2 Jazzy workspace. To ensure a consistent build environment, Podman is used.

## 1. Compile and Flash the Firmware (ESP-IDF)

To avoid installing dependencies every time, we create a persistent container named `robot-build-env`. It will remember your `pip` installations between sessions.

### First-Time Setup (Create and Configure Container)
Run this command from the **root directory** (`robot-arm/`) to create the container, install the ROS2 build tools, and enter the workspace:

```bash
podman run -it \
    --name robot-build-env \
    --device=/dev/ttyUSB0 \
    --group-add keep-groups \
    -v "$(pwd)":/workspace \
    -w /workspace/robot-controller-firmware \
    docker.io/espressif/idf:release-v6.1 \
    bash -c "pip install catkin_pkg colcon-common-extensions lark && bash"
```

Once inside, you can build right away:
```bash
idf.py build
```
To exit the container when you are done, simply type `exit`.

---

### Next-Time Use (Resume Your Work)
When you return to work later, the container still exists with all packages installed. You do not need to run the long `podman run` command again. 

Just **start** it and **attach** to it from your terminal:

```bash
# 1. Start the existing container in the background
podman start robot-build-env

# 2. Enter the active container terminal
podman attach robot-build-env
```

### Inside the container, you are ready to build or flash:
```bash
idf.py build
idf.py flash monitor
```

---

## 2. Run the micro-ROS Agent on a Raspberry Pi Server (UDP)
To run the micro-ROS agent as a permanent background service on your Raspberry Pi, use the `-d` (detached) flag and set a restart policy.

### Option A: Use Host Networking (Recommended)
This allows the agent to use the Raspberry Pi's network interface directly, ensuring the ESP32 can easily find it over UDP:
```bash
podman run -d \
    --name microros-agent \
    --restart unless-stopped \
    --net=host \
    docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888 -l 1000 -a 300
```

The `-l 1000` and `-a 300` flags enable liveliness checks to automatically prune stale nodes (e.g., after power cycling the ESP32).

### Option B: Run ROS2 Discovery Server (For Multi-Machine Setup)
If you need your PC to discover ROS2 topics from the Pi (e.g., to see `/stepper_state` from your ESP32), run a discovery server in a separate container:
```bash
podman run -d \
    --name ros2-discovery-server \
    --restart unless-stopped \
    --net=host \
    docker.io/ros/ros2:jazzy-ros-core \
    ros2 daemon start --discovery-server-port 11811
```

Then on **all machines** (Pi and PC), set this environment variable:
```bash
export ROS_DISCOVERY_SERVER=192.168.0.10:11811
```

### Option C: Isolated Port Forwarding
If you prefer to isolate the container and only expose the specific UDP port:
```bash
podman run -d \
    --name microros-agent \
    --restart unless-stopped \
    -p 8888:8888/udp \
    docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888 -l 1000 -a 300
```

### Useful Management Commands for the Server:
* **View logs (check connection):** `podman logs -f microros-agent`
* **Stop the agent:** `podman stop microros-agent`
* **Start it again:** `podman start microros-agent`

---

## 3. Run the ROS2 Discovery Server on Raspberry Pi

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

### Run the Container (Locally):

```bash
podman run -it --rm \
    --name robot-hmi \
    --net=host \
    robot-web-hmi:latest
```

The interface will now be accessible at `http://<your-raspberry-pi-ip>:8000`.
