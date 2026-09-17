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
```bash
podman run -d \
    --name microros-agent \
    --restart unless-stopped \
    --net=host \
    -v ./fastdds_config/fastdds_profile_pi.xml:/tmp/fastdds_profile.xml:z \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml \
    docker.io/microros/micro-ros-agent:jazzy \
    udp4 --port 8888 -l 1000 -a 300
```

The `-l 1000` and `-a 300` flags enable liveliness checks to automatically prune stale nodes (e.g., after power cycling the ESP32).

### Optional: Enter ros2 env on pi
```bash
podman run -it --rm \
    --net=host \
    -v $(pwd):/workspace:z \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/fastdds_config/fastdds_profile_pi.xml \
    docker.io/ros:jazzy-ros-base
```

---

## 3. Run the Web HMI Container on Raspberry Pi
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
