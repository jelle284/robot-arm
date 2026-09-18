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
    docker.io/espressif/idf:release-v6.1
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
podman start robot-build-env &&
podman attach robot-build-env
```
### Inside the container, you are ready to build or flash:
```bash
idf.py build
idf.py flash monitor
```

---

## 2. Run the web hmi

### Build the container
```bash
podman build -f Dockerfile -t robot-web-hmi:latest
```

### Run the container
```bash
podman run -d \
    --name robot-hmi \
    --restart unless-stopped \
    --net=host \
    robot-web-hmi:latest
```

The interface will now be accessible at `http://<your-raspberry-pi-ip>:8000`.

## 3. Run ros on the desktop
```bash
source ./ros_setup.sh
```
```bash
ros2 launch robot_stepper_controller stepper_control.launch.py
```
```bash
ros2 launch robot_stepper_controller planning.launch.py
```