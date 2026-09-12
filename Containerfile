FROM docker.io/espressif/idf:release-v6.1

# Installer ROS2 Jazzy afhængigheder og Colcon build-værktøjer via pip
RUN pip install --no-cache-dir \
    catkin_pkg \
    colcon-common-extensions \
    lark

# Sæt arbejdskataloget til hele projektet
WORKDIR /workspace
