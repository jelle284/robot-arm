FROM ros:jazzy-ros-base

# Install build tools, micro-ROS agent, and Python dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-micro-ros-agent \
    && rm -rf /var/lib/apt/lists/*

# Install FastAPI and dependencies for HMI
RUN pip3 install --no-cache-dir --break-system-packages \
    fastapi \
    uvicorn \
    jinja2 \
    pydantic

# Copy FastDDS config (if still needed for other QoS settings)
COPY ./fastdds_profile.xml /fastdds_profile.xml

# Set up ROS2 workspace and build stepper_msgs
WORKDIR /ros_ws
COPY ./ros_ws/src/stepper_msgs /ros_ws/src/stepper_msgs
RUN git clone https://github.com/micro-ROS/micro_ros_agent.git -b jazzy
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build stepper_msgs

# Copy HMI code
WORKDIR /app
COPY ./web_hmi/main.py /app/main.py
COPY ./web_hmi/templates /app/templates

# Copy entrypoint script
COPY ./entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# Expose ports for agent (8888) and HMI (8000)
EXPOSE 8000 8888

# Set entrypoint
ENTRYPOINT ["/app/entrypoint.sh"]