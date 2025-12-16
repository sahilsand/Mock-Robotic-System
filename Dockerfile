FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y python3-colcon-common-extensions python3-rosdep python3-numpy dos2unix && rm -rf /var/lib/apt/lists/*
WORKDIR /ros_ws
RUN mkdir -p src
COPY src/ ./src/
RUN find src -type f -name "*.py" -exec dos2unix {} +
RUN find src -type f -name "*.xml" -exec dos2unix {} +
RUN find src -type f -name "setup.py" -exec dos2unix {} +
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN colcon build --event-handlers console_cohesion+
ENV ROS_DOMAIN_ID=0
ENV PYTHONUNBUFFERED=1
COPY entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
