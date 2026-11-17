FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-tf2-ros ros-jazzy-rmw-cyclonedds-cpp python3-venv python3-dev build-essential \
    libgl1-mesa-dri libegl1 mesa-utils ca-certificates cmake && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/robot-validation
COPY . .
RUN source /opt/ros/jazzy/setup.bash && \
    cmake -S deploy/contact_monitor -B build/contact_monitor && \
    cmake --build build/contact_monitor -j2
RUN python3 -m venv --system-site-packages /opt/validation-venv && \
    /opt/validation-venv/bin/pip install --no-cache-dir 'setuptools>=68' wheel && \
    /opt/validation-venv/bin/pip install --no-cache-dir --no-build-isolation ./third_party/breezyslam/python && \
    /opt/validation-venv/bin/pip install --no-cache-dir -e . && \
    /opt/validation-venv/bin/pip freeze > /opt/robot-validation/deploy/python-resolved.txt && \
    dpkg-query -W > /opt/robot-validation/deploy/apt-resolved.txt
ENV PATH="/opt/validation-venv/bin:${PATH}" \
    LIBGL_ALWAYS_SOFTWARE=1 \
    PYTHONUNBUFFERED=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROBOT_VALIDATE_ROOT=/opt/robot-validation \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/robot-validation/build/contact_monitor
ENTRYPOINT ["/opt/robot-validation/deploy/entrypoint.sh"]
CMD ["sim", "--scenario", "demo", "--output", "/results/demo"]
