FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-slam-toolbox \
        ros-jazzy-teleop-twist-keyboard \
        python3-pip python3-venv git cmake sudo \
    && rm -rf /var/lib/apt/lists/*

# Use the image's default user (uid 1000) with access to USB serial ports for flashing.
RUN usermod -aG dialout ubuntu \
    && echo "ubuntu ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && mkdir -p /ws/src && chown -R ubuntu:ubuntu /ws
USER ubuntu

# PlatformIO laid out like its installer does it (micro_ros_platformio expects ~/.platformio/penv).
RUN python3 -m venv ~/.platformio/penv && ~/.platformio/penv/bin/pip install --no-cache-dir platformio
ENV PATH=/home/ubuntu/.platformio/penv/bin:$PATH

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc
WORKDIR /ws
