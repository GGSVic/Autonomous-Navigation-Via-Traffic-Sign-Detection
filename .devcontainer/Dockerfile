FROM osrf/ros:humble-desktop-full

# Define user arguments for consistency with the host system
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Update and install all system dependencies in a single layer to ensure consistency
# Including --fix-missing to handle potential network hiccups during build
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    sudo \
    python3-skimage \
    python3-venv \
    git \
    terminator \
    nano \
    gedit \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user and grant sudo privileges for a professional ROS Lab setup
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up the virtual environment for Ultralytics/YOLOv8 to avoid dependency conflicts
RUN mkdir -p /python_envs && python3 -m venv /python_envs/yolo_env
COPY requirements.txt .
RUN . /python_envs/yolo_env/bin/activate && \
    pip install --upgrade pip && \
    pip install -r requirements.txt && \ 
    pip install dill && \
    deactivate

# Final user-level configurations for ROS 2 Humble
USER $USERNAME
# Grant webcam access and initialize rosdep
RUN sudo usermod --append --groups video $USERNAME \
    && rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Entrypoint setup for the autonomous navigation stack
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["terminator"]