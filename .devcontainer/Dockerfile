FROM osrf/ros:humble-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Since the container includes node using yolo models, we need to install ultralytics, 
# which requires specific versions of numpy and opencv-python. Instead of modifying the
# complete container, we create a virtual environment

RUN apt-get update && apt-get install -y \
    python3-skimage \
    python3-venv \
    && python3 -m venv /python_envs/yolo_env

# Install dependencies:
COPY requirements.txt .
RUN . /python_envs/yolo_env/bin/activate && \
    pip install -r requirements.txt && \ 
    pip install dill && \
    deactivate

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Development Toos & Terminal Configuration 
RUN sudo apt install -y terminator && \
    sudo apt install -y nano && \
    sudo apt install -y gedit


# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["terminator"]
