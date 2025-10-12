FROM ros:humble-ros-base 

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

# Packages for running URDF, Gazebo, and Rviz
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Packages for image and vision processing 
RUN apt-get update \
    && apt-get install -y \
    ros-humble-image-tools \
    ros-humble-vision-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Git and ROS2 build tools
RUN apt-get update \
    && apt-get install -y \
    git \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Rosdep update
RUN rosdep update

######################
## .bashrc sections ##
######################

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Set DOMAIN_ID for Go2 Robot
RUN echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc

# Set Physical Robot connection parameters
RUN echo "export ROBOT_IP=\"192.168.0.224\"" >> ~/.bashrc
RUN echo "CONN_TYPE=\"webrtc\"" >> ~/.bashrc

###############################
## Setting up ROS2 workspace ##
###############################

# Create a workspace
RUN mkdir -p ~/go2_ws/src
WORKDIR /home/$USERNAME/go2_ws/src

# Clone the Unofficial Go2 ROS2 SDK repository & Install dependencies
RUN git clone --recurse-submodules https://github.com/abizovnuralem/go2_ros2_sdk.git go2_ros2_sdk
WORKDIR /home/$USERNAME/go2_ws/src/go2_ros2_sdk
RUN pip install --no-cache-dir -r requirements.txt && \
    rm -rf ~/.cache/pip

# Install additional dependencies using rosdep and colcon build
WORKDIR /home/$USERNAME/go2_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN colcon build --symlink-install
RUN source install/setup.bash