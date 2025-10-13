FROM arm64v8/ros:humble-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi 

# Install ROS 2 interface packages
RUN apt-get update \
    && apt-get install -y \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rosidl-generator-c \
    ros-humble-rosidl-generator-cpp \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-rosidl-typesupport-introspection-c \
    ros-humble-rosidl-typesupport-introspection-cpp \
    ros-humble-rosidl-typesupport-fastrtps-c \
    ros-humble-rosidl-typesupport-fastrtps-cpp \
    ros-humble-builtin-interfaces \
    python3-empy \
    python3-lark \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Add user to video group to allow access to webcam
RUN usermod --append --groups video $USERNAME

# Update all packages
RUN apt update && apt upgrade -y

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Switch from root to user
USER $USERNAME

######################
## .bashrc sections ##
######################

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Set DOMAIN_ID for Go2 Robot
RUN echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc

# Set Physical Robot connection parameters
RUN echo "export ROBOT_IP=\"192.168.0.224\"" >> ~/.bashrc \
    && echo "CONN_TYPE=\"webrtc\"" >> ~/.bashrc

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

# Switch back to root to install dependencies
USER root
WORKDIR /home/$USERNAME/go2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}"

# Packages for running URDF, Gazebo, and Rviz
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rosbag2 \
    ros-humble-xacro \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Packages for image and vision processing 
RUN apt-get update \
    && apt-get install -y \
    ros-humble-image-tools \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Git and ROS2 build tools
RUN apt-get update \
    && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    && rm -rf /var/lib/apt/lists/*

# Drop privileges for the build
USER ${USERNAME}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && find /opt/ros/humble/ -name '*builtin_interfaces__rosidl_generator_c*' \
    && colcon build --symlink-install"