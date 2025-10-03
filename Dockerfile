FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    nano \
    bash-completion \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add sudo support
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Programs for testing device
RUN apt-get update && apt-get install -y \
    evtest \
    jstest-gtk \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Serial port access
RUN usermod -aG dialout $USERNAME    

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/$USERNAME/.bashrc

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]