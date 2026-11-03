# Start with an official ROS 2 base image for the desired distribution
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    RCUTILS_COLORIZED_OUTPUT=1

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Use Bash as the default shell
SHELL ["/bin/bash", "-c"]

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        iputils-ping \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
        libgtest-dev \
        libgmock-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN if [[ `getent passwd $USER_UID | cut -d":" -f 1| wc -w` > 0 ]] ; then userdel `getent passwd $USER_UID | cut -d":" -f 1` ; fi 
RUN if [[ `getent group $USER_GID | cut -d":" -f 1| wc -w` > 0 ]] ; then groupmod `getent group $USER_GID | cut -d":" -f 1` -n ${USERNAME} ; else  groupadd --gid $USER_GID ${USERNAME}  ; fi
RUN  useradd --uid $USER_UID --gid $USER_GID -m $USERNAME --groups dialout \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

# Install some ROS 2 dependencies to create a cache layer
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-jazzy-gz-sim-vendor \
        ros-jazzy-gz-plugin-vendor \
        ros-jazzy-sdformat-urdf \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-ros2controlcli \
        ros-jazzy-controller-interface \
        ros-jazzy-hardware-interface-testing \
        ros-jazzy-ament-cmake-clang-format \
        ros-jazzy-ament-cmake-clang-tidy \
        ros-jazzy-ament-cmake-ros \
        ros-jazzy-ament-cmake-test \
        ros-jazzy-controller-manager \
        ros-jazzy-ros2-control-test-assets \
        ros-jazzy-hardware-interface \
        ros-jazzy-example-interfaces \
        ros-jazzy-ros2-control-cmake \
        ros-jazzy-control-msgs \
        ros-jazzy-backward-ros \
        ros-jazzy-generate-parameter-library \
        ros-jazzy-realtime-tools \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-moveit-ros-move-group \
        ros-jazzy-moveit-kinematics \
        ros-jazzy-moveit-planners-ompl \
        ros-jazzy-moveit-ros-visualization \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-moveit-simple-controller-manager \
        ros-jazzy-rviz2 \
        ros-jazzy-xacro \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-joy \
        ros-jazzy-teleop-twist-joy \
        ros-jazzy-tsid \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Install the missing ROS 2 dependencies
COPY . /ros2_ws/src
RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws \
    && vcs import src < src/dependency.repos --recursive --skip-existing \
    && git clone https://github.com/pal-robotics/launch_pal.git /ros2_ws/src/launch_pal \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys=zed_wrapper \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf /home/$USERNAME/.ros \
    && rm -rf src \
    && mkdir -p src

COPY ./franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

RUN sudo apt update \
    && sudo apt install -y terminator
    
# Install VS code
RUN sudo apt install -y wget gpg
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
RUN sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
RUN echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
RUN rm -f packages.microsoft.gpg
RUN sudo apt install -y apt-transport-https && sudo apt update && sudo apt install -y code
RUN code --install-extension ms-vscode.cpptools && code --install-extension ms-python.python && code --install-extension  ms-vscode.cpptools-themes && code --install-extension  ms-vscode.cpptools-extension-pack && code --install-extension cschlosser.doxdocgen

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
WORKDIR /ros2_ws
