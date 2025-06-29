FROM osrf/ros:humble-desktop-full-jammy
#FROM moveit/moveit2:humble-release

# Add ros user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Add normal user and give sudo privileges
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*
    
# Install binary dependencies for smart_diffbot
RUN apt-get update \
  && apt-get install -y ros-humble-xacro \
     ros-humble-joint-state-publisher \
     ros-humble-joint-state-publisher-gui \
     ignition-fortress \
     python3-pip \
     ros-humble-ros-gz \
     ros-humble-ros2-control \
     ros-humble-ros2-controllers \
     ros-humble-robot-localization \
     ros-humble-navigation2 \
     ros-humble-tf-transformations \
  && rm -rf /var/lib/apt/lists/*

# Install Moveit! dependencies 
RUN apt-get update \
  && apt-get install -y ros-humble-moveit \
    ros-humble-ur 

# Install Dynamixel dependencies 
RUN usermod -a -G dialout ${USERNAME}
RUN apt-get update \
  && apt-get install -y ros-humble-dynamixel-sdk \
    ros-humble-dynamixel-workbench-toolbox

# Install additional tools
RUN apt-get update \
  && apt-get install -y wget \
  && rm -rf /var/lib/apt/lists/*

# Switch from root to user
USER $USERNAME

# Create directories for workspaces
RUN mkdir -p /home/${USERNAME}/ros2_ws && mkdir -p /home/${USERNAME}/prerequisite_ws/src

# Clone prerequisites
RUN cd /home/${USERNAME}/prerequisite_ws/src && \
    git clone --branch humble https://github.com/ros-controls/gz_ros2_control.git && \
    git clone --branch dashing https://github.com/blacksoul000/rviz_satellite.git && \
    git clone --branch main https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git

# Install required python packages
RUN pip install transforms3d setuptools==58.2.0 opencv-contrib-python==4.6.0.66 

# Run rosdep update to update dependencies
RUN rosdep update

# Build prerequisites - using bash as shell from now on, so that we can easily source workspaces
SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
        cd /home/${USERNAME}/prerequisite_ws && \
        colcon build --symlink-install

# Add source statements to .bashrc so that both ROS and the prerequisite workspace 
# are automatically sourced when a bash shell is started
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/${USERNAME}/prerequisite_ws/install/setup.bash" >> ~/.bashrc