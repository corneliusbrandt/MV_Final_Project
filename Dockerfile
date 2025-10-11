FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ARG USERNAME=robo

# Environment Variables
ENV ROS_DOMAIN_ID=30
ENV TURTLEBOT3_MODEL=waffle_pi
ENV PROJECT_PATH=gesturebot_ws
ENV TURTLEBOT_WS=temp_tb3_ws
ENV DEBIAN_FRONTEND=noninteractive

# ALL system dependencies
RUN apt update -y && apt upgrade -y && apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-dynamixel-sdk \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gripper-controllers \
    ros-humble-moveit* \
    python3-colcon-common-extensions \
    vim wget xpra x11-utils mesa-utils git python3-pip sudo adduser

# Mediapipe and OpenCV
RUN pip install mediapipe opencv-python

# User Setup
RUN adduser --disabled-password --gecos "" $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# HIGHLY condensed TurtleBot3 Setup, Sourcing, Cloning and Building
RUN source /opt/ros/humble/setup.bash && mkdir -p ~/$TURTLEBOT_WS/src \
    && cd ~/$TURTLEBOT_WS/src/ \
    && git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git \
    && cd ~/$TURTLEBOT_WS \
    && colcon build --symlink-install


# Sourcing
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
    && echo "source /home/$USERNAME/$TURTLEBOT_WS/install/setup.bash" >> ~/.bashrc


WORKDIR /home/$USERNAME/$PROJECT_PATH

CMD /bin/bash -c "rosdep init ; rosdep update && rosdep install --from-paths src -i -y && colcon build --symlink-install ; source install/setup.bash ; /bin/bash -i"
