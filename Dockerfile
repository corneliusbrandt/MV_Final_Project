FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ARG USERNAME=robo

ENV TURTLEBOT3_MODEL=waffle_pi
ENV ROS_DOMAIN_ID=30
ENV PROJECT_PATH=gesturebot_ws

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y 
RUN apt upgrade -y

RUN apt install ros-humble-navigation2 -y
RUN apt install ros-humble-nav2-bringup -y
RUN apt install ros-humble-gazebo-* -y
RUN apt install ros-humble-cartographer -y
RUN apt install ros-humble-cartographer-ros -y

RUN apt install sudo -y
RUN apt install adduser -y
RUN apt install vim -y
RUN apt install wget -y
RUN apt install xpra -y
RUN apt install x11-utils -y
RUN apt install mesa-utils -y
RUN apt install python3-pip -y
RUN apt install git -y

RUN adduser --disabled-password --gecos "" $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

RUN source /opt/ros/humble/setup.bash
RUN mkdir -p ~/turtlebot3_ws/src

RUN cd ~/turtlebot3_ws/src/ \
    && git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN cd ~/turtlebot3_ws/src/ \ 
    && git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN cd ~/turtlebot3_ws/src/ \ 
    && git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git

RUN sudo apt install python3-colcon-common-extensions

RUN cd ~/turtlebot3_ws \ 
    && source /opt/ros/humble/setup.bash && colcon build --packages-select dynamixel_sdk --symlink-install --parallel-workers 1
RUN cd ~/turtlebot3_ws \ 
    && source /opt/ros/humble/setup.bash && colcon build --packages-select turtlebot3_msgs --symlink-install --parallel-workers 1
RUN cd ~/turtlebot3_ws \ 
    && source /opt/ros/humble/setup.bash && colcon build --packages-select turtlebot3_node --symlink-install --parallel-workers 1

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'export ROS_DOMAIN_ID=$ROS_DOMAIN_ID #TURTLEBOT3' >> ~/.bashrc
RUN source ~/.bashrc

RUN cd ~/turtlebot3_ws/src/ \ 
    && git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN cd ~/turtlebot3_ws \ 
    && source /opt/ros/humble/setup.bash && colcon build --packages-select turtlebot3_gazebo --symlink-install --parallel-workers 1

RUN cd ~/turtlebot3_ws \ 
    && source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 1

RUN source ~/turtlebot3_ws/install/setup.bash

RUN echo 'export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL' >> ~/.bashrc

WORKDIR /home/$USERNAME/$PROJECT_PATH

RUN echo $USERNAME $PROJECT_PATH

CMD /bin/bash -c "cd ~/$PROJECT_PATH && echo | pwd && rosdep init || true && rosdep update && rosdep install --from-paths src -i -y && source install/setup.bash && /bin/bash -i"
