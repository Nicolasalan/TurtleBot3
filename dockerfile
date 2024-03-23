ARG IMG=osrf/ros:humble-desktop-full
FROM ${IMG}

ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

SHELL [ "/bin/bash" , "-c"]

# Setup Environment
RUN apt-get update \
 && apt-get install -yq python3-pip apt-utils git vim python3-colcon-common-extensions

# Install Dependencies with pip
RUN pip3 install transforms3d setuptools==58.2.0 pyserial smbus trimesh scipy pandas pytest matplotlib

RUN apt-get update
RUN apt install -y xterm tmux git
RUN apt install -y ros-${ROS_DISTRO}-gazebo-ros
RUN apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
RUN apt install -y ros-${ROS_DISTRO}-cartographer
RUN apt install -y ros-${ROS_DISTRO}-cartographer-ros
RUN apt install -y ros-${ROS_DISTRO}-navigation2
RUN apt install -y ros-${ROS_DISTRO}-nav2-bringup
RUN apt install -y ros-${ROS_DISTRO}-dynamixel-sdk
RUN apt install -y ros-${ROS_DISTRO}-turtlebot3-msgs
RUN apt install -y ros-${ROS_DISTRO}-turtlebot3
RUN apt install -y ros-${ROS_DISTRO}-turtlebot3-gazebo

RUN mkdir /turtlebot3_ws
WORKDIR /turtlebot3_ws

# Source ROS and Build
RUN cd /turtlebot3_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Source ROS and Build
RUN echo "source /turtlebot3_ws/install/setup.bash" >> ~/.bashrc

RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
RUN source ~/.bashrc

RUN echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
RUN source ~/.bashrc

RUN echo 'TURTLEBOT3_MODEL=burger' >> ~/.bashrc
RUN source ~/.bashrc

# config tmux
RUN echo "unbind -n Tab"                                                                    >> ~/.tmux.conf
RUN echo "set -g window-style        'fg=#ffffff,bg=#8445ca'"                               >> ~/.tmux.conf
RUN echo "set -g window-active-style 'fg=#ffffff,bg=#5e2b97'"                               >> ~/.tmux.conf
RUN echo "set-option -g default-shell '/bin/bash'"                                          >> ~/.tmux.conf
RUN echo "run-shell '. /opt/ros/humble/setup.bash'"                                         >> ~/.tmux.conf
RUN echo "set -g mouse on"                                                                  >> ~/.tmux.conf
RUN echo "bind-key -n C-Left select-pane -L"                                                >> ~/.tmux.conf
RUN echo "bind-key -n C-Right select-pane -R"                                               >> ~/.tmux.conf
RUN echo "bind-key -n C-Up select-pane -U"                                                  >> ~/.tmux.conf
RUN echo "bind-key -n C-Down select-pane -D"                                                >> ~/.tmux.conf
RUN echo "bind -n M-Right split-window -h"                                                  >> ~/.tmux.conf
RUN echo "bind -n M-Down split-window -v"                                                   >> ~/.tmux.conf
RUN echo "bind C-c run 'tmux save-buffer - | xclip -i -sel clipboard'"                      >> ~/.tmux.conf
RUN echo "bind C-v run 'tmux set-buffer '\$(xclip -o -sel clipboard)'; tmux paste-buffer'"  >> ~/.tmux.conf
