FROM dorowu/ubuntu-desktop-lxde-vnc:focal


# Fix dirmngr
RUN apt-get purge dirmngr -y && apt-get update && apt-get install dirmngr -y
RUN apt-get dist-upgrade -y

RUN apt-get install python3.6 -y
RUN apt-get update && apt-get install python3-pip -y



# Adding keys for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install curl wget nano -y
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Installing ROS
RUN apt-get update && apt-get install -y ros-noetic-desktop-full 
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN apt-get install -y \
      ros-noetic-libfranka python3-catkin-tools libeigen3-dev 
RUN apt-get install vim -y
RUN rosdep init && rosdep update
RUN pip3 install rospkg

RUN mkdir -p /root/catkin_ws/src /root/Desktop /root/webots && ln -s /usr/share/applications/lxterminal.desktop /root/Desktop/lxterminal.desktop
ENV ROS_DISTRO=noetic

RUN /bin/bash -c "echo -e 'umask 000\n \
      source /opt/ros/noetic/setup.bash\n' >> /root/.bashrc "

WORKDIR /root/catkin_ws
# COPY vnc/copyws.sh /root/copyws.sh
# RUN chmod a+x /root/copyws.sh
# COPY ./src ~/catkin_ws/src

RUN apt-get update && rosdep install --from-paths . -r -y

# RUN catkin config \
#       --extend /opt/ros/noetic

# Install Moveit
RUN apt install --yes ros-noetic-moveit


# install webots

# Determine Webots version to be used and set default argument
ARG WEBOTS_VERSION=R2021a


# Install Webots runtime dependencies
RUN apt update && apt install --yes wget && rm -rf /var/lib/apt/lists/
RUN wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh
RUN chmod +x linux_runtime_dependencies.sh && ./linux_runtime_dependencies.sh && rm ./linux_runtime_dependencies.sh && rm -rf /var/lib/apt/lists/

# Install X virtual framebuffer to be able to use Webots without GPU and GUI (e.g. CI)
RUN apt update && apt install --yes xvfb && rm -rf /var/lib/apt/lists/

# Install Webots
WORKDIR /usr/local
RUN wget https://github.com/cyberbotics/webots/releases/download/$WEBOTS_VERSION/webots-$WEBOTS_VERSION-x86-64_ubuntu-18.04.tar.bz2
RUN tar xjf webots-*.tar.bz2 && rm webots-*.tar.bz2
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME /usr/local/webots
ENV PATH /usr/local/webots:${PATH}
ENV LD_LIBRARY_PATH ${WEBOTS_HOME}/lib/controller
ENV PYTHONPATH ${WEBOTS_HOME}/lib/controller/python36

RUN sudo apt-get update && apt-get install -y ros-noetic-webots-ros

# Copy codes into the environment and build
RUN apt install --yes ros-noetic-moveit-visual-tools
RUN apt install --yes ros-noetic-rviz-visual-tools

# copy files
# COPY src /root/catkin_ws/src
# RUN cd /root/catkin_ws && rosdep install --from-paths src --ignore-src -r -y && catkin build
# RUN chmod +x /root/catkin_ws/src/webots-ros/scripts/*
# Add the character transformation in Windows to solve python issues
# RUN apt-get install dos2unix -y
# RUN dos2unix /root/catkin_ws/src/webots-ros/scripts/*.py

# Copy Webots world 
# COPY webots /root/webots