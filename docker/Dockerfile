ARG ROS_DISTRO
ARG UBUNTU_VERSION
FROM osrf/ros:${ROS_DISTRO}-desktop-${UBUNTU_VERSION}

RUN echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y wget git
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -

RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y \
    python-catkin-tools \
    python-pip \
    python-rosdep \
    python-setuptools \
    python-wstool

RUN pip install 'pip==9.0.3' && \
    pip install 'setuptools==44.1.0'

ARG ROS_DISTRO
RUN cd ~ && \
    mkdir -p ros/${ROS_DISTRO}/src && \
    cd ros/${ROS_DISTRO}/src && \
    wstool init && \
    wstool set jsk-ros-pkg/jsk_visualization https://github.com/jsk-ros-pkg/jsk_visualization.git -v master --git -y && \
    wstool up -j 2

RUN rosdep update --include-eol-distros

# /opt/ros/${ROS_DISTRO}/share can be changed after rosdep install, so we run it 3 times.
RUN for i in $(seq 3); do rosdep install --rosdistro ${ROS_DISTRO} -r -y -i --from-paths /opt/ros/${ROS_DISTRO}/share ~/ros/${ROS_DISTRO}/src; done

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ~/ros/${ROS_DISTRO} && \
    catkin build; \
