ARG from
FROM ${from}

# Timezone Configuration
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         gnupg2 \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         vim




# Install ROS desktop and Gazebo

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-noetic-desktop-full \
                                         ros-noetic-ros-control \
                                         ros-noetic-gazebo-ros-control \
                                         ros-noetic-geographic-info \
                                         ros-noetic-teleop-twist-keyboard \
                                         ros-noetic-joy \
                                         ros-noetic-effort-controllers \
                                         ros-noetic-controller-manager \
                                         python3-rosdep \
                                         python3-rosinstall \
                                         python3-rosinstall-generator \
                                         python3-wstool \
                                         python3-catkin-tools \
                                         libcanberra-gtk-module \
                                         libcanberra-gtk3-module \
                                            ros-noetic-ddynamic-reconfigure \
                                            ros-noetic-ompl \
                                            ros-noetic-moveit-core \
                                            ros-noetic-moveit-ros-planning-interface \
                                            ros-noetic-moveit-visual-tools \
                                            ros-noetic-rosparam-shortcuts \
                                            libglew-dev \
                                         ros-noetic-pid \
                                         ros-noetic-visp* && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc && \
    echo "source /workspace/devel/setup.bash"  >> ~/.bashrc

# Install extra libraries
RUN apt-get install -y python3-pip
RUN pip3 install pathlib statistics scipy
RUN pip3 install catkin-tools rosdep wstool

RUN sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
RUN git clone --recursive https://github.com/IntelRealSense/librealsense.git && cd librealsense && mkdir build && cd build && cmake ../ -DBUILD_EXAMPLES=true && make -j 24 && make install


RUN echo "export ROS_HOSTNAME=192.168.88.190" >> ~/.bashrc && \
    echo "export ROS_IP=192.168.88.190" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=http://192.168.88.190:11311" >> ~/.bashrc

RUN pip3 install gbd pyserial

