FROM osrf/ros:humble-desktop

RUN apt update
RUN apt install nano

#install stuff
RUN apt-get update \
    && apt-get install -y \
    wget curl unzip \
    lsb-release \
    mesa-utils \
    build-essential \
    && apt-get clean

# Install Gazebo
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs

# RUN mkdir -p /robots_ws/src && \
#     cd /robots_ws/src

# # sjtu drone setup
# RUN cd /robots_ws/src && \
#     git clone https://github.com/albud187/sjtu-drone.git 

RUN curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp && mkdir -p ~/.gazebo/models/ && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip

# RUN /bin/bash -c 'cd /robots_ws/ \
#     && source /opt/ros/humble/setup.bash \
#     && rosdep install --from-paths src --ignore-src -r -y \
#     && colcon build'

RUN apt install -y python3-pip
COPY requirements.txt requirements.txt
RUN python3 -m pip install -r requirements.txt

RUN pip install "numpy<2"

RUN apt update && \
    apt install -y xterm && \
    apt install ros-humble-ros2-control -y && \
    apt install ros-humble-controller-manager -y && \
    apt install ros-humble-ros2-controllers -y && \
    apt install ros-humble-gazebo-ros2-control -y && \
    apt-get install ros-humble-xacro

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]

# Clean up
RUN rm -rf /var/lib/apt/lists/*