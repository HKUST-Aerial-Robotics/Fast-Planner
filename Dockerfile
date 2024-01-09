# Use the official ROS Melodic base image
FROM ros:melodic

# Set the working directory
WORKDIR /workspace

# Install additional dependencies if needed
# For example, you can uncomment the line below to install a package
# Install additional dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    libxml2-dev \
    libxslt-dev \
    libarmadillo-dev \
    ros-melodic-nlopt \
    python3 \
    python3-pip \
    libeigen3-dev\
    libpcl-dev \
    ros-melodic-pcl-conversions \
    libopencv-dev \
    ros-melodic-cv-bridge \
    ros-melodic-tf\
    ros-melodic-interactive-markers\
    ros-melodic-image-geometry\
    ros-melodic-pcl-ros \
    ros-melodic-geometry-msgs \
    ros-melodic-laser-geometry \
    ros-melodic-rviz

RUN pip3 install setuptools

RUN pip3 install catkin-tools
# Copy your ROS packages into the workspace
COPY . /workspace/src/

WORKDIR /workspace


# Set environment variables
ENV ROS_DISTRO melodic
ENV ROS_VERSION 1
# RUN catkin build
RUN . /opt/ros/melodic/setup.sh && catkin_make


# Source the ROS setup file
RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc

# Expose ROS master port
EXPOSE 11311

# Set entry point to start ROS
CMD ["roscore"]
