# Build-and-test image for machines without a native ROS 2 Jazzy install.
# Not needed to build or run the project on Ubuntu 24.04 (see docs/setup.md).
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-robot-localization \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-tf2-tools \
        ros-jazzy-tf2-ros-py \
        ros-jazzy-rviz2 \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-cmake-flake8 \
        ros-jazzy-ament-cmake-pep257 \
        ros-jazzy-ament-cmake-xmllint \
        ros-jazzy-ament-flake8 \
        ros-jazzy-ament-pep257 \
        liburdfdom-tools \
        python3-numpy \
        python3-pil \
        python3-pytest \
        python3-yaml \
        libegl1 \
        libgl1-mesa-dri \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update --rosdistro jazzy

# Software rendering for headless Gazebo sensors.
ENV LIBGL_ALWAYS_SOFTWARE=1
WORKDIR /ws
