# Copyright 2020 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
################################################################################
FROM gcr.io/oss-fuzz-base/base-builder

# Install ROS 2 dependencies
RUN apt-get update && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2 packages and rosdep
RUN apt-get update && \
    apt-get install -y \
    ros-foxy-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    clang \
    cmake \
    ninja-build

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Install dependencies for your project
RUN mkdir -p /root/src
COPY . /root/src
RUN cd /root && rosdep install --from-paths src --ignore-src -r -y

# Install additional dependencies
RUN apt-get install -y \
    wget \
    libprotobuf-dev \
    git \
    build-essential \
    libprotoc-dev \
    protobuf-compiler \
    autoconf \
    automake \
    libtool \
    curl \
    unzip \
    libpcl-dev \
    libboost-system-dev

# Clone and build blickfeld-scanner-lib
RUN cd /root && \
    git clone --recursive https://github.com/Blickfeld/blickfeld-scanner-lib.git && \
    mkdir blickfeld-scanner-lib/build && \
    cd blickfeld-scanner-lib/build && \
    cmake -G Ninja .. && \
    cmake --build . --verbose && \
    cmake --install .

COPY . $SRC/clusterfuzzlite-blickfeldHCL
WORKDIR $SRC/clusterfuzzlite-blickfeldHCL
COPY .clusterfuzzlite/build.sh $SRC/
