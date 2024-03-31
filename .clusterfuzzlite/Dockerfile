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
RUN apt-get update
RUN apt-get -y install clang && apt-get clean
RUN bash -c "cd .. && rosdep install --from-paths src --ignore-src -r -y"
RUN bash -c "cd /root && apt install -y wget libprotobuf-dev libprotobuf23 && apt install -y git build-essential libprotobuf-dev libprotoc-dev protobuf-compiler && apt install -y git build-essential autoconf automake libtool curl unzip"
RUN bash -c "cd /root && git clone --recursive https://github.com/Blickfeld/blickfeld-scanner-lib.git && mkdir blickfeld-scanner-lib/build && cd blickfeld-scanner-lib/build && cmake .. && make && make install"

COPY . $SRC/clusterfuzzlite-example1
WORKDIR clusterfuzzlite-example1
COPY .clusterfuzzlite/build.sh $SRC/
