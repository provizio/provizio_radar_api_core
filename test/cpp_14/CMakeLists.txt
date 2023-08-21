# Copyright 2022 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.10)

find_package(Threads REQUIRED)
add_executable(provizio_radar_api_core_test_cpp_14 src/smoketest.cpp)
target_link_libraries(provizio_radar_api_core_test_cpp_14
                      provizio_radar_api_core Threads::Threads)
set_property(TARGET provizio_radar_api_core_test_cpp_14 PROPERTY CXX_STANDARD
                                                                 14)
add_dependencies(provizio_radar_api_core_test_cpp_14 unity)

add_test(NAME provizio_radar_api_core_test_cpp_14
         COMMAND provizio_radar_api_core_test_cpp_14)
