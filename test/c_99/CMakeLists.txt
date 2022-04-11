# Copyright 2022 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.1.0)

if(WIN32)
  todo("https://github.com/provizio/pthread-win32")
else(WIN32)
  find_package(Threads)
  set(PTHREAD_LIB Threads::Threads)
endif(WIN32)

add_executable(provizio_radar_api_core_test_c_99 src/test_main.c src/test_common.c src/test_util.c src/test_radar_point_cloud.c)
target_link_libraries(provizio_radar_api_core_test_c_99 provizio_radar_api_core unity_testing m ${PTHREAD_LIB})
set_property(TARGET provizio_radar_api_core_test_c_99 PROPERTY C_STANDARD 99) # As defined by MISRA
add_dependencies(provizio_radar_api_core_test_c_99 unity)

add_test(NAME provizio_radar_api_core_test_c_99 COMMAND provizio_radar_api_core_test_c_99)
