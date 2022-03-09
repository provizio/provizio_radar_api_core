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

add_executable(provizio_radar_api_core_test_c_99 src/test_core.c)
target_link_libraries(provizio_radar_api_core_test_c_99 provizio_radar_api_core)
set_property(TARGET provizio_radar_api_core_test_c_99 PROPERTY C_STANDARD 99)
add_test(NAME provizio_radar_api_core_test_c_99 COMMAND provizio_radar_api_core_test_c_99)
