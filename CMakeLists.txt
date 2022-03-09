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

project(provizio_radar_api_core VERSION 0.1.0)

# Enable CTest
include(CTest)
enable_testing()

# Release by default
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Choose the type of build." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Strict language standards
set(C_STANDARD_REQUIRED TRUE)
set(CXX_STANDARD_REQUIRED TRUE)

# Define the core library
add_library(provizio_radar_api_core src/core.c)
target_include_directories(provizio_radar_api_core PUBLIC include)
set_property(TARGET provizio_radar_api_core PROPERTY C_STANDARD 99)

if(BUILD_TESTING)
    add_subdirectory(test)
endif(BUILD_TESTING)
