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

# Include support for ExternalProject_Add
include(ExternalProject)

# Resolve Unity unit test framework (https://github.com/provizio/Unity)
set(UNITY_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/unity_build")
set(UNITY_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/unity")
set(UNITY_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
set(UNITY_GITHUB_PROJECT "provizio/Unity")
set(UNITY_GITHUB_BRANCH "provizio")
set(UNITY_INSTALL_DIR "${UNITY_BINARY_DIR}/install")
ExternalProject_Add(
  unity
  GIT_REPOSITORY "https://github.com/${UNITY_GITHUB_PROJECT}.git"
  GIT_TAG "${UNITY_GITHUB_BRANCH}"
  PREFIX "${UNITY_PREFIX}"
  SOURCE_DIR "${UNITY_SOURCE_DIR}"
  BINARY_DIR "${UNITY_BINARY_DIR}"
  CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
             "-DCMAKE_INSTALL_PREFIX=${UNITY_INSTALL_DIR}"
             "-DUNITY_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/include"
  INSTALL_DIR "${UNITY_INSTALL_DIR}")
include_directories("${UNITY_INSTALL_DIR}/include"
                    "${CMAKE_CURRENT_SOURCE_DIR}/include")
link_directories("${UNITY_INSTALL_DIR}/lib")
add_definitions(-DUNITY_INCLUDE_CONFIG_H)

if(MSVC)
  # Make stack 8Mb large (as default MSVCs 1Mb is not enough for storing api
  # contexts)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:8388608")
endif(MSVC)

add_subdirectory(c_99)
add_subdirectory(cpp_14)
