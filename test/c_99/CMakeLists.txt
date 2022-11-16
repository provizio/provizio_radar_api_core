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

cmake_minimum_required(VERSION 3.1.0)

if(WIN32)
  # Resolve pthread-win32 (https://github.com/provizio/pthread-win32)
  set(PTHREAD_WIN32_BINARY_DIR
      "${CMAKE_CURRENT_BINARY_DIR}/pthread_win32_build")
  set(PTHREAD_WIN32_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/pthread_win32")
  set(PTHREAD_WIN32_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
  set(PTHREAD_WIN32_GITHUB_PROJECT "provizio/pthread-win32")
  set(PTHREAD_WIN32_GITHUB_BRANCH "v3.0.3.1")
  set(PTHREAD_WIN32_INSTALL_DIR "${PTHREAD_WIN32_BINARY_DIR}/install")
  ExternalProject_Add(
    pthread_win32
    GIT_REPOSITORY "https://github.com/${PTHREAD_WIN32_GITHUB_PROJECT}.git"
    GIT_TAG "${PTHREAD_WIN32_GITHUB_BRANCH}"
    PREFIX "${PTHREAD_WIN32_PREFIX}"
    SOURCE_DIR "${PTHREAD_WIN32_SOURCE_DIR}"
    BINARY_DIR "${PTHREAD_WIN32_BINARY_DIR}"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
               "-DCMAKE_INSTALL_PREFIX=${PTHREAD_WIN32_INSTALL_DIR}"
    INSTALL_DIR "${PTHREAD_WIN32_INSTALL_DIR}")
  include_directories(
    "${PTHREAD_WIN32_INSTALL_DIR}/ARM/${CMAKE_BUILD_TYPE}/include"
    "${PTHREAD_WIN32_INSTALL_DIR}/ARM64/${CMAKE_BUILD_TYPE}/include"
    "${PTHREAD_WIN32_INSTALL_DIR}/x86_64/${CMAKE_BUILD_TYPE}/include"
    "${PTHREAD_WIN32_INSTALL_DIR}/x86/${CMAKE_BUILD_TYPE}/include"
    "${PTHREAD_WIN32_INSTALL_DIR}/x64/${CMAKE_BUILD_TYPE}/include")
  link_directories(
    "${PTHREAD_WIN32_INSTALL_DIR}/ARM/${CMAKE_BUILD_TYPE}/lib"
    "${PTHREAD_WIN32_INSTALL_DIR}/ARM64/${CMAKE_BUILD_TYPE}/lib"
    "${PTHREAD_WIN32_INSTALL_DIR}/x86_64/${CMAKE_BUILD_TYPE}/lib"
    "${PTHREAD_WIN32_INSTALL_DIR}/x86/${CMAKE_BUILD_TYPE}/lib"
    "${PTHREAD_WIN32_INSTALL_DIR}/x64/${CMAKE_BUILD_TYPE}/lib")
  set(PTHREAD_LIB libpthreadVC3)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PTHREAD_LIB "${PTHREAD_LIB}d")
  endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

  set(MATH_LIB "")
else(WIN32)
  find_package(Threads)
  set(PTHREAD_LIB Threads::Threads)
  set(MATH_LIB m)
endif(WIN32)

add_executable(
  provizio_radar_api_core_test_c_99
  src/test_main.c
  src/test_common.c
  src/test_util.c
  src/test_radar_point_cloud.c
  src/test_radar_points_accumulation_types.c
  src/test_radar_points_accumulation_filters.c
  src/test_radar_points_accumulation.c
  src/test_core.c)
target_include_directories(provizio_radar_api_core_test_c_99
                           PRIVATE ${CMAKE_BINARY_DIR}/linmath)
target_link_libraries(provizio_radar_api_core_test_c_99 provizio_radar_api_core
                      unity_testing ${MATH_LIB} ${PTHREAD_LIB})
set_property(TARGET provizio_radar_api_core_test_c_99 PROPERTY C_STANDARD 99
)# As defined by MISRA
add_dependencies(provizio_radar_api_core_test_c_99 unity)

add_test(NAME provizio_radar_api_core_test_c_99
         COMMAND provizio_radar_api_core_test_c_99)
