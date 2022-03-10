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

# Release by default
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Choose the type of build." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Make sure TLS_VERIFY has any value
if(NOT DEFINED TLS_VERIFY)
      set(TLS_VERIFY ON)
    endif(NOT DEFINED TLS_VERIFY)

# Static analysis disabled by default
if(NOT STATIC_ANALYSIS)
  set(STATIC_ANALYSIS
        "OFF"
        CACHE STRING "Static analysis")
endif(NOT STATIC_ANALYSIS)

# Linux/macOS specific checks
if(UNIX)
  # clang-tidy (use as clang-tidy;arguments)
  set(CMAKE_CXX_CLANG_TIDY
    ""
    CACHE STRING "clang-tidy binary and config")
  
  # Automatically enable clang-tidy if STATIC_ANALYSIS is turned on
  if(NOT CMAKE_CXX_CLANG_TIDY AND STATIC_ANALYSIS)
    message(STATUS "STATIC_ANALYSIS is enabled. Turning on clang-tidy.")
    set(CMAKE_CXX_CLANG_TIDY
      "clang-tidy"
      CACHE STRING "clang-tidy binary and config" FORCE)
  endif()
  
  # Enable Format.cmake (forked from https://github.com/TheLartians/Format.cmake)
  set(FORMAT_CMAKE_VERSION "1.7.3")
  set(FORMAT_CMAKE_PATH
      "${CMAKE_BINARY_DIR}/Format.cmake-${FORMAT_CMAKE_VERSION}")
  if(NOT EXISTS "${FORMAT_CMAKE_PATH}")
    set(FORMAT_CMAKE_DOWNLOAD_URL
        "https://github.com/provizio/Format.cmake/archive/refs/tags/v${FORMAT_CMAKE_VERSION}.tar.gz"
    )
    file(DOWNLOAD "${FORMAT_CMAKE_DOWNLOAD_URL}" "${FORMAT_CMAKE_PATH}.tar.gz"
         TLS_VERIFY ${TLS_VERIFY})
    execute_process(COMMAND tar -xf "${FORMAT_CMAKE_PATH}.tar.gz" -C
                            "${CMAKE_BINARY_DIR}")
  endif(NOT EXISTS "${FORMAT_CMAKE_PATH}")
  set(FORMAT_SKIP_CMAKE YES CACHE BOOL "" FORCE)
  add_subdirectory("${FORMAT_CMAKE_PATH}" EXCLUDE_FROM_ALL)
endif(UNIX)

# Automatically enable clang-format checks if STATIC_ANALYSIS is turned on
if(STATIC_ANALYSIS)
  message(STATUS "STATIC_ANALYSIS is enabled. Adding check-format to ALL.")
  add_custom_target(check-format-all ALL DEPENDS check-format)
endif()

# Enable generating compile_commands.json to be used by tools
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Strict language standards
set(C_STANDARD_REQUIRED TRUE)
set(CXX_STANDARD_REQUIRED TRUE)
add_compile_options(-Wall -Wextra -pedantic -Werror)

# Define the core library
add_library(provizio_radar_api_core src/core.c)
target_include_directories(provizio_radar_api_core PUBLIC include)
set_property(TARGET provizio_radar_api_core PROPERTY C_STANDARD 99)

# Enable CTest
include(CTest)
enable_testing()
if(BUILD_TESTING)
  add_subdirectory(test)
endif(BUILD_TESTING)
