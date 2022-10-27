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

# Dependencies versions
set(FORMAT_CMAKE_VERSION "1.7.3") # Format.cmake, optional code format checker

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

# Make sure TLS_VERIFY has any value
if(NOT DEFINED TLS_VERIFY)
  set(TLS_VERIFY ON)
endif(NOT DEFINED TLS_VERIFY)

# ccache
find_program(CCACHE ccache)
if(CCACHE AND NOT CMAKE_C_COMPILER_LAUNCHER)
  message("ccache found!")
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE}")
endif(CCACHE AND NOT CMAKE_C_COMPILER_LAUNCHER)

# Format checking option is enabled by default
if(NOT ENABLE_CHECK_FORMAT)
  set(ENABLE_CHECK_FORMAT
        "ON"
        CACHE STRING "Enable Format Checking")
endif(NOT ENABLE_CHECK_FORMAT)

# Static analysis disabled by default
if(NOT STATIC_ANALYSIS)
  set(STATIC_ANALYSIS
        "OFF"
        CACHE STRING "Enable Static Analysis")
endif(NOT STATIC_ANALYSIS)

# Generating code coverage info is disabled by default
if(NOT ENABLE_COVERAGE)
  set(ENABLE_COVERAGE
        "OFF"
        CACHE STRING "Enable Code Coverage Checks")
endif(NOT ENABLE_COVERAGE)

# Linux/macOS specific checks
if(UNIX)
  # clang-tidy (use as clang-tidy;arguments)
  set(CMAKE_C_CLANG_TIDY
    ""
    CACHE STRING "clang-tidy binary and config (C)")
  set(CMAKE_CXX_CLANG_TIDY
    ""
    CACHE STRING "clang-tidy binary and config (C++)")
  
  # Automatically enable clang-tidy if STATIC_ANALYSIS is turned on
  if(NOT CMAKE_C_CLANG_TIDY AND STATIC_ANALYSIS)
    message(STATUS "STATIC_ANALYSIS is enabled. Turning on clang-tidy.")
    find_program(CLANG_TIDY_PATH clang-tidy)
    if(CLANG_TIDY_PATH)
      set(CMAKE_C_CLANG_TIDY
        "${CLANG_TIDY_PATH}"
        CACHE STRING "clang-tidy binary and config" FORCE)
      set(CMAKE_CXX_CLANG_TIDY
        "${CLANG_TIDY_PATH}"
        CACHE STRING "clang-tidy binary and config" FORCE)
    else(CLANG_TIDY_PATH)
      message("clang-tidy not found. Appropriate checks won't be done!")
    endif(CLANG_TIDY_PATH)
  endif()
  
  # Enable Format.cmake (forked from https://github.com/TheLartians/Format.cmake), if supported and enabled
  if(ENABLE_CHECK_FORMAT)
    execute_process(COMMAND clang-format --version OUTPUT_VARIABLE CLANG_FORMAT_VERSION)
    if(CLANG_FORMAT_VERSION)
      string(STRIP CLANG_FORMAT_VERSION "${CLANG_FORMAT_VERSION}")
      string(REPLACE "\n" "" CLANG_FORMAT_VERSION "${CLANG_FORMAT_VERSION}")
      string(REPLACE "clang-format version " "" CLANG_FORMAT_VERSION "${CLANG_FORMAT_VERSION}")
      string(REGEX MATCH "[^\.]*" CLANG_FORMAT_VERSION_MAJOR "${CLANG_FORMAT_VERSION}")
    endif(CLANG_FORMAT_VERSION)
    if(CLANG_FORMAT_VERSION_MAJOR AND NOT CLANG_FORMAT_VERSION_MAJOR LESS 10)
      message("Compatible version of clang-format found: ${CLANG_FORMAT_VERSION}")
      set(FORMAT_CMAKE_PATH
          "${CMAKE_BINARY_DIR}/Format.cmake-${FORMAT_CMAKE_VERSION}")
      if(NOT EXISTS "${FORMAT_CMAKE_PATH}")
        set(FORMAT_CMAKE_DOWNLOAD_URL
            "https://github.com/provizio/Format.cmake/archive/refs/tags/v${FORMAT_CMAKE_VERSION}.tar.gz"
        )
        file(DOWNLOAD "${FORMAT_CMAKE_DOWNLOAD_URL}" "${FORMAT_CMAKE_PATH}.tar.gz" TLS_VERIFY ${TLS_VERIFY})
        execute_process(COMMAND tar -xf "${FORMAT_CMAKE_PATH}.tar.gz" -C
                                "${CMAKE_BINARY_DIR}")
      endif(NOT EXISTS "${FORMAT_CMAKE_PATH}")
      set(FORMAT_SKIP_CMAKE YES CACHE BOOL "" FORCE)
      add_subdirectory("${FORMAT_CMAKE_PATH}" EXCLUDE_FROM_ALL)
    
      # Automatically enable clang-format checks if STATIC_ANALYSIS is turned on
      if(STATIC_ANALYSIS)
        message(STATUS "STATIC_ANALYSIS is enabled. Adding check-format to ALL.")
        add_custom_target(check-format-all ALL DEPENDS check-format)
      endif(STATIC_ANALYSIS)
    else(CLANG_FORMAT_VERSION_MAJOR AND NOT CLANG_FORMAT_VERSION_MAJOR LESS 10)
      if (CLANG_FORMAT_VERSION)
        # clang-format is too old
        message(WARNING "clang-format version ${CLANG_FORMAT_VERSION} is not supported. Format checks won't be done!")
      else(CLANG_FORMAT_VERSION)
        # clang-format is missing
        message(WARNING "clang-format is missing. Format checks won't be done!")
      endif(CLANG_FORMAT_VERSION)
    endif(CLANG_FORMAT_VERSION_MAJOR AND NOT CLANG_FORMAT_VERSION_MAJOR LESS 10)
  endif(ENABLE_CHECK_FORMAT)

  # cppcheck
  find_program(CPPCHECK_PATH cppcheck)
  if(CPPCHECK_PATH)
    execute_process(COMMAND "${CPPCHECK_PATH}" --version OUTPUT_VARIABLE CPPCHECK_VERSION)
    string(STRIP CPPCHECK_VERSION "${CPPCHECK_VERSION}")
    string(REPLACE "\n" "" CPPCHECK_VERSION "${CPPCHECK_VERSION}")
    string(REPLACE "Cppcheck " "" CPPCHECK_VERSION "${CPPCHECK_VERSION}")
    if(NOT "${CPPCHECK_VERSION}" VERSION_LESS "1.90")
      message("Compatible version of cppcheck found: ${CPPCHECK_VERSION}. Adding target cppcheck-misra.")
      add_custom_target(cppcheck-misra "${CPPCHECK_PATH}" --enable=all --inline-suppr --check-config --suppress=missingIncludeSystem --dump --addon=misra.py --project="${CMAKE_BINARY_DIR}/compile_commands.json")
      # Automatically enable cppcheck-misra if STATIC_ANALYSIS is turned on
      if(STATIC_ANALYSIS)
        message(STATUS "STATIC_ANALYSIS is enabled. Adding cppcheck-misra to ALL.")
        add_custom_target(cppcheck-misra-all ALL DEPENDS cppcheck-misra)
      endif(STATIC_ANALYSIS)
    else(NOT "${CPPCHECK_VERSION}" VERSION_LESS "1.90")
      message(WARNING "Unsupported version of cppcheck: ${CPPCHECK_VERSION}. cppcheck-misra target won't be added.")
    endif(NOT "${CPPCHECK_VERSION}" VERSION_LESS "1.90")
  else(CPPCHECK_PATH)
    message(WARNING "cppcheck not found!")
  endif(CPPCHECK_PATH)

  # ASan+LSan+UBsan / MSan / TSan
  if(NOT ENABLE_TSAN AND NOT ENABLE_MSAN AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    # ASan is automatically enabled in Debug builds, unless TSan or MSan is enabled
    set(ENABLE_ASAN TRUE)
  endif(NOT ENABLE_TSAN AND NOT ENABLE_MSAN AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  if(ENABLE_TSAN)
    message("Enabling TSan")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=thread")
  elseif(ENABLE_MSAN)
    message("Enabling MSan")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=memory")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=memory")
  elseif(ENABLE_ASAN)
    message("Enabling ASan, LSan (GCC) and UBSan")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined")
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=leak")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=leak")
    endif(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  endif(ENABLE_TSAN)
endif(UNIX)

if(ENABLE_COVERAGE)
  if(BUILD_TESTING)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        message("Enabling code coverage checking")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")

        add_custom_target(code_coverage
          COMMAND lcov -c -d "${CMAKE_BINARY_DIR}" -o "${CMAKE_BINARY_DIR}/lcov.info" --exclude '/usr/include/*' --exclude '/usr/lib/*' --exclude '/usr/local/*'
          COMMAND genhtml "${CMAKE_BINARY_DIR}/lcov.info" -o "${CMAKE_BINARY_DIR}/code_coverage_report" > "${CMAKE_BINARY_DIR}/genhtml.out" 2>&1
          COMMAND cat "${CMAKE_BINARY_DIR}/genhtml.out"
          COMMAND grep "lines.*100\.0\%" "${CMAKE_BINARY_DIR}/genhtml.out")
      else(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        message(WARNING "Can't enable code coverage checking as only GCC is supported")
      endif(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    else(CMAKE_BUILD_TYPE STREQUAL "Debug")
      message(WARNING "Can't enable code coverage checking as only Debug builds are supported")
    endif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  else(BUILD_TESTING)
    message(WARNING "Can't enable code coverage checking as BUILD_TESTING is off")
  endif(BUILD_TESTING)
endif(ENABLE_COVERAGE)

# Enable generating compile_commands.json to be used by tools
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Strict language standards
set(C_STANDARD_REQUIRED TRUE)
set(CXX_STANDARD_REQUIRED TRUE)
if (NOT MSVC)
  add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-unknown-pragmas)
else()
  add_compile_options(/W4 /WX /wd4068 /wd4996)
endif()

# linmath.h (https://github.com/provizio/linmath.h)
set(LINMATH_BRANCH "provizio")
set(LINMATH_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/linmath")
set(LINMATH_HEADER_NAME "linmath.h")
set(LINMATH_DOWNLOAD_URL "https://raw.githubusercontent.com/provizio/linmath.h/${LINMATH_BRANCH}/${LINMATH_HEADER_NAME}")
file(DOWNLOAD "${LINMATH_DOWNLOAD_URL}" "${LINMATH_HEADER_DIR}/${LINMATH_HEADER_NAME}" TLS_VERIFY ${TLS_VERIFY})

# Define the core library
add_library(provizio_radar_api_core STATIC src/common.c src/socket.c src/radar_point_cloud.c src/radar_points_accumulation.c src/radar_points_accumulation_filters.c src/radar_points_accumulation_types.c src/util.c src/core.c)
target_include_directories(provizio_radar_api_core PUBLIC include)
target_include_directories(provizio_radar_api_core SYSTEM PRIVATE "${LINMATH_HEADER_DIR}")
set_property(TARGET provizio_radar_api_core PROPERTY C_STANDARD 99) # As defined by MISRA
if(WIN32)
  target_link_libraries(provizio_radar_api_core ws2_32)
endif(WIN32)

# Installation config
install(TARGETS provizio_radar_api_core
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/provizio DESTINATION include)

# Add tests, if enabled
if(BUILD_TESTING)
  add_subdirectory(test)
endif(BUILD_TESTING)
