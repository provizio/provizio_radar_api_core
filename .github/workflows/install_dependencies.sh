#!/bin/bash

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

# Use as:
# build.sh [STATIC_ANALYSIS=OFF] [CODE_COVERAGE=OFF]

set -eu

STATIC_ANALYSIS=${1:-"OFF"}
CODE_COVERAGE=${2:-"OFF"}
CC=${CC:-"gcc"}

if [[ "${OSTYPE}" == "darwin"* ]]; then
  # macOS

  # Install GCC/clang
  if [[ "${CC}" == "gcc" ]]; then
    brew install gcc
  else
    brew install llvm
  fi

  # Install CMake
  brew install cmake

  if [[ "${STATIC_ANALYSIS}" != "OFF" ]]; then
    if [[ "${CC}" == "gcc" ]]; then
      # Despite building with GCC, llvm tools are required
      brew install llvm
    fi

    # Install cppcheck
    brew install cppcheck

    # Install clang-format and clang-tidy
    ln -s "$(brew --prefix llvm)/bin/clang-format" "/usr/local/bin/clang-format"
    ln -s "$(brew --prefix llvm)/bin/clang-tidy" "/usr/local/bin/clang-tidy"
  fi
else
  # Linux (Ubuntu 18+ assumed)

  if [[ "${EUID}" != "0" ]]; then
    echo "Root permissions required"
    exit 1
  fi

  # Update apt cache
  apt update

  # Install GCC/clang
  if [[ "${CC}" == "gcc" ]]; then
    apt install -y --no-install-recommends gcc g++
  else
    apt install -y --no-install-recommends clang
  fi

  # Install CMake and make
  apt install -y --no-install-recommends make cmake

  if [[ "${STATIC_ANALYSIS}" != "OFF" ]]; then
    # Install cppcheck, clang-format and clang-tidy (and clang for proper clang-tidy checks)
    apt install -y --no-install-recommends clang clang-format clang-tidy cppcheck
  fi

  if [[ "${CODE_COVERAGE}" != "OFF" ]]; then
    # Install lcov
    apt install -y lcov
  fi
fi
