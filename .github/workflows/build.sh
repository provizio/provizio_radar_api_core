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
# build.sh [CMake arguments]

set -eu

cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)

export CC=${CC:-"gcc"}
if [ -z "${CXX:-}" ]; then
    case "${CC}" in
        gcc)
            export CXX=g++
            ;;
        clang)
            export CXX=clang++
            ;;
        *)
            ;;
    esac
fi

mkdir -p ../../build
cd ../../build

cmake .. $@
cmake --build . -- -j 16
