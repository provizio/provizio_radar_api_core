// Copyright 2025 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDE_TEST_ENTITIES_CALLBACKS
#define INCLUDE_TEST_ENTITIES_CALLBACKS

#include "provizio/radar_api/entities.h"

#define PROVIZIO__TEST_CALLBACK_DATA_NUM_ENTITIES_FRAMES 4
typedef struct test_provizio_radar_entities_callback_data
{
    int32_t called_times;
    provizio_radar_entities_frame last_entities_frames[PROVIZIO__TEST_CALLBACK_DATA_NUM_ENTITIES_FRAMES];
} test_provizio_radar_entities_callback_data;

void test_provizio_radar_entities_callback(const provizio_radar_entities_frame *entities_frame,
                                           provizio_radar_entities_api_context *context);

#endif // INCLUDE_TEST_ENTITIES_CALLBACKS
