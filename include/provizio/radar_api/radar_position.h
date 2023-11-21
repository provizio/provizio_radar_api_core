// Copyright 2022 Provizio Ltd.
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

#ifndef PROVIZIO_RADAR_API_RADAR_POSITION
#define PROVIZIO_RADAR_API_RADAR_POSITION

#include "provizio/common.h"

typedef enum provizio_radar_position
{
    provizio_radar_position_front_center = 0,
    provizio_radar_position_front_left = 1,
    provizio_radar_position_front_right = 2,
    provizio_radar_position_rear_left = 3,
    provizio_radar_position_rear_right = 4,
    provizio_radar_position_rear_center = 5,

    provizio_radar_position_custom = ((uint16_t)0x1000),

    provizio_radar_position_unknown = ((uint16_t)0xffff),
    provizio_radar_position_any = provizio_radar_position_unknown,
    provizio_radar_position_max = provizio_radar_position_unknown
} provizio_radar_position;

#endif // PROVIZIO_RADAR_API_RADAR_POSITION
