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

#include "provizio/radar_api/radar_points_accumulation_types.h"

#include <math.h>

float provizio_enu_distance(const provizio_enu_position *position_a, const provizio_enu_position *position_b)
{
    const float diff_east = position_a->east_meters - position_b->east_meters;
    const float diff_north = position_a->north_meters - position_b->north_meters;
    const float diff_up = position_a->up_meters - position_b->up_meters;
    return sqrtf((diff_east * diff_east) + (diff_north * diff_north) + (diff_up * diff_up));
}
