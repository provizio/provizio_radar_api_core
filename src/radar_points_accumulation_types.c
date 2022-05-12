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

void provizio_quaternion_set_identity(provizio_quaternion *out_quaternion)
{
    out_quaternion->w = 1.0F;
    out_quaternion->x = 0.0F;
    out_quaternion->y = 0.0F;
    out_quaternion->z = 0.0F;
}

void provizio_quaternion_set_euler_angles(float x_rad, float y_rad, float z_rad, provizio_quaternion *out_quaternion)
{
    const float half = 0.5F;
    const float cos_half_x = cosf(x_rad * half);
    const float sin_half_x = sinf(x_rad * half);
    const float cos_half_y = cosf(y_rad * half);
    const float sin_half_y = sinf(y_rad * half);
    const float cos_half_z = cosf(z_rad * half);
    const float sin_half_z = sinf(z_rad * half);

    out_quaternion->w = cos_half_x * cos_half_y * cos_half_z + sin_half_x * sin_half_y * sin_half_z;
    out_quaternion->x = sin_half_x * cos_half_y * cos_half_z - cos_half_x * sin_half_y * sin_half_z;
    out_quaternion->y = cos_half_x * sin_half_y * cos_half_z + sin_half_x * cos_half_y * sin_half_z;
    out_quaternion->z = cos_half_x * cos_half_y * sin_half_z - sin_half_x * sin_half_y * cos_half_z;
}

uint8_t provizio_quaternion_is_valid_rotation(const provizio_quaternion *quaternion)
{
    const float epsilon = 0.0001F;
    const float squared_length = quaternion->w * quaternion->w + quaternion->x * quaternion->x +
                                 quaternion->y * quaternion->y + quaternion->z * quaternion->z;
    return 1.0F - epsilon < squared_length && squared_length < 1.0F + epsilon;
}

float provizio_enu_distance(const provizio_enu_position *position_a, const provizio_enu_position *position_b)
{
    const float diff_east = position_a->east_meters - position_b->east_meters;
    const float diff_north = position_a->north_meters - position_b->north_meters;
    const float diff_up = position_a->up_meters - position_b->up_meters;
    return sqrtf(diff_east * diff_east + diff_north * diff_north + diff_up * diff_up);
}
