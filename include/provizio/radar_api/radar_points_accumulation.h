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

#ifndef PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION
#define PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION

#include "provizio/common.h"
#include "provizio/radar_api/radar_point_cloud.h"

typedef struct provizio_quaternion
{
    float w;
    float x;
    float y;
    float z;
} provizio_quaternion;

typedef struct provizio_enu_position
{
    float east_meters;
    float north_meters;
    float up_meters;
} provizio_enu_position;

typedef struct provizio_enu_fix
{
    provizio_quaternion orientation;
    provizio_enu_position position;
} provizio_enu_fix;

typedef struct provizio_accumulated_radar_point_cloud
{
    provizio_radar_point_cloud point_cloud;
    provizio_enu_fix fix_when_received;
    int8_t valid;
} provizio_accumulated_radar_point_cloud;

typedef struct provizio_accumulated_radar_point_cloud_iterator
{
    size_t point_cloud_index;
    size_t point_index;
} provizio_accumulated_radar_point_cloud_iterator;

PROVIZIO__EXTERN_C void provizio_quaternion_set_identity(provizio_quaternion *out_quaternion);
// As applied in order: z, y, x (yaw, pitch, roll)
PROVIZIO__EXTERN_C void provizio_quaternion_set_euler_angles(float x_rad, float y_rad, float z_rad,
                                                             provizio_quaternion *out_quaternion);
PROVIZIO__EXTERN_C uint8_t provizio_quaternion_is_valid_rotation(const provizio_quaternion *quaternion);

PROVIZIO__EXTERN_C void provizio_accumulated_radar_point_clouds_init(
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

PROVIZIO__EXTERN_C provizio_accumulated_radar_point_cloud_iterator provizio_accumulate_radar_point_cloud(
    const provizio_radar_point_cloud *point_cloud, const provizio_enu_fix *fix_when_received,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

PROVIZIO__EXTERN_C size_t provizio_accumulated_radar_point_clouds_count(
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

PROVIZIO__EXTERN_C size_t provizio_accumulated_radar_points_count(
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

PROVIZIO__EXTERN_C int8_t provizio_accumulated_radar_point_cloud_iterator_is_end(
    const provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

PROVIZIO__EXTERN_C void provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

PROVIZIO__EXTERN_C void provizio_accumulated_radar_point_cloud_iterator_next_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

PROVIZIO__EXTERN_C const provizio_accumulated_radar_point_cloud *
provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point_cloud *optional_out_transformed_point_cloud);

PROVIZIO__EXTERN_C const provizio_radar_point *provizio_accumulated_radar_point_cloud_iterator_get_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point *optional_out_transformed_point);

#endif // PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION
