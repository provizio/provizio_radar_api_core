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

#ifndef PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION_TYPES
#define PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION_TYPES

#include "provizio/common.h"
#include "provizio/radar_api/radar_point_cloud.h"

/**
 * @brief Represents a quaternion, normally a unit quaternion storing a spatial orientation.
 *
 * @see https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
 * @see provizio_quaternion_set_identity
 * @see provizio_quaternion_set_euler_angles
 * @see provizio_quaternion_is_valid_rotation
 */
typedef struct provizio_quaternion
{
    float w;
    float x;
    float y;
    float z;
} provizio_quaternion;

/**
 * @brief Represents a position as right-handed cartesian coordinates (East, North, Up) in meters, relative to whatever
 * reference point - normally on Earth's surface.
 *
 * @see https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
 */
typedef struct provizio_enu_position
{
    float east_meters;
    float north_meters;
    float up_meters;
} provizio_enu_position;

/**
 * @brief Represents an orientation and position.
 *
 * @see provizio_quaternion
 * @see provizio_enu_position
 */
typedef struct provizio_enu_fix
{
    provizio_quaternion orientation; // Unit quaternion storing a spatial orientation. Identity quaternion stands for
                                     // east-looking orientation.
    provizio_enu_position position;  // Position as right-handed cartesian coordinates (East, North, Up) in meters,
                                     // relative to whatever reference point - normally on Earth's surface.
} provizio_enu_fix;

/**
 * @brief Represents a single past point cloud with a provizio_enu_fix of the radar that captured it at the moment of
 * capture.
 *
 * @see provizio_radar_point_cloud
 * @see provizio_enu_fix
 * @see provizio_accumulated_radar_point_clouds_init
 * @see provizio_accumulate_radar_point_cloud
 */
typedef struct provizio_accumulated_radar_point_cloud
{
    provizio_radar_point_cloud point_cloud;
    provizio_enu_fix fix_when_received;
} provizio_accumulated_radar_point_cloud;

/**
 * @brief An iterator pointing to a specific radar point of a specific provizio_accumulated_radar_point_cloud. Used for
 * iterating over accumulated point clouds and points - from newest to oldest.
 *
 * @see provizio_accumulated_radar_point_cloud
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_is_end
 * @see provizio_accumulated_radar_point_clouds_count
 * @see provizio_accumulated_radar_points_count
 * @see provizio_accumulated_radar_point_cloud_iterator_next_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_next_point
 * @see provizio_accumulated_radar_point_cloud_iterator_get_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_get_point
 */
typedef struct provizio_accumulated_radar_point_cloud_iterator
{
    size_t point_cloud_index;
    size_t point_index;
} provizio_accumulated_radar_point_cloud_iterator;

/**
 * @brief Sets the specified quaternion to identity, i.e. east-looking orientation.
 *
 * @param out_quaternion The quaternion to be set.
 * @see provizio_quaternion
 */
PROVIZIO__EXTERN_C void provizio_quaternion_set_identity(provizio_quaternion *out_quaternion);

/**
 * @brief Sets the specified quaternion from the specified Euler angles, as applied in this order: z, y, x (yaw, pitch,
 * roll).
 *
 * @param x_rad Rotation around the forward (roll) or east axis, depending on the context (radians).
 * @param y_rad Rotation around the left (pitch) or north axis, depending on the context (radians).
 * @param z_rad Rotation around the up (yaw) axis (radians).
 * @param out_quaternion The rotation/orientation quaternion to be set.
 * @see https://en.wikipedia.org/wiki/Euler_angles
 * @see provizio_quaternion
 * @see provizio_quaternion_set_identity
 */
PROVIZIO__EXTERN_C void provizio_quaternion_set_euler_angles(float x_rad, float y_rad, float z_rad,
                                                             provizio_quaternion *out_quaternion);

/**
 * @brief Checks if the specified quaternion is a valid rotation/orientation quaternion.
 *
 * @param quaternion The quaternion to be checked.
 * @return Non-zero if valid, zero otherwise.
 * @see provizio_quaternion
 */
PROVIZIO__EXTERN_C uint8_t provizio_quaternion_is_valid_rotation(const provizio_quaternion *quaternion);

/**
 * @brief Measures distance (in meters) between two ENU positions
 *
 * @param position_a One ENU position
 * @param position_b Another ENU position
 * @return Distance between position_a and position_b in meters
 * @see provizio_enu_position
 */
PROVIZIO__EXTERN_C float provizio_enu_distance(const provizio_enu_position *position_a,
                                               const provizio_enu_position *position_b);

#endif // PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION_TYPES
