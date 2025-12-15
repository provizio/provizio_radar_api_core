// Copyright 2025 Provizio Ltd.
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

#ifndef PROVIZIO_QUATERNION
#define PROVIZIO_QUATERNION

#include "provizio/common.h"

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

#endif // PROVIZIO_QUATERNION
