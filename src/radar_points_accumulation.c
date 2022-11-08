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

#include "provizio/radar_api/radar_points_accumulation.h"

#include <assert.h>
#include <linmath.h>
#include <string.h>

enum
{
    provizio_transformation_matrix_components = 4 * 4
};

static uint8_t provizio_accumulated_radar_point_cloud_valid(const provizio_accumulated_radar_point_cloud *point_cloud)
{
    return provizio_quaternion_is_valid_rotation(&point_cloud->fix_when_received.orientation);
}

static void provizio_transform_radar_point(const provizio_radar_point *point, const provizio_enu_fix *fix_when_received,
                                           const provizio_enu_fix *current_fix,
                                           provizio_radar_point *out_transformed_point)
{
    assert(point != NULL);
    assert(fix_when_received != NULL);
    assert(current_fix != NULL);
    assert(out_transformed_point != NULL);
    assert(provizio_quaternion_is_valid_rotation(&fix_when_received->orientation));
    assert(provizio_quaternion_is_valid_rotation(&current_fix->orientation));

    // 1. Convert point from "fix_when_received" reference frame to ENU
    // 1.1. Rotate
    vec3 point_vec3 = {point->x_meters, point->y_meters, point->z_meters};
    quat point_to_enu_quat = {fix_when_received->orientation.x, fix_when_received->orientation.y,
                              fix_when_received->orientation.z, fix_when_received->orientation.w};
    vec3 point_enu;
    quat_mul_vec3(point_enu, point_to_enu_quat, point_vec3);
    // 1.2. Translate
    point_enu[0] += fix_when_received->position.east_meters;
    point_enu[1] += fix_when_received->position.north_meters;
    point_enu[2] += fix_when_received->position.up_meters;
    // 2. Convert point from ENU to "current_fix" reference frame
    // 2.1. Reversed translate
    point_enu[0] -= current_fix->position.east_meters;
    point_enu[1] -= current_fix->position.north_meters;
    point_enu[2] -= current_fix->position.up_meters;
    // 2.2. Reversed rotate
    quat enu_to_out_point_quat = {-current_fix->orientation.x, -current_fix->orientation.y, -current_fix->orientation.z,
                                  current_fix->orientation.w};
    vec3 out_point;
    quat_mul_vec3(out_point, enu_to_out_point_quat, point_enu);

    // 3. Set out_transformed_point fields
    out_transformed_point->x_meters = out_point[0];
    out_transformed_point->y_meters = out_point[1];
    out_transformed_point->z_meters = out_point[2];
    out_transformed_point->radar_relative_radial_velocity_m_s = point->radar_relative_radial_velocity_m_s;
    out_transformed_point->ground_relative_radial_velocity_m_s = point->ground_relative_radial_velocity_m_s;
    out_transformed_point->signal_to_noise_ratio = point->signal_to_noise_ratio;
}

static void provizio_build_transformation_matrix(const provizio_enu_fix *fix_when_received,
                                                 const provizio_enu_fix *current_fix, float *out_matrix)
{
    assert(fix_when_received != NULL);
    assert(current_fix != NULL);
    assert(out_matrix != NULL);
    assert(provizio_quaternion_is_valid_rotation(&fix_when_received->orientation));
    assert(provizio_quaternion_is_valid_rotation(&current_fix->orientation));

    mat4x4 operation_mat4x4;
    mat4x4 out_mat4x4;
    assert(sizeof(out_mat4x4) == 64);

    // Given column-major matrices are used, the operations order is reversed

    // 2. Convert point from ENU to "current_fix" reference frame
    // 2.2. Reversed rotate
    quat enu_to_out_point_quat = {-current_fix->orientation.x, -current_fix->orientation.y, -current_fix->orientation.z,
                                  current_fix->orientation.w};
    mat4x4_from_quat(operation_mat4x4, enu_to_out_point_quat);
    mat4x4_dup(out_mat4x4, operation_mat4x4);
    // 2.1. Reversed translate
    mat4x4_translate(operation_mat4x4, -current_fix->position.east_meters, -current_fix->position.north_meters,
                     -current_fix->position.up_meters);
    mat4x4_mul(out_mat4x4, out_mat4x4, operation_mat4x4);
    // 1. Convert point from "fix_when_received" reference frame to ENU
    // 1.2. Translate
    mat4x4_translate(operation_mat4x4, fix_when_received->position.east_meters,
                     fix_when_received->position.north_meters, fix_when_received->position.up_meters);
    mat4x4_mul(out_mat4x4, out_mat4x4, operation_mat4x4);
    // 1.1. Rotate
    quat point_to_enu_quat = {fix_when_received->orientation.x, fix_when_received->orientation.y,
                              fix_when_received->orientation.z, fix_when_received->orientation.w};
    mat4x4_from_quat(operation_mat4x4, point_to_enu_quat);
    mat4x4_mul(out_mat4x4, out_mat4x4, operation_mat4x4);

    memcpy(out_matrix, out_mat4x4, sizeof(out_mat4x4));
}

static void provizio_transform_radar_point_cloud(const provizio_radar_point_cloud *point_cloud,
                                                 const provizio_enu_fix *fix_when_received,
                                                 const provizio_enu_fix *current_fix,
                                                 provizio_radar_point_cloud *out_transformed_point_cloud)
{
    // Copy the point cloud header
    memcpy(out_transformed_point_cloud, point_cloud, offsetof(provizio_radar_point_cloud, radar_points));

    assert(point_cloud->num_points_received <= point_cloud->num_points_expected);

    provizio_radar_point *dest_point = out_transformed_point_cloud->radar_points;
    for (const provizio_radar_point *source_point = point_cloud->radar_points,
                                    *source_end = point_cloud->radar_points + point_cloud->num_points_received;
         source_point != source_end; ++source_point, ++dest_point)
    {
        provizio_transform_radar_point(source_point, fix_when_received, current_fix, dest_point);
    }
}

void provizio_accumulated_radar_point_clouds_init(provizio_accumulated_radar_point_cloud *accumulated_point_clouds,
                                                  size_t num_accumulated_point_clouds)
{
    memset(accumulated_point_clouds, 0, sizeof(provizio_accumulated_radar_point_cloud) * num_accumulated_point_clouds);
}

provizio_accumulated_radar_point_cloud_iterator provizio_accumulate_radar_point_cloud(
    const provizio_radar_point_cloud *point_cloud, const provizio_enu_fix *fix_when_received,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_points_accumulation_filter filter, void *filter_user_data)
{
    provizio_accumulated_radar_point_cloud_iterator iterator = {0, 0};

    if (num_accumulated_point_clouds == 0)
    {
        // Nowhere to accumulate
        provizio_error("provizio_accumulate_radar_point_cloud: num_accumulated_point_clouds can't be 0");
        return iterator;
    }

    if (!provizio_quaternion_is_valid_rotation(&fix_when_received->orientation))
    {
        // Nowhere to accumulate
        provizio_error("provizio_accumulate_radar_point_cloud: fix_when_received->orientation is not a valid rotation");
        iterator.point_cloud_index = num_accumulated_point_clouds; // Explicit end
        return iterator;
    }

    // Find the latest accumulated cloud
    for (provizio_accumulated_radar_point_cloud *accumulated_cloud = accumulated_point_clouds,
                                                *end = accumulated_point_clouds + num_accumulated_point_clouds;
         accumulated_cloud != end && provizio_accumulated_radar_point_cloud_valid(accumulated_cloud);
         ++accumulated_cloud)
    {
        const uint32_t current_frame_index = accumulated_cloud->point_cloud.frame_index;
        const uint32_t iterators_frame_index =
            accumulated_point_clouds[iterator.point_cloud_index].point_cloud.frame_index;

        if (current_frame_index > iterators_frame_index)
        {
            ++iterator.point_cloud_index;
        }
        else if (current_frame_index != iterators_frame_index)
        {
            break;
        }
    }

    assert(point_cloud->num_points_received <= point_cloud->num_points_expected);
    if (point_cloud->num_points_received == 0)
    {
        // Nothing to accumulate. Just skip.
        iterator.point_cloud_index = num_accumulated_point_clouds; // Explicit end
        return iterator;
    }

    int8_t no_latest = provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                              num_accumulated_point_clouds);
    if (!no_latest &&
        accumulated_point_clouds[iterator.point_cloud_index].point_cloud.frame_index >= point_cloud->frame_index)
    {
        const uint32_t small_frame_index_cap = 0x0000ffff;
        const uint32_t large_frame_index_threashold = 0xffff0000;

        if (point_cloud->frame_index >= small_frame_index_cap ||
            accumulated_point_clouds[iterator.point_cloud_index].point_cloud.frame_index <=
                large_frame_index_threashold)
        {
            provizio_error(
                "provizio_accumulate_radar_point_cloud: Can't accumulate an older point cloud after a newer one");

            iterator.point_cloud_index = num_accumulated_point_clouds;
            return iterator;
        }

        // A very special case: frame indices seem to have exceeded the 0xffffffff and have been reset. Let's reset
        // accumulation to avoid complicated state-related issues.
        provizio_warning(
            "provizio_accumulate_radar_point_cloud: frame indices overflow detected - resetting accumulation");
        provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_point_clouds);
        no_latest = 1;
        iterator.point_cloud_index = 0;
    }

    assert(iterator.point_index == 0);
    if (!no_latest)
    {
        iterator.point_cloud_index = (iterator.point_cloud_index + 1) % num_accumulated_point_clouds;
    }
    else
    {
        assert(iterator.point_cloud_index == 0);
    }
    provizio_accumulated_radar_point_cloud *accumulated_cloud = &accumulated_point_clouds[iterator.point_cloud_index];

    // Copy the "header" part of things first
    memcpy(&accumulated_cloud->point_cloud, point_cloud,
           sizeof(provizio_radar_point_cloud) - offsetof(provizio_radar_point_cloud, radar_points));
    // Copy the points applying the specified filter
    accumulated_cloud->point_cloud.num_points_received = 0;
    (filter != NULL ? filter : &provizio_radar_points_accumulation_filter_copy_all)(
        &point_cloud->radar_points[0], point_cloud->num_points_received, accumulated_point_clouds,
        num_accumulated_point_clouds, &iterator, filter_user_data, &accumulated_cloud->point_cloud.radar_points[0],
        &accumulated_cloud->point_cloud.num_points_received);
    if (accumulated_cloud->point_cloud.num_points_received == 0)
    {
        provizio_warning("provizio_accumulate_radar_point_cloud: filter removed all points, which is not supported, so "
                         "accumulating the first point instead");
        assert(point_cloud->num_points_received > 0);
        accumulated_cloud->point_cloud.num_points_received = accumulated_cloud->point_cloud.num_points_expected = 1;
        memcpy(&accumulated_cloud->point_cloud.radar_points[0], &point_cloud->radar_points[0],
               sizeof(provizio_radar_point));
    }
    assert(accumulated_cloud->point_cloud.num_points_expected <= PROVIZIO__MAX_RADAR_POINTS_IN_POINT_CLOUD);
    assert(accumulated_cloud->point_cloud.num_points_received <= accumulated_cloud->point_cloud.num_points_expected);

    memcpy(&accumulated_cloud->fix_when_received, fix_when_received, sizeof(provizio_enu_fix));
    assert(provizio_accumulated_radar_point_cloud_valid(accumulated_cloud));

    return iterator;
}

size_t provizio_accumulated_radar_point_clouds_count(
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    // We check back to front as most of the time all num_accumulated_point_clouds are filled: it's quicker
    size_t invalid_count = 0;
    for (; invalid_count < num_accumulated_point_clouds; ++invalid_count)
    {
        if (provizio_accumulated_radar_point_cloud_valid(
                &accumulated_point_clouds[num_accumulated_point_clouds - invalid_count - 1]))
        {
            break;
        }
    }

    return num_accumulated_point_clouds - invalid_count;
}

size_t provizio_accumulated_radar_points_count(const provizio_accumulated_radar_point_cloud *accumulated_point_clouds,
                                               size_t num_accumulated_point_clouds)
{
    size_t num_points = 0;
    for (const provizio_accumulated_radar_point_cloud *accumulated_cloud = accumulated_point_clouds,
                                                      *end = accumulated_point_clouds + num_accumulated_point_clouds;
         accumulated_cloud != end && provizio_accumulated_radar_point_cloud_valid(accumulated_cloud);
         ++accumulated_cloud)
    {
        assert(accumulated_cloud->point_cloud.num_points_received <=
               accumulated_cloud->point_cloud.num_points_expected);
        num_points += accumulated_cloud->point_cloud.num_points_received;
    }

    return num_points;
}

int8_t provizio_accumulated_radar_point_cloud_iterator_is_end(
    const provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    const int8_t end = (iterator->point_cloud_index >= num_accumulated_point_clouds || // Explicit end
                        !provizio_accumulated_radar_point_cloud_valid(
                            &accumulated_point_clouds[iterator->point_cloud_index])) // End of accumulated clouds range
                           ? 1
                           : 0;

    // Make sure the iterator isn't broken, i.e. doesn't point to a valid cloud but out of points range
    assert(end || iterator->point_index <
                      accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received);

    return end;
}

void provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    if (num_accumulated_point_clouds == 0)
    {
        provizio_error("provizio_accumulated_radar_point_cloud_iterator_next_point_cloud: num_accumulated_point_clouds "
                       "can't be 0");
        return;
    }

    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        provizio_error(
            "provizio_accumulated_radar_point_cloud_iterator_next_point_cloud: can't go next cloud on an end iterator");
        return;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    const uint32_t current_frame_index = accumulated_point_clouds[iterator->point_cloud_index].point_cloud.frame_index;

    iterator->point_index = 0;
    // Adding num_accumulated_point_clouds makes sure '- 1' doesn't get negative, which can't be stored in size_t
    iterator->point_cloud_index =
        (num_accumulated_point_clouds + iterator->point_cloud_index - 1) % num_accumulated_point_clouds;

    const provizio_accumulated_radar_point_cloud *accumulated_cloud =
        &accumulated_point_clouds[iterator->point_cloud_index];
    if (!provizio_accumulated_radar_point_cloud_valid(accumulated_cloud) ||
        accumulated_cloud->point_cloud.frame_index >= current_frame_index)
    {
        // Complete loop over the circular buffer, i.e. we finished iterating
        iterator->point_cloud_index = num_accumulated_point_clouds;
    }
    else
    {
        assert(accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received > 0);
        assert(accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received <=
               accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_expected);
        assert(accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received >
               iterator->point_index);
    }
}

void provizio_accumulated_radar_point_cloud_iterator_next_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    if (num_accumulated_point_clouds == 0)
    {
        provizio_error(
            "provizio_accumulated_radar_point_cloud_iterator_next_point: num_accumulated_point_clouds can't be 0");
        return;
    }

    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        provizio_error(
            "provizio_accumulated_radar_point_cloud_iterator_next_point: can't go next point on an end iterator");
        return;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    ++iterator->point_index;
    if (iterator->point_index >= accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received)
    {
        iterator->point_index = 0;
        provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(iterator, accumulated_point_clouds,
                                                                         num_accumulated_point_clouds);
    }
}

const provizio_accumulated_radar_point_cloud *provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point_cloud *optional_out_transformed_point_cloud, float *optional_out_transformation_matrix)
{
    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        if (optional_out_transformed_point_cloud)
        {
            memset(optional_out_transformed_point_cloud, 0, sizeof(provizio_radar_point_cloud));
        }

        if (optional_out_transformation_matrix)
        {
            memset(optional_out_transformation_matrix, 0, provizio_transformation_matrix_components * sizeof(float));
        }

        return NULL;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    const provizio_accumulated_radar_point_cloud *accumulated_cloud =
        &accumulated_point_clouds[iterator->point_cloud_index];

    if (optional_out_transformed_point_cloud)
    {
        provizio_transform_radar_point_cloud(&accumulated_cloud->point_cloud, &accumulated_cloud->fix_when_received,
                                             current_fix, optional_out_transformed_point_cloud);
    }

    if (optional_out_transformation_matrix)
    {
        provizio_build_transformation_matrix(&accumulated_cloud->fix_when_received, current_fix,
                                             optional_out_transformation_matrix);
    }

    return accumulated_cloud;
}

const provizio_radar_point *provizio_accumulated_radar_point_cloud_iterator_get_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point *optional_out_transformed_point, float *optional_out_transformation_matrix)
{
    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        if (optional_out_transformed_point)
        {
            memset(optional_out_transformed_point, 0, sizeof(provizio_radar_point));
        }

        if (optional_out_transformation_matrix)
        {
            memset(optional_out_transformation_matrix, 0, provizio_transformation_matrix_components * sizeof(float));
        }

        return NULL;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    const provizio_accumulated_radar_point_cloud *accumulated_cloud =
        &accumulated_point_clouds[iterator->point_cloud_index];
    assert(iterator->point_index < accumulated_cloud->point_cloud.num_points_received);
    assert(accumulated_cloud->point_cloud.num_points_received <= accumulated_cloud->point_cloud.num_points_expected);

    const provizio_radar_point *point = &accumulated_cloud->point_cloud.radar_points[iterator->point_index];

    if (optional_out_transformed_point)
    {
        provizio_transform_radar_point(point, &accumulated_cloud->fix_when_received, current_fix,
                                       optional_out_transformed_point);
    }

    if (optional_out_transformation_matrix)
    {
        provizio_build_transformation_matrix(&accumulated_cloud->fix_when_received, current_fix,
                                             optional_out_transformation_matrix);
    }

    return point;
}
