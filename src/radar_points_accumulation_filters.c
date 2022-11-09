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

#include "provizio/radar_api/radar_points_accumulation_filters.h"

#include <assert.h>
#include <float.h>
#include <linmath.h>
#include <math.h>
#include <string.h>

#include "provizio/radar_api/radar_points_accumulation.h"
#include "provizio/util.h"

// Estimates ego's own velocity by assuming most points in the point cloud belong to static objects
static float provizio_estimate_radars_forward_velocity_using_velocities_histogram(const provizio_radar_point *in_points,
                                                                                  uint16_t num_in_points)
{
    if (num_in_points == 0)
    {
        return 0;
    }

    enum
    {
        histogram_bins = 50
    };

    const float min_bin_size = 0.3F; // Higher precision spreads velocities too thin (like 0-2 point per bin) making
                                     // velocity estimation imprecise
    float min_velocity = FLT_MAX;
    float max_velocity = -FLT_MAX;

    for (uint16_t i = 0; i < num_in_points; ++i)
    {
        const float velocity = in_points[i].radar_relative_radial_velocity_m_s;
        if (velocity < min_velocity)
        {
            min_velocity = velocity;
        }

        if (velocity > max_velocity)
        {
            max_velocity = velocity;
        }
    }

    // In case there velocities distribution is too narrow, we'll extend min/max so a single bin covers no more than
    // min_bin_size
    const float min_velocities_range = min_bin_size * histogram_bins;
    if (max_velocity - min_velocity < min_velocities_range)
    {
        const float half = 0.5F;
        const float average_velocity = (max_velocity + min_velocity) * half;
        min_velocity = average_velocity - min_velocities_range * half;
        max_velocity = average_velocity + min_velocities_range * half;
    }

    const float bin_size = (max_velocity - min_velocity) / histogram_bins;
    const float epsilon = 0.0001F; // Due to floats precision limits
    (void)epsilon;                 // As asserts are taken out in Release builds
    assert(bin_size >= min_bin_size - epsilon);
    uint16_t velocities_histogram[histogram_bins] = {0};
    size_t largest_bin = 0;
    uint16_t largest_bin_value = 0;
    for (uint16_t i = 0; i < num_in_points; ++i)
    {
        const float velocity = in_points[i].radar_relative_radial_velocity_m_s;
        const size_t bin =
            (size_t)lroundf((velocity - min_velocity) / bin_size) * (histogram_bins - 1) / histogram_bins;
        assert(bin < histogram_bins);
        const uint16_t bin_value = ++velocities_histogram[bin];
        if (bin_value > largest_bin_value)
        {
            largest_bin = bin;
            largest_bin_value = bin_value;
        }
    }

    const float half = 0.5F;
    return -(min_velocity + (half + (float)largest_bin) * bin_size);
}

static float provizio_estimate_radars_forward_velocity(
    const provizio_radar_point *in_points, uint16_t num_in_points,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    const provizio_accumulated_radar_point_cloud_iterator *new_iterator)
{
    const int64_t velocity_averaging_duration_max_ns = 3000000000LL; // 3 sec
    const int64_t velocity_averaging_duration_min_ns = 1000000000LL; // 1 sec
    const float min_move_distance_to_detect_orientation_m = 1.5F;

    float total_distance_m = 0.0F;
    int64_t total_duration_ns = 0;
    if (new_iterator != NULL)
    {
        for (provizio_accumulated_radar_point_cloud_iterator iterator = *new_iterator, next_iterator = *new_iterator;
             total_duration_ns < velocity_averaging_duration_max_ns; iterator = next_iterator)
        {
            provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(&next_iterator, accumulated_point_clouds,
                                                                             num_accumulated_point_clouds);

            const provizio_accumulated_radar_point_cloud *current_cloud =
                provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
                    &iterator, NULL, accumulated_point_clouds, num_accumulated_point_clouds, NULL, NULL);
            const provizio_accumulated_radar_point_cloud *next_cloud =
                provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
                    &next_iterator, NULL, accumulated_point_clouds, num_accumulated_point_clouds, NULL, NULL);

            if (next_cloud == NULL)
            {
                break;
            }
            assert(current_cloud != NULL); // Should never happen given the check above

            assert(current_cloud->point_cloud.timestamp >=
                   next_cloud->point_cloud.timestamp); // Next must always be older

            total_duration_ns += (int64_t)(current_cloud->point_cloud.timestamp - next_cloud->point_cloud.timestamp);
            total_distance_m += provizio_enu_distance(&current_cloud->fix_when_received.position,
                                                      &next_cloud->fix_when_received.position);
        }

        if (total_duration_ns >= velocity_averaging_duration_min_ns)
        {
            // Ego's forward velocity is now easy to calculate
            const float ego_forward_velocity = total_distance_m / provizio_nanoseconds_to_seconds(total_duration_ns);

            // Now we need to find the radar's forward velocity, given the radar doesn't have to be aligned with Ego
            // Find the latest position and orientation of the radar
            provizio_accumulated_radar_point_cloud_iterator iterator = *new_iterator;
            assert(provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
                       &iterator, NULL, accumulated_point_clouds, num_accumulated_point_clouds, NULL, NULL) != NULL);
            const provizio_enu_fix current_fix =
                provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
                    &iterator, NULL, accumulated_point_clouds, num_accumulated_point_clouds, NULL, NULL)
                    ->fix_when_received;

            // Find a recent previous position far away enough to detect ego's movement direction despite of GNSS
            // imprecision
            provizio_enu_position previous_position = current_fix.position;
            float distance = 0.0F;
            do
            {
                provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(&iterator, accumulated_point_clouds,
                                                                                 num_accumulated_point_clouds);
                const provizio_accumulated_radar_point_cloud *point_cloud =
                    provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
                        &iterator, NULL, accumulated_point_clouds, num_accumulated_point_clouds, NULL, NULL);
                if (point_cloud == NULL)
                {
                    break;
                }

                previous_position = point_cloud->fix_when_received.position;
                distance = provizio_enu_distance(&current_fix.position, &previous_position);
            } while (distance < min_move_distance_to_detect_orientation_m);

            if (distance >= min_move_distance_to_detect_orientation_m)
            {
                const float ego_direction_north = current_fix.position.north_meters - previous_position.north_meters;
                const float ego_direction_east = current_fix.position.east_meters - previous_position.east_meters;
                const float ego_direction_up = current_fix.position.up_meters - previous_position.up_meters;

                provizio_quaternion ego_orientation;
                if (ego_direction_north * ego_direction_north + ego_direction_east * ego_direction_east > 0)
                {
                    const float yaw = atan2f(ego_direction_north, ego_direction_east);
                    const float pitch =
                        ego_direction_up != 0.0F ? (atan2f(ego_direction_east, -ego_direction_up)) : 0.0F;
                    provizio_quaternion_set_euler_angles(0.0F, pitch, yaw, &ego_orientation);
                }
                else
                {
                    provizio_warning(
                        "provizio_estimate_radars_forward_velocity: Ego moving straight up or down... Wait, really?!");
                    provizio_quaternion_set_euler_angles(0.0F, (float)M_PI_2 * (ego_direction_up > 0 ? -1.0F : 1.0F),
                                                         0.0F, &ego_orientation);
                }

                assert(provizio_quaternion_is_valid_rotation(&current_fix.orientation));
                assert(provizio_quaternion_is_valid_rotation(&ego_orientation));

                vec3 enu_velocity_vec3;
                vec3 ego_relative_velocity_vec3 = {ego_forward_velocity, 0.0F, 0.0F};
                quat ego_orientation_quat = {ego_orientation.x, ego_orientation.y, ego_orientation.z,
                                             ego_orientation.w};
                quat_mul_vec3(enu_velocity_vec3, ego_orientation_quat, ego_relative_velocity_vec3);
                quat radar_orientation_inv_quat = {-current_fix.orientation.x, -current_fix.orientation.y,
                                                   -current_fix.orientation.z, current_fix.orientation.w};
                vec3 radar_velocity_vec3;
                quat_mul_vec3(radar_velocity_vec3, radar_orientation_inv_quat, enu_velocity_vec3);

                return radar_velocity_vec3[0]; // Return only the forward component of ego velocity, i.e. its
                                               // radar-forward projection length
            }
        }
    }

    // We haven't accumulated enough or moving too slow, let's estimate velocity using histogram
    return provizio_estimate_radars_forward_velocity_using_velocities_histogram(in_points, num_in_points);
}

void provizio_radar_points_accumulation_filter_copy_all(
    const provizio_radar_point *in_points, uint16_t num_in_points,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    const provizio_accumulated_radar_point_cloud_iterator *new_iterator, void *user_data,
    provizio_radar_point *out_points, uint16_t *num_out_points)
{
    // Following arguments are not used by this simplest "filter"
    (void)accumulated_point_clouds;
    (void)num_accumulated_point_clouds;
    (void)new_iterator;
    (void)user_data;

    memcpy(out_points, in_points, sizeof(provizio_radar_point) * num_in_points);
    *num_out_points = num_in_points;
}

void provizio_radar_points_accumulation_filter_static(
    const provizio_radar_point *in_points, uint16_t num_in_points,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    const provizio_accumulated_radar_point_cloud_iterator *new_iterator, void *user_data,
    provizio_radar_point *out_points, uint16_t *num_out_points)
{
    (void)user_data;

    const float dynamic_velocity_threashold_m_s = 1.5F;
    const float radars_forward_velocity_m_s = provizio_estimate_radars_forward_velocity(
        in_points, num_in_points, accumulated_point_clouds, num_accumulated_point_clouds, new_iterator);

    uint16_t num_filtered_points = 0;
    for (const provizio_radar_point *point = in_points, *end = in_points + num_in_points; point != end; ++point)
    {
        // TODO(iivanov): uncomment this fix and update the unit tests
        // if (fabsf(point->radar_relative_radial_velocity_m_s + radars_forward_velocity_m_s *
        // cosf(atan2f(point->y_meters, point->x_meters) * -1)) < dynamic_velocity_threashold_m_s)
        if (fabsf(point->radar_relative_radial_velocity_m_s + radars_forward_velocity_m_s) <
            dynamic_velocity_threashold_m_s)
        {
            // Static point, let's accumulate it
            out_points[num_filtered_points++] = *point;
        }
    }
    *num_out_points = num_filtered_points;
}
