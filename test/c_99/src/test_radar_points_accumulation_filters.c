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
#include "provizio/radar_api/radar_points_accumulation_filters.h"

#include <stdlib.h>
#include <string.h>

#include "unity/unity.h"

void test_provizio_radar_points_accumulation_filter_copy_all_empty(void)
{
    provizio_radar_point in_points = {0};
    provizio_radar_point out_points = {0};
    uint16_t num_out_points = UINT16_MAX;
    provizio_radar_points_accumulation_filter_copy_all(&in_points, 0, NULL, 0, NULL, NULL, &out_points,
                                                       &num_out_points);
    TEST_ASSERT_EQUAL_UINT16(0, num_out_points);
}

void test_provizio_radar_points_accumulation_filter_copy_all(void)
{
    provizio_radar_point in_points[5]; // NOLINT
    const uint16_t num_in_points = (uint16_t)(sizeof(in_points) / sizeof(in_points[0]));
    for (uint16_t i = 0; i < num_in_points; ++i)
    {
        in_points[i].x_meters = 1.0F * i;                            // NOLINT
        in_points[i].y_meters = 2.0F * i;                            // NOLINT
        in_points[i].z_meters = 3.0F * i;                            // NOLINT
        in_points[i].radar_relative_radial_velocity_m_s = 4.0F * i;  // NOLINT
        in_points[i].signal_to_noise_ratio = 5.0F * i;               // NOLINT
        in_points[i].ground_relative_radial_velocity_m_s = 4.0F * i; // NOLINT
    }

    provizio_radar_point out_points[sizeof(in_points) / sizeof(in_points[0])];
    memset(&out_points, 0, sizeof(out_points));
    uint16_t num_out_points = 0;

    provizio_radar_points_accumulation_filter_copy_all(in_points, num_in_points, NULL, 0, NULL, NULL, out_points,
                                                       &num_out_points);

    TEST_ASSERT_EQUAL_UINT16(num_in_points, num_out_points);
    TEST_ASSERT_EQUAL(0, memcmp(&in_points, &out_points, sizeof(in_points))); // NOLINT
}

void test_provizio_radar_points_accumulation_filter_static_empty(void)
{
    provizio_radar_point in_points = {0};
    provizio_radar_point out_points = {0};
    uint16_t num_out_points = UINT16_MAX;
    provizio_radar_points_accumulation_filter_static(&in_points, 0, NULL, 0, NULL, NULL, &out_points, &num_out_points);
    TEST_ASSERT_EQUAL_UINT16(0, num_out_points);
}

void test_provizio_radar_points_accumulation_filter_static_stationary_ego(void)
{
    const float default_signal_to_noise_ratio = 10.0F;
    const size_t num_accumulated_clouds = 20;
    const uint64_t time_between_frames_ns = 100000000ULL; // 0.1s in nanoseconds

    provizio_enu_fix the_fix;
    memset(&the_fix, 0, sizeof(provizio_enu_fix));
    provizio_quaternion_set_identity(&the_fix.orientation);

    provizio_radar_point_cloud *point_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));
    point_cloud->num_points_received = point_cloud->num_points_expected = 3;

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds =
        malloc(num_accumulated_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    memset(&iterator, 0, sizeof(iterator));

    // Assumes ego is stationary. Every frame there are 3 points, 2 static and one moving, with same
    // ego(=radar)-relative coordinates: static (0, 1, 0) and (0, -1, 0), moving (1, 0, 0)
    provizio_radar_point out_points[3];
    memset(out_points, 0, sizeof(out_points));
    uint16_t num_out_points = 0;

    for (size_t i = 0; i < num_accumulated_clouds; ++i)
    {
        // A static point at EgoRelative(0, 1, 0)
        point_cloud->radar_points[0].x_meters = 0.0F;
        point_cloud->radar_points[0].y_meters = 1.0F;
        point_cloud->radar_points[0].radar_relative_radial_velocity_m_s = 0.1F; // NOLINT
        point_cloud->radar_points[0].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A static point at EgoRelative(0, -1, 0)
        point_cloud->radar_points[1].x_meters = 0.0F;
        point_cloud->radar_points[1].y_meters = -1.0F;
        point_cloud->radar_points[1].radar_relative_radial_velocity_m_s = -0.1F; // NOLINT
        point_cloud->radar_points[1].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A moving point at EgoRelative(1, 0, 0)
        point_cloud->radar_points[2].x_meters = 1.0F;
        point_cloud->radar_points[2].y_meters = 0.0F;
        point_cloud->radar_points[2].radar_relative_radial_velocity_m_s = 10.0F; // NOLINT
        point_cloud->radar_points[2].signal_to_noise_ratio = default_signal_to_noise_ratio;

        iterator = provizio_accumulate_radar_point_cloud(point_cloud, &the_fix, accumulated_point_clouds,
                                                         num_accumulated_clouds, NULL, NULL);
        provizio_radar_points_accumulation_filter_static(point_cloud->radar_points, point_cloud->num_points_received,
                                                         accumulated_point_clouds, num_accumulated_clouds, &iterator,
                                                         NULL, out_points, &num_out_points);
        TEST_ASSERT_EQUAL_UINT16(2, num_out_points);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].x_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(1.0F, out_points[0].y_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].z_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.1F, out_points[0].radar_relative_radial_velocity_m_s);             // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[0].signal_to_noise_ratio); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].x_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-1.0F, out_points[1].y_meters);                                      // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].z_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-0.1F, out_points[1].radar_relative_radial_velocity_m_s);            // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[1].signal_to_noise_ratio); // NOLINT

        ++point_cloud->frame_index;
        point_cloud->timestamp += time_between_frames_ns;
    }

    free(accumulated_point_clouds);
    free(point_cloud);
}

void test_provizio_radar_points_accumulation_filter_static_forward_radar(void)
{
    const float default_signal_to_noise_ratio = 10.0F;
    const size_t num_accumulated_clouds = 20;
    const uint64_t time_between_frames_ns = 100000000ULL; // 0.1s in nanoseconds
    const float time_between_frames_s = 0.1F;

    // Ego starts at ENU(0, 0, 0) going East. The radar is in Ego(0, 0, 0) forward-looking (i.e. = ego).
    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(provizio_enu_fix));
    provizio_quaternion_set_identity(&fix_when_received.orientation);

    provizio_radar_point_cloud *point_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));
    point_cloud->num_points_received = point_cloud->num_points_expected = 3;

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds =
        malloc(num_accumulated_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    memset(&iterator, 0, sizeof(iterator));

    // Assumes ego moves forward starting at ENU(0, 0, 0) with starting velocity of 1 m/s and accelerating at
    // 5m/s^2. Every frame there are 3 points, 2 static and one moving, with same radar-relative coordinates: static
    // (0, 1, 0) and (0, -1, 0), moving (1, 0, 0).
    float ego_velocity_m_s = 1.0F;
    const float acceleration_per_frame = 0.5F; // As time_between_frames = 0.1s
    provizio_radar_point out_points[3];
    memset(out_points, 0, sizeof(out_points));
    uint16_t num_out_points = 0;

    for (size_t i = 0; i < num_accumulated_clouds; ++i)
    {
        // As soon as there is enough history of ego positions readings, i.e. 1 second, there is an up to 3 seconds
        // velocity averaging to avoid issues caused by GNSS imprecision
        const float estimated_ego_velocity =
            (i < 10) ? ego_velocity_m_s
                     : (fix_when_received.position.east_meters /
                        ((float)(point_cloud->timestamp / 1000000ULL) / // NOLINT
                         1000.0F)); // 2-step division as floats can't accurately store 10^9 values

        // A static point at EgoRelative(0, 1, 0)
        point_cloud->radar_points[0].x_meters = 0.0F;
        point_cloud->radar_points[0].y_meters = 1.0F;
        point_cloud->radar_points[0].radar_relative_radial_velocity_m_s = -estimated_ego_velocity + 0.1F; // NOLINT
        point_cloud->radar_points[0].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A static point at EgoRelative(0, -1, 0)
        point_cloud->radar_points[1].x_meters = 0.0F;
        point_cloud->radar_points[1].y_meters = -1.0F;
        point_cloud->radar_points[1].radar_relative_radial_velocity_m_s = -estimated_ego_velocity - 0.1F; // NOLINT
        point_cloud->radar_points[1].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A moving point at EgoRelative(1, 0, 0)
        point_cloud->radar_points[2].x_meters = 1.0F;
        point_cloud->radar_points[2].y_meters = 0.0F;
        point_cloud->radar_points[2].radar_relative_radial_velocity_m_s = -estimated_ego_velocity + 10.0F; // NOLINT
        point_cloud->radar_points[2].signal_to_noise_ratio = default_signal_to_noise_ratio;

        iterator = provizio_accumulate_radar_point_cloud(point_cloud, &fix_when_received, accumulated_point_clouds,
                                                         num_accumulated_clouds, NULL, NULL);
        provizio_radar_points_accumulation_filter_static(point_cloud->radar_points, point_cloud->num_points_received,
                                                         accumulated_point_clouds, num_accumulated_clouds, &iterator,
                                                         NULL, out_points, &num_out_points);
        TEST_ASSERT_EQUAL_UINT16(2, num_out_points);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].x_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(1.0F, out_points[0].y_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].z_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity + 0.1F, // NOLINT
                                out_points[0].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[0].signal_to_noise_ratio); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].x_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-1.0F, out_points[1].y_meters);                                      // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].z_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity - 0.1F,                                      // NOLINT
                                out_points[1].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[1].signal_to_noise_ratio); // NOLINT

        fix_when_received.position.east_meters += ego_velocity_m_s * time_between_frames_s;
        ego_velocity_m_s += acceleration_per_frame;
        ++point_cloud->frame_index;
        point_cloud->timestamp += time_between_frames_ns;
    }

    free(accumulated_point_clouds);
    free(point_cloud);
}

void test_provizio_radar_points_accumulation_filter_static_rear_corner_radar(void)
{
    const float default_signal_to_noise_ratio = 10.0F;
    const size_t num_accumulated_clouds = 20;
    const uint64_t time_between_frames_ns = 100000000ULL; // 0.1s in nanoseconds
    const float time_between_frames_s = 0.1F;

    // // Ego starts at ENU(0, 0, 0) going East. The radar is at the left rear corner: in Ego(-1, 1, 0) looking 135
    // degrees from forward.
    provizio_enu_fix ego_fix, radar_fix; // NOLINT
    memset(&ego_fix, 0, sizeof(provizio_enu_fix));
    memset(&radar_fix, 0, sizeof(provizio_enu_fix));
    radar_fix.position.east_meters = -1.0F;
    radar_fix.position.north_meters = 1.0F;
    provizio_quaternion_set_identity(&ego_fix.orientation);
    provizio_quaternion_set_euler_angles(0, 0, (float)M_PI_4 * 3.0F, &radar_fix.orientation); // NOLINT

    provizio_radar_point_cloud *point_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));
    point_cloud->num_points_received = point_cloud->num_points_expected = 3;

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds =
        malloc(num_accumulated_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    memset(&iterator, 0, sizeof(iterator));

    // Assumes ego moves forward starting at ENU(0, 0, 0) with starting velocity of 1 m/s and accelerating at
    // 5m/s^2. Every frame there are 3 points, 2 static and one moving, with same ego-relative coordinates: static
    // (0, 1, 0) and (0, -1, 0), moving (1, 0, 0) => radar-relative (-sqrt(2)/2, -sqrt(2)/2, 0), (-sqrt(2)*3/2,
    // sqrt(2)/2, 0), (-sqrt(2)*3/2, -sqrt(2)/2, 0).
    float ego_velocity_m_s = 1.0F;
    const float acceleration_per_frame = 0.5F; // As time_between_frames = 0.1s
    provizio_radar_point out_points[3];
    memset(out_points, 0, sizeof(out_points));
    uint16_t num_out_points = 0;

    for (size_t i = 0; i < num_accumulated_clouds; ++i)
    {
        // As soon as there is enough history of ego positions readings, i.e. 1 second, there is an up to 3 seconds
        // velocity averaging to avoid issues caused by GNSS imprecision
        const float estimated_ego_velocity =
            (i < 10) ? ego_velocity_m_s
                     : (ego_fix.position.east_meters /
                        ((float)(point_cloud->timestamp / 1000000ULL) / // NOLINT
                         1000.0F)); // 2-step division as floats can't accurately store 10^9 values
        const float estimated_ego_velocity_radar_projection = -(float)M_SQRT2 / 2.0F * estimated_ego_velocity;

        // A static point at EgoRelative(0, 1, 0) -> RadarRelative(-sqrt(2)/2, -sqrt(2)/2, 0)
        point_cloud->radar_points[0].x_meters = -(float)M_SQRT2 / 2.0F; // NOLINT
        point_cloud->radar_points[0].y_meters = -(float)M_SQRT2 / 2.0F; // NOLINT
        point_cloud->radar_points[0].radar_relative_radial_velocity_m_s =
            -estimated_ego_velocity_radar_projection + 0.1F; // NOLINT
        point_cloud->radar_points[0].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A static point at EgoRelative(0, -1, 0) -> RadarRelative(-sqrt(2)*3/2, sqrt(2)/2, 0)
        point_cloud->radar_points[1].x_meters = -(float)M_SQRT2 * 3.0F / 2.0F; // NOLINT
        point_cloud->radar_points[1].y_meters = (float)M_SQRT2 / 2.0F;         // NOLINT
        point_cloud->radar_points[1].radar_relative_radial_velocity_m_s =
            -estimated_ego_velocity_radar_projection - 0.1F; // NOLINT
        point_cloud->radar_points[1].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A moving point at EgoRelative(1, 0, 0) -> RadarRelative(-sqrt(2)*3/2, -sqrt(2)/2, 0)
        point_cloud->radar_points[2].x_meters = -(float)M_SQRT2 * 3.0F / 2.0F; // NOLINT
        point_cloud->radar_points[2].y_meters = -(float)M_SQRT2 / 2.0F;        // NOLINT
        point_cloud->radar_points[2].radar_relative_radial_velocity_m_s =
            -estimated_ego_velocity_radar_projection + 10.0F; // NOLINT
        point_cloud->radar_points[2].signal_to_noise_ratio = default_signal_to_noise_ratio;

        iterator = provizio_accumulate_radar_point_cloud(point_cloud, &radar_fix, accumulated_point_clouds,
                                                         num_accumulated_clouds, NULL, NULL);
        provizio_radar_points_accumulation_filter_static(point_cloud->radar_points, point_cloud->num_points_received,
                                                         accumulated_point_clouds, num_accumulated_clouds, &iterator,
                                                         NULL, out_points, &num_out_points);
        TEST_ASSERT_EQUAL_UINT16(2, num_out_points);
        TEST_ASSERT_EQUAL_FLOAT(-(float)M_SQRT2 / 2.0F, out_points[0].x_meters); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-(float)M_SQRT2 / 2.0F, out_points[0].y_meters); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].z_meters);                   // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity_radar_projection + 0.1F, // NOLINT
                                out_points[0].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[0].signal_to_noise_ratio); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-(float)M_SQRT2 * 3.0F / 2.0F, out_points[1].x_meters);              // NOLINT
        TEST_ASSERT_EQUAL_FLOAT((float)M_SQRT2 / 2.0F, out_points[1].y_meters);                      // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].z_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity_radar_projection - 0.1F,                     // NOLINT
                                out_points[1].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[1].signal_to_noise_ratio); // NOLINT

        ego_fix.position.east_meters += ego_velocity_m_s * time_between_frames_s;
        radar_fix.position.east_meters = ego_fix.position.east_meters - 1.0F;
        ego_velocity_m_s += acceleration_per_frame;
        ++point_cloud->frame_index;
        point_cloud->timestamp += time_between_frames_ns;
    }

    free(accumulated_point_clouds);
    free(point_cloud);
}

void test_provizio_radar_points_accumulation_filter_static_move_up(void)
{
    const float default_signal_to_noise_ratio = 10.0F;
    const size_t num_accumulated_clouds = 20;
    const uint64_t time_between_frames_ns = 100000000ULL; // 0.1s in nanoseconds
    const float time_between_frames_s = 0.1F;

    // Ego starts at ENU(0, 0, 0) going up. The radar is in Ego(0, 0, 0) forward-looking (i.e. = ego).
    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(provizio_enu_fix));
    provizio_quaternion_set_euler_angles(0.0F, (float)-M_PI_2, 0.0F, &fix_when_received.orientation);

    provizio_radar_point_cloud *point_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));
    point_cloud->num_points_received = point_cloud->num_points_expected = 3;

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds =
        malloc(num_accumulated_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    memset(&iterator, 0, sizeof(iterator));

    // Assumes ego moves up starting at ENU(0, 0, 0) with starting velocity of 1 m/s and accelerating at
    // 5m/s^2. Every frame there are 3 points, 2 static and one moving, with same radar-relative coordinates: static
    // (0, 1, 0) and (0, -1, 0), moving (1, 0, 0).
    float ego_velocity_m_s = 1.0F;
    const float acceleration_per_frame = 0.5F; // As time_between_frames = 0.1s
    provizio_radar_point out_points[3];
    memset(out_points, 0, sizeof(out_points));
    uint16_t num_out_points = 0;

    for (size_t i = 0; i < num_accumulated_clouds; ++i)
    {
        // As soon as there is enough history of ego positions readings, i.e. 1 second, there is an up to 3 seconds
        // velocity averaging to avoid issues caused by GNSS imprecision
        const float estimated_ego_velocity =
            (i < 10) ? ego_velocity_m_s
                     : (fix_when_received.position.up_meters /
                        ((float)(point_cloud->timestamp / 1000000ULL) / // NOLINT
                         1000.0F)); // 2-step division as floats can't accurately store 10^9 values

        // A static point at EgoRelative(0, 1, 0)
        point_cloud->radar_points[0].x_meters = 0.0F;
        point_cloud->radar_points[0].y_meters = 1.0F;
        point_cloud->radar_points[0].radar_relative_radial_velocity_m_s = -estimated_ego_velocity + 0.1F; // NOLINT
        point_cloud->radar_points[0].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A static point at EgoRelative(0, -1, 0)
        point_cloud->radar_points[1].x_meters = 0.0F;
        point_cloud->radar_points[1].y_meters = -1.0F;
        point_cloud->radar_points[1].radar_relative_radial_velocity_m_s = -estimated_ego_velocity - 0.1F; // NOLINT
        point_cloud->radar_points[1].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A moving point at EgoRelative(1, 0, 0)
        point_cloud->radar_points[2].x_meters = 1.0F;
        point_cloud->radar_points[2].y_meters = 0.0F;
        point_cloud->radar_points[2].radar_relative_radial_velocity_m_s = -estimated_ego_velocity + 10.0F; // NOLINT
        point_cloud->radar_points[2].signal_to_noise_ratio = default_signal_to_noise_ratio;

        iterator = provizio_accumulate_radar_point_cloud(point_cloud, &fix_when_received, accumulated_point_clouds,
                                                         num_accumulated_clouds, NULL, NULL);
        provizio_radar_points_accumulation_filter_static(point_cloud->radar_points, point_cloud->num_points_received,
                                                         accumulated_point_clouds, num_accumulated_clouds, &iterator,
                                                         NULL, out_points, &num_out_points);
        TEST_ASSERT_EQUAL_UINT16(2, num_out_points);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].x_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(1.0F, out_points[0].y_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].z_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity + 0.1F, // NOLINT
                                out_points[0].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[0].signal_to_noise_ratio); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].x_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-1.0F, out_points[1].y_meters);                                      // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].z_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity - 0.1F,                                      // NOLINT
                                out_points[1].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[1].signal_to_noise_ratio); // NOLINT

        fix_when_received.position.up_meters += ego_velocity_m_s * time_between_frames_s;
        ego_velocity_m_s += acceleration_per_frame;
        ++point_cloud->frame_index;
        point_cloud->timestamp += time_between_frames_ns;
    }

    free(accumulated_point_clouds);
    free(point_cloud);
}

void test_provizio_radar_points_accumulation_filter_static_move_down(void)
{
    const float default_signal_to_noise_ratio = 10.0F;
    const size_t num_accumulated_clouds = 20;
    const uint64_t time_between_frames_ns = 100000000ULL; // 0.1s in nanoseconds
    const float time_between_frames_s = 0.1F;

    // Ego starts at ENU(0, 0, 0) going down. The radar is in Ego(0, 0, 0) forward-looking (i.e. = ego).
    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(provizio_enu_fix));
    provizio_quaternion_set_euler_angles(0.0F, (float)M_PI_2, 0.0F, &fix_when_received.orientation);

    provizio_radar_point_cloud *point_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));
    point_cloud->num_points_received = point_cloud->num_points_expected = 3;

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds =
        malloc(num_accumulated_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    memset(&iterator, 0, sizeof(iterator));

    // Assumes ego moves down starting at ENU(0, 0, 0) with starting velocity of 1 m/s and accelerating at
    // 5m/s^2. Every frame there are 3 points, 2 static and one moving, with same radar-relative coordinates: static
    // (0, 1, 0) and (0, -1, 0), moving (1, 0, 0).
    float ego_velocity_m_s = 1.0F;
    const float acceleration_per_frame = 0.5F; // As time_between_frames = 0.1s
    provizio_radar_point out_points[3];
    memset(out_points, 0, sizeof(out_points));
    uint16_t num_out_points = 0;

    for (size_t i = 0; i < num_accumulated_clouds; ++i)
    {
        // As soon as there is enough history of ego positions readings, i.e. 1 second, there is an up to 3 seconds
        // velocity averaging to avoid issues caused by GNSS imprecision
        const float estimated_ego_velocity =
            (i < 10) ? ego_velocity_m_s
                     : (-fix_when_received.position.up_meters /
                        ((float)(point_cloud->timestamp / 1000000ULL) / // NOLINT
                         1000.0F)); // 2-step division as floats can't accurately store 10^9 values

        // A static point at EgoRelative(0, 1, 0)
        point_cloud->radar_points[0].x_meters = 0.0F;
        point_cloud->radar_points[0].y_meters = 1.0F;
        point_cloud->radar_points[0].radar_relative_radial_velocity_m_s = -estimated_ego_velocity + 0.1F; // NOLINT
        point_cloud->radar_points[0].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A static point at EgoRelative(0, -1, 0)
        point_cloud->radar_points[1].x_meters = 0.0F;
        point_cloud->radar_points[1].y_meters = -1.0F;
        point_cloud->radar_points[1].radar_relative_radial_velocity_m_s = -estimated_ego_velocity - 0.1F; // NOLINT
        point_cloud->radar_points[1].signal_to_noise_ratio = default_signal_to_noise_ratio;
        // A moving point at EgoRelative(1, 0, 0)
        point_cloud->radar_points[2].x_meters = 1.0F;
        point_cloud->radar_points[2].y_meters = 0.0F;
        point_cloud->radar_points[2].radar_relative_radial_velocity_m_s = -estimated_ego_velocity + 10.0F; // NOLINT
        point_cloud->radar_points[2].signal_to_noise_ratio = default_signal_to_noise_ratio;

        iterator = provizio_accumulate_radar_point_cloud(point_cloud, &fix_when_received, accumulated_point_clouds,
                                                         num_accumulated_clouds, NULL, NULL);
        provizio_radar_points_accumulation_filter_static(point_cloud->radar_points, point_cloud->num_points_received,
                                                         accumulated_point_clouds, num_accumulated_clouds, &iterator,
                                                         NULL, out_points, &num_out_points);
        TEST_ASSERT_EQUAL_UINT16(2, num_out_points);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].x_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(1.0F, out_points[0].y_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[0].z_meters);  // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity + 0.1F, // NOLINT
                                out_points[0].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[0].signal_to_noise_ratio); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].x_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-1.0F, out_points[1].y_meters);                                      // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, out_points[1].z_meters);                                       // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-estimated_ego_velocity - 0.1F,                                      // NOLINT
                                out_points[1].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(default_signal_to_noise_ratio, out_points[1].signal_to_noise_ratio); // NOLINT

        fix_when_received.position.up_meters -= ego_velocity_m_s * time_between_frames_s;
        ego_velocity_m_s += acceleration_per_frame;
        ++point_cloud->frame_index;
        point_cloud->timestamp += time_between_frames_ns;
    }

    free(accumulated_point_clouds);
    free(point_cloud);
}

int provizio_run_test_radar_points_accumulation_filters(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_provizio_radar_points_accumulation_filter_copy_all_empty);
    RUN_TEST(test_provizio_radar_points_accumulation_filter_copy_all);
    RUN_TEST(test_provizio_radar_points_accumulation_filter_static_empty);
    RUN_TEST(test_provizio_radar_points_accumulation_filter_static_stationary_ego);
    RUN_TEST(test_provizio_radar_points_accumulation_filter_static_forward_radar);
    RUN_TEST(test_provizio_radar_points_accumulation_filter_static_rear_corner_radar);
    RUN_TEST(test_provizio_radar_points_accumulation_filter_static_move_up);
    RUN_TEST(test_provizio_radar_points_accumulation_filter_static_move_down);

    return UNITY_END();
}
