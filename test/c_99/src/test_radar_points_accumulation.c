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

#include "unity/unity.h"

#include <stdlib.h>
#include <string.h>

#include "provizio/radar_api/radar_points_accumulation.h"

static void test_accumulate_radar_point_clouds_move(void)
{
    enum
    {
        num_accumulated_point_clouds = 3
    };
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));

    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    TEST_ASSERT_EQUAL_size_t(0, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        0, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));

    provizio_radar_point_cloud *transformed_accumulated_point_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(transformed_accumulated_point_cloud, 0, sizeof(provizio_radar_point_cloud));

    provizio_radar_point transformed_accumulated_point;
    memset(&transformed_accumulated_point, 0, sizeof(provizio_radar_point));

    enum
    {
        num_point_clouds = 2
    };
    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_point_clouds * sizeof(provizio_radar_point_cloud));
    provizio_enu_fix fix_when_received[num_point_clouds + 1];
    memset(fix_when_received, 0, sizeof(fix_when_received));

    // Point Cloud 0
    point_clouds[0].frame_index = 0;
    point_clouds[0].num_points_expected = point_clouds[0].num_points_received = 1;
    point_clouds[0].radar_points[0].x_meters = 1.0F;
    point_clouds[0].radar_points[0].y_meters = 2.0F;
    point_clouds[0].radar_points[0].z_meters = 3.0F;
    point_clouds[0].radar_points[0].velocity_m_s = 4.0F;
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 2;
    point_clouds[1].radar_points[0].x_meters = 10.0F;
    point_clouds[1].radar_points[0].y_meters = 20.0F;
    point_clouds[1].radar_points[0].z_meters = 30.0F;
    point_clouds[1].radar_points[0].velocity_m_s = 40.0F;
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 50.0F;
    point_clouds[1].radar_points[1].x_meters = 100.0F;
    point_clouds[1].radar_points[1].y_meters = 200.0F;
    point_clouds[1].radar_points[1].z_meters = 300.0F;
    point_clouds[1].radar_points[1].velocity_m_s = 400.0F;
    point_clouds[1].radar_points[1].signal_to_noise_ratio = 500.0F;

    // Fix 0
    fix_when_received[0].position.east_meters = 1.0F;
    fix_when_received[0].position.north_meters = 2.0F;
    fix_when_received[0].position.up_meters = 3.0F;
    provizio_quaternion_set_identity(&fix_when_received[0].orientation);

    // Fix 1
    fix_when_received[1].position.east_meters = 6.0F;
    fix_when_received[1].position.north_meters = 5.0F;
    fix_when_received[1].position.up_meters = 4.0F;
    provizio_quaternion_set_identity(&fix_when_received[1].orientation);

    // Fix 2
    fix_when_received[2].position.east_meters = 1000.0F;
    fix_when_received[2].position.north_meters = 2000.0F;
    fix_when_received[2].position.up_meters = 3000.0F;
    provizio_quaternion_set_identity(&fix_when_received[2].orientation);

    // Accumulate the first cloud
    provizio_accumulated_radar_point_cloud_iterator iterator = provizio_accumulate_radar_point_cloud(
        &point_clouds[0], &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator_was = iterator;

    TEST_ASSERT_EQUAL_size_t(1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));

    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));

    // We got our point cloud back, as we haven't moved yet
    const provizio_accumulated_radar_point_cloud *accumulated_point_cloud =
        provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
            &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
            transformed_accumulated_point_cloud);
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(&accumulated_point_cloud->point_cloud, &point_clouds[0], sizeof(provizio_radar_point_cloud)));
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(transformed_accumulated_point_cloud, &point_clouds[0], sizeof(provizio_radar_point_cloud)));

    // Same no changes with the only point in the point cloud
    const provizio_radar_point *accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_INT32(0,
                            memcmp(accumulated_point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point)));
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(&transformed_accumulated_point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point)));

    // No more points when iterating one point forward
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            (size_t)num_accumulated_point_clouds));
    // ... or one point cloud forward from the original iterator
    iterator = iterator_was;
    provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(&iterator, accumulated_point_clouds,
                                                                     (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            (size_t)num_accumulated_point_clouds));

    // Accumulate the second cloud
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    iterator_was = iterator;

    TEST_ASSERT_EQUAL_size_t(2, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        3, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));

    // We got our last point cloud back, as we haven't moved yet since it's been received
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    accumulated_point_cloud = provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        transformed_accumulated_point_cloud);
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(&accumulated_point_cloud->point_cloud, &point_clouds[1], sizeof(provizio_radar_point_cloud)));
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(transformed_accumulated_point_cloud, &point_clouds[1], sizeof(provizio_radar_point_cloud)));

    // Same no changes with both points in the last point cloud
    accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_INT32(0,
                            memcmp(accumulated_point, &point_clouds[1].radar_points[0], sizeof(provizio_radar_point)));
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(&transformed_accumulated_point, &point_clouds[1].radar_points[0], sizeof(provizio_radar_point)));
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_INT32(0,
                            memcmp(accumulated_point, &point_clouds[1].radar_points[1], sizeof(provizio_radar_point)));
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(&transformed_accumulated_point, &point_clouds[1].radar_points[1], sizeof(provizio_radar_point)));

    // We moved since the older point cloud was received, so its position has moved
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    accumulated_point_cloud = provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        transformed_accumulated_point_cloud);
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(&accumulated_point_cloud->point_cloud, &point_clouds[0], sizeof(provizio_radar_point_cloud)));
    TEST_ASSERT_NOT_EQUAL_INT32(
        0, memcmp(transformed_accumulated_point_cloud, &point_clouds[0], sizeof(provizio_radar_point_cloud)));
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].x_meters - fix_when_received[1].position.east_meters +
                                fix_when_received[0].position.east_meters,
                            transformed_accumulated_point_cloud->radar_points[0].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].y_meters - fix_when_received[1].position.north_meters +
                                fix_when_received[0].position.north_meters,
                            transformed_accumulated_point_cloud->radar_points[0].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].z_meters - fix_when_received[1].position.up_meters +
                                fix_when_received[0].position.up_meters,
                            transformed_accumulated_point_cloud->radar_points[0].z_meters);
    // TODO(APT-746): Velocities need to be updated for accumulated points
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].velocity_m_s,
                            transformed_accumulated_point_cloud->radar_points[0].velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].signal_to_noise_ratio,
                            transformed_accumulated_point_cloud->radar_points[0].signal_to_noise_ratio);

    accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_INT32(0,
                            memcmp(accumulated_point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point)));
    TEST_ASSERT_NOT_EQUAL_INT32(
        0, memcmp(&transformed_accumulated_point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point)));
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].x_meters - fix_when_received[1].position.east_meters +
                                fix_when_received[0].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].y_meters - fix_when_received[1].position.north_meters +
                                fix_when_received[0].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].z_meters - fix_when_received[1].position.up_meters +
                                fix_when_received[0].position.up_meters,
                            transformed_accumulated_point.z_meters);
    // TODO(APT-746): Velocities need to be updated for accumulated points
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].velocity_m_s, transformed_accumulated_point.velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].signal_to_noise_ratio,
                            transformed_accumulated_point.signal_to_noise_ratio);

    // No more points when iterating one point forward
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            (size_t)num_accumulated_point_clouds));
    // ... or two point clouds forward from the original iterator
    iterator = iterator_was;
    provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(&iterator, accumulated_point_clouds,
                                                                     (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(&iterator, accumulated_point_clouds,
                                                                     (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            (size_t)num_accumulated_point_clouds));

    iterator = iterator_was;

    // Point 0, using new fix
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[0].x_meters - fix_when_received[2].position.east_meters +
                                fix_when_received[1].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[0].y_meters - fix_when_received[2].position.north_meters +
                                fix_when_received[1].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[0].z_meters - fix_when_received[2].position.up_meters +
                                fix_when_received[1].position.up_meters,
                            transformed_accumulated_point.z_meters);

    // Point 1, using new fix
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[1].x_meters - fix_when_received[2].position.east_meters +
                                fix_when_received[1].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[1].y_meters - fix_when_received[2].position.north_meters +
                                fix_when_received[1].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[1].z_meters - fix_when_received[2].position.up_meters +
                                fix_when_received[1].position.up_meters,
                            transformed_accumulated_point.z_meters);

    // Point 2, using new fix
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].x_meters - fix_when_received[2].position.east_meters +
                                fix_when_received[0].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].y_meters - fix_when_received[2].position.north_meters +
                                fix_when_received[0].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].z_meters - fix_when_received[2].position.up_meters +
                                fix_when_received[0].position.up_meters,
                            transformed_accumulated_point.z_meters);

    free(transformed_accumulated_point_cloud);
    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_clouds_rotation_yaw(void)
{
    enum
    {
        num_accumulated_point_clouds = 3
    };
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));

    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    provizio_radar_point transformed_accumulated_point;
    memset(&transformed_accumulated_point, 0, sizeof(provizio_radar_point));

    enum
    {
        num_point_clouds = num_accumulated_point_clouds
    };
    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_point_clouds * sizeof(provizio_radar_point_cloud));
    provizio_enu_fix fix_when_received[num_point_clouds];
    memset(fix_when_received, 0, sizeof(fix_when_received));

    // Point Cloud 0
    point_clouds[0].frame_index = 0;
    point_clouds[0].num_points_expected = point_clouds[0].num_points_received = 1;
    point_clouds[0].radar_points[0].x_meters = 101.0F;
    point_clouds[0].radar_points[0].y_meters = 102.0F;
    point_clouds[0].radar_points[0].z_meters = 103.0F;
    point_clouds[0].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation);

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F;
    point_clouds[1].radar_points[0].y_meters = 120.0F;
    point_clouds[1].radar_points[0].z_meters = 130.0F;
    point_clouds[1].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 6.0), &fix_when_received[1].orientation);

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F;
    point_clouds[2].radar_points[0].y_meters = 300.0F;
    point_clouds[2].radar_points[0].z_meters = 400.0F;
    point_clouds[2].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 4.0), &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator = provizio_accumulate_radar_point_cloud(
        &point_clouds[0], &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(138.4686F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(37.83459F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 2
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(137.31F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(87.44099F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(143.5427F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(0.7071018F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_clouds_rotation_pitch(void)
{
    enum
    {
        num_accumulated_point_clouds = 3
    };
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));

    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    provizio_radar_point transformed_accumulated_point;
    memset(&transformed_accumulated_point, 0, sizeof(provizio_radar_point));

    enum
    {
        num_point_clouds = num_accumulated_point_clouds
    };
    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_point_clouds * sizeof(provizio_radar_point_cloud));
    provizio_enu_fix fix_when_received[num_point_clouds];
    memset(fix_when_received, 0, sizeof(fix_when_received));

    // Point Cloud 0
    point_clouds[0].frame_index = 0;
    point_clouds[0].num_points_expected = point_clouds[0].num_points_received = 1;
    point_clouds[0].radar_points[0].x_meters = 101.0F;
    point_clouds[0].radar_points[0].y_meters = 102.0F;
    point_clouds[0].radar_points[0].z_meters = 103.0F;
    point_clouds[0].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation);

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F;
    point_clouds[1].radar_points[0].y_meters = 120.0F;
    point_clouds[1].radar_points[0].z_meters = 130.0F;
    point_clouds[1].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles(0.0F, (float)(M_PI / 6.0), 0.0F, &fix_when_received[1].orientation);

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F;
    point_clouds[2].radar_points[0].y_meters = 300.0F;
    point_clouds[2].radar_points[0].z_meters = 400.0F;
    point_clouds[2].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles(0.0F, (float)(M_PI / 4.0), 0.0F, &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator = provizio_accumulate_radar_point_cloud(
        &point_clouds[0], &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(35.96857F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(139.7F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 2
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(72.60535F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(154.04F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(-1.414223F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(144.25F, transformed_accumulated_point.z_meters);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_clouds_rotation_roll(void)
{
    enum
    {
        num_accumulated_point_clouds = 3
    };
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));

    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    provizio_radar_point transformed_accumulated_point;
    memset(&transformed_accumulated_point, 0, sizeof(provizio_radar_point));

    enum
    {
        num_point_clouds = num_accumulated_point_clouds
    };
    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_point_clouds * sizeof(provizio_radar_point_cloud));
    provizio_enu_fix fix_when_received[num_point_clouds];
    memset(fix_when_received, 0, sizeof(fix_when_received));

    // Point Cloud 0
    point_clouds[0].frame_index = 0;
    point_clouds[0].num_points_expected = point_clouds[0].num_points_received = 1;
    point_clouds[0].radar_points[0].x_meters = 101.0F;
    point_clouds[0].radar_points[0].y_meters = 102.0F;
    point_clouds[0].radar_points[0].z_meters = 103.0F;
    point_clouds[0].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation);

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F;
    point_clouds[1].radar_points[0].y_meters = 120.0F;
    point_clouds[1].radar_points[0].z_meters = 130.0F;
    point_clouds[1].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles((float)(M_PI / 6.0), 0.0F, 0.0F, &fix_when_received[1].orientation);

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F;
    point_clouds[2].radar_points[0].y_meters = 300.0F;
    point_clouds[2].radar_points[0].z_meters = 400.0F;
    point_clouds[2].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;
    provizio_quaternion_set_euler_angles((float)(M_PI / 4.0), 0.0F, 0.0F, &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator = provizio_accumulate_radar_point_cloud(
        &point_clouds[0], &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(139.8346F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(38.20061F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 2
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(149.5576F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(94.51205F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(144.9569F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(0.7070923F, transformed_accumulated_point.z_meters);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_clouds_rotation_and_move_simple(void)
{
    provizio_accumulated_radar_point_cloud *accumulated_point_cloud =
        (provizio_accumulated_radar_point_cloud *)malloc(sizeof(provizio_accumulated_radar_point_cloud));
    provizio_radar_point_cloud *point_cloud = (provizio_radar_point_cloud *)malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));

    point_cloud->num_points_expected = point_cloud->num_points_received = 1;
    point_cloud->radar_points[0].x_meters = 1.0F;
    point_cloud->radar_points[0].y_meters = 2.0F;
    point_cloud->radar_points[0].z_meters = 3.0F;
    point_cloud->radar_points[0].velocity_m_s = 10.0F;
    point_cloud->radar_points[0].signal_to_noise_ratio = 10.0F;

    provizio_enu_fix fix_when_received;
    fix_when_received.position.east_meters = 10.0F;
    fix_when_received.position.north_meters = 20.0F;
    fix_when_received.position.up_meters = 0.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)M_PI * 3.0F / 4.0F, &fix_when_received.orientation);

    provizio_enu_fix current_fix;
    current_fix.position.east_meters = 20.0F;
    current_fix.position.north_meters = 10.0F;
    current_fix.position.up_meters = 0.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)M_PI / 4.0F, &current_fix.orientation);

    provizio_accumulated_radar_point_clouds_init(accumulated_point_cloud, 1);
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(point_cloud, &fix_when_received, accumulated_point_cloud, 1);

    provizio_radar_point out_point;
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &current_fix, accumulated_point_cloud, 1,
                                                              &out_point);

    TEST_ASSERT_EQUAL_FLOAT(-point_cloud->radar_points[0].y_meters, out_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_cloud->radar_points[0].x_meters + 10 * sqrtf(2), out_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_cloud->radar_points[0].z_meters, out_point.z_meters);

    free(point_cloud);
    free(accumulated_point_cloud);
}

static void test_accumulate_radar_point_clouds_rotation_and_move(void)
{
    enum
    {
        num_accumulated_point_clouds = 3
    };
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));

    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    provizio_radar_point transformed_accumulated_point;
    memset(&transformed_accumulated_point, 0, sizeof(provizio_radar_point));

    enum
    {
        num_point_clouds = num_accumulated_point_clouds
    };
    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_point_clouds * sizeof(provizio_radar_point_cloud));
    provizio_enu_fix fix_when_received[num_point_clouds];
    memset(fix_when_received, 0, sizeof(fix_when_received));

    // UTM Zone 29 is used for positions, 506000 is deducted from East, 5839000 from North for precision

    // Point Cloud 0
    point_clouds[0].frame_index = 0;
    point_clouds[0].num_points_expected = point_clouds[0].num_points_received = 1;
    point_clouds[0].radar_points[0].x_meters = 101.0F;
    point_clouds[0].radar_points[0].y_meters = 102.0F;
    point_clouds[0].radar_points[0].z_meters = 103.0F;
    point_clouds[0].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;
    fix_when_received[0].position.east_meters = 879.020F;
    fix_when_received[0].position.north_meters = 529.971F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation);

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F;
    point_clouds[1].radar_points[0].y_meters = 120.0F;
    point_clouds[1].radar_points[0].z_meters = 130.0F;
    point_clouds[1].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F;
    fix_when_received[1].position.east_meters = 871.156F;
    fix_when_received[1].position.north_meters = 548.981F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 6.0), &fix_when_received[1].orientation);

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F;
    point_clouds[2].radar_points[0].y_meters = 300.0F;
    point_clouds[2].radar_points[0].z_meters = 400.0F;
    point_clouds[2].radar_points[0].velocity_m_s = 5.0F;
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;
    fix_when_received[2].position.east_meters = 899.447F;
    fix_when_received[2].position.north_meters = 562.369F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 4.0), &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator = provizio_accumulate_radar_point_cloud(
        &point_clouds[0], &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(118.07F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(35.11F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds);
    // Point 2
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(147.89F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(57.97F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point);
    TEST_ASSERT_EQUAL_FLOAT(133.73F, transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(-31.49F, transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_clouds_overflow(void)
{
    // TODO(iivanov): test_accumulate_radar_point_clouds_overflow
}

int provizio_run_test_points_accumulation(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_accumulate_radar_point_clouds_move);
    RUN_TEST(test_accumulate_radar_point_clouds_rotation_yaw);
    RUN_TEST(test_accumulate_radar_point_clouds_rotation_pitch);
    RUN_TEST(test_accumulate_radar_point_clouds_rotation_roll);
    RUN_TEST(test_accumulate_radar_point_clouds_rotation_and_move_simple);
    RUN_TEST(test_accumulate_radar_point_clouds_rotation_and_move);
    RUN_TEST(test_accumulate_radar_point_clouds_overflow);

    return UNITY_END();
}
