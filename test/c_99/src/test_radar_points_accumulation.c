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

#include "provizio/radar_api/common.h"

#include <linmath.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "provizio/radar_api/radar_points_accumulation.h"

enum
{
    test_message_length = 1024
};
static char provizio_test_error[test_message_length];   // NOLINT: non-const global by design
static char provizio_test_warning[test_message_length]; // NOLINT: non-const global by design

static void test_provizio_on_error(const char *error)
{
    strncpy(provizio_test_error, error, test_message_length - 1);
}

static void test_provizio_on_warning(const char *warning)
{
    strncpy(provizio_test_warning, warning, test_message_length - 1);
}

static void check_transformation_matrix(float expected_x, float expected_y, float expected_z,
                                        const provizio_radar_point *in_point, mat4x4 transformation_matrix)
{
    vec4 out_vec = {0};
    vec4 in_vec = {in_point->x_meters, in_point->y_meters, in_point->z_meters, 1.0F};
    mat4x4_mul_vec4(out_vec, transformation_matrix, in_vec);
    TEST_ASSERT_EQUAL_FLOAT(expected_x, out_vec[0]); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(expected_y, out_vec[1]); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(expected_z, out_vec[2]); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(1.0F, out_vec[3]);       // NOLINT
}

static void check_transformation_matrix_for_cloud(const provizio_radar_point_cloud *point_cloud,
                                                  const provizio_radar_point_cloud *transformed_point_cloud,
                                                  mat4x4 transformation_matrix)
{
    TEST_ASSERT_EQUAL_UINT16(point_cloud->num_points_received, transformed_point_cloud->num_points_received);
    for (uint16_t i = 0; i < point_cloud->num_points_received; ++i)
    {
        check_transformation_matrix(
            transformed_point_cloud->radar_points[i].x_meters, transformed_point_cloud->radar_points[i].y_meters,
            transformed_point_cloud->radar_points[i].z_meters, &point_cloud->radar_points[i], transformation_matrix);
    }
}

static void filter_out_all(const provizio_radar_point *in_points, uint16_t num_in_points,
                           provizio_accumulated_radar_point_cloud *accumulated_point_clouds,
                           size_t num_accumulated_point_clouds,
                           const provizio_accumulated_radar_point_cloud_iterator *new_iterator, void *user_data,
                           provizio_radar_point *out_points, uint16_t *num_out_points)
{
    (void)in_points;
    (void)num_in_points;
    (void)accumulated_point_clouds;
    (void)num_accumulated_point_clouds;
    (void)new_iterator;
    (void)user_data;
    (void)out_points;

    *num_out_points = 0;
}

static void test_accumulate_radar_point_cloud_0_accumulated_point_clouds(void)
{
    provizio_set_on_error(&test_provizio_on_error);

    const provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(NULL, NULL, NULL, 0, NULL, NULL);
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_cloud_index);
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_index);
    TEST_ASSERT_EQUAL_STRING("provizio_accumulate_radar_point_cloud: num_accumulated_point_clouds can't be 0",
                             provizio_test_error);

    provizio_set_on_error(NULL);
}

static void test_accumulate_radar_point_cloud_invalid_orientation(void)
{
    provizio_set_on_error(&test_provizio_on_error);

    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(fix_when_received));

    const provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(NULL, &fix_when_received, NULL, 1, NULL, NULL);
    TEST_ASSERT_EQUAL_size_t(1, iterator.point_cloud_index);
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_index);
    TEST_ASSERT_EQUAL_STRING(
        "provizio_accumulate_radar_point_cloud: fix_when_received->orientation is not a valid rotation",
        provizio_test_error);

    provizio_set_on_error(NULL);
}

static void test_accumulate_radar_point_cloud_obsolete_frame(void)
{
    enum
    {
        num_accumulated_point_clouds = 2
    };

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_accumulated_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_accumulated_point_clouds * sizeof(provizio_radar_point_cloud));

    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(fix_when_received));
    provizio_quaternion_set_identity(&fix_when_received.orientation);

    // Newer cloud
    point_clouds[0].frame_index = 2;
    point_clouds[0].timestamp = 2;
    point_clouds[0].num_points_received = point_clouds[0].num_points_expected = 1;
    provizio_accumulated_radar_point_cloud_iterator iterator = provizio_accumulate_radar_point_cloud(
        &point_clouds[0], &fix_when_received, accumulated_point_clouds, num_accumulated_point_clouds, NULL, NULL);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_point_clouds));

    // Older cloud
    point_clouds[1].frame_index = 1;
    point_clouds[1].timestamp = 1;
    point_clouds[1].num_points_received = point_clouds[1].num_points_expected = 2;
    provizio_set_on_error(&test_provizio_on_error);
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received, accumulated_point_clouds,
                                                     num_accumulated_point_clouds, NULL, NULL);

    // Makes sure the older cloud got dropped
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_STRING(
        "provizio_accumulate_radar_point_cloud: Can't accumulate an older point cloud after a newer one",
        provizio_test_error);

    provizio_set_on_error(NULL);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_cloud_move(void)
{
    enum
    {
        num_accumulated_point_clouds = 2
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

    mat4x4 transformation_matrix;
    memset(transformation_matrix, 0, sizeof(transformation_matrix));

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
    point_clouds[0].radar_points[0].x_meters = 1.0F;                           // NOLINT: Fine in tests
    point_clouds[0].radar_points[0].y_meters = 2.0F;                           // NOLINT: Fine in tests
    point_clouds[0].radar_points[0].z_meters = 3.0F;                           // NOLINT: Fine in tests
    point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s = 4.0F; // NOLINT: Fine in tests
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: Fine in tests

    // Point Cloud 1
    point_clouds[1].frame_index = 2;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 2;
    point_clouds[1].radar_points[0].x_meters = 10.0F;                            // NOLINT: Fine in tests
    point_clouds[1].radar_points[0].y_meters = 20.0F;                            // NOLINT: Fine in tests
    point_clouds[1].radar_points[0].z_meters = 30.0F;                            // NOLINT: Fine in tests
    point_clouds[1].radar_points[0].radar_relative_radial_velocity_m_s = 40.0F;  // NOLINT: Fine in tests
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 50.0F;               // NOLINT: Fine in tests
    point_clouds[1].radar_points[1].x_meters = 100.0F;                           // NOLINT: Fine in tests
    point_clouds[1].radar_points[1].y_meters = 200.0F;                           // NOLINT: Fine in tests
    point_clouds[1].radar_points[1].z_meters = 300.0F;                           // NOLINT: Fine in tests
    point_clouds[1].radar_points[1].radar_relative_radial_velocity_m_s = 400.0F; // NOLINT: Fine in tests
    point_clouds[1].radar_points[1].signal_to_noise_ratio = 500.0F;              // NOLINT: Fine in tests

    // Fix 0
    fix_when_received[0].position.east_meters = 1.0F;  // NOLINT: Fine in tests
    fix_when_received[0].position.north_meters = 2.0F; // NOLINT: Fine in tests
    fix_when_received[0].position.up_meters = 3.0F;    // NOLINT: Fine in tests
    provizio_quaternion_set_identity(&fix_when_received[0].orientation);

    // Fix 1
    fix_when_received[1].position.east_meters = 6.0F;  // NOLINT: Fine in tests
    fix_when_received[1].position.north_meters = 5.0F; // NOLINT: Fine in tests
    fix_when_received[1].position.up_meters = 4.0F;    // NOLINT: Fine in tests
    provizio_quaternion_set_identity(&fix_when_received[1].orientation);

    // Fix 2
    fix_when_received[2].position.east_meters = 1000.0F;  // NOLINT: Fine in tests
    fix_when_received[2].position.north_meters = 2000.0F; // NOLINT: Fine in tests
    fix_when_received[2].position.up_meters = 3000.0F;    // NOLINT: Fine in tests
    provizio_quaternion_set_identity(&fix_when_received[2].orientation);

    // Accumulate the first cloud
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received[0], accumulated_point_clouds,
                                              (size_t)num_accumulated_point_clouds, NULL, NULL);
    provizio_accumulated_radar_point_cloud_iterator iterator_was = iterator;

    TEST_ASSERT_EQUAL_size_t(1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_cloud_index);
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_index);

    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));

    // We got our point cloud back, as we haven't moved yet
    const provizio_accumulated_radar_point_cloud *accumulated_point_cloud =
        provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
            &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
            transformed_accumulated_point_cloud, (float *)transformation_matrix);
    TEST_ASSERT_EQUAL_INT32(0, memcmp(&accumulated_point_cloud->point_cloud, &point_clouds[0], // NOLINT
                                      sizeof(provizio_radar_point_cloud)));
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(transformed_accumulated_point_cloud, &point_clouds[0], sizeof(provizio_radar_point_cloud))); // NOLINT
    check_transformation_matrix_for_cloud(&accumulated_point_cloud->point_cloud, transformed_accumulated_point_cloud,
                                          transformation_matrix);

    // Same no changes with the only point in the point cloud
    const provizio_radar_point *accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(accumulated_point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point)));  // NOLINT
    TEST_ASSERT_EQUAL_INT32(0, memcmp(&transformed_accumulated_point, &point_clouds[0].radar_points[0], // NOLINT
                                      sizeof(provizio_radar_point)));

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

    // Accumulate the same cloud in the second time (with only frame index increased) - to test the oldest cloud will
    // then be dropped on accumulating the next one
    ++point_clouds[0].frame_index;
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received[0], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    TEST_ASSERT_EQUAL_size_t(2, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        2, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(1, iterator.point_cloud_index);
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_index);

    // Accumulate the second cloud
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    iterator_was = iterator;

    TEST_ASSERT_EQUAL_size_t(2, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        3, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_cloud_index);
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_index);

    // We got our last point cloud back, as we haven't moved yet since it's been received
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    accumulated_point_cloud = provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        transformed_accumulated_point_cloud, NULL);
    TEST_ASSERT_EQUAL_INT32(0, memcmp(&accumulated_point_cloud->point_cloud, &point_clouds[1], // NOLINT
                                      sizeof(provizio_radar_point_cloud)));
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(transformed_accumulated_point_cloud, &point_clouds[1], sizeof(provizio_radar_point_cloud))); // NOLINT

    // Same no changes with both points in the last point cloud
    accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(accumulated_point, &point_clouds[1].radar_points[0], sizeof(provizio_radar_point)));  // NOLINT
    TEST_ASSERT_EQUAL_INT32(0, memcmp(&transformed_accumulated_point, &point_clouds[1].radar_points[0], // NOLINT
                                      sizeof(provizio_radar_point)));
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(accumulated_point, &point_clouds[1].radar_points[1], sizeof(provizio_radar_point)));  // NOLINT
    TEST_ASSERT_EQUAL_INT32(0, memcmp(&transformed_accumulated_point, &point_clouds[1].radar_points[1], // NOLINT
                                      sizeof(provizio_radar_point)));

    // We moved since the older point cloud was received, so its position has moved
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             (size_t)num_accumulated_point_clouds));
    accumulated_point_cloud = provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        transformed_accumulated_point_cloud, NULL);
    TEST_ASSERT_EQUAL_INT32(0, memcmp(&accumulated_point_cloud->point_cloud, &point_clouds[0], // NOLINT
                                      sizeof(provizio_radar_point_cloud)));
    TEST_ASSERT_NOT_EQUAL_INT32(
        0, memcmp(transformed_accumulated_point_cloud, &point_clouds[0], sizeof(provizio_radar_point_cloud))); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].x_meters -                                         // NOLINT
                                fix_when_received[1].position.east_meters + fix_when_received[0].position.east_meters,
                            transformed_accumulated_point_cloud->radar_points[0].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].y_meters - // NOLINT
                                fix_when_received[1].position.north_meters + fix_when_received[0].position.north_meters,
                            transformed_accumulated_point_cloud->radar_points[0].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].z_meters - // NOLINT
                                fix_when_received[1].position.up_meters + fix_when_received[0].position.up_meters,
                            transformed_accumulated_point_cloud->radar_points[0].z_meters);
    // TODO(APT-746): Velocities need to be updated for accumulated points
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s, // NOLINT
                            transformed_accumulated_point_cloud->radar_points[0].radar_relative_radial_velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].signal_to_noise_ratio, // NOLINT
                            transformed_accumulated_point_cloud->radar_points[0].signal_to_noise_ratio);

    accumulated_point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_INT32(
        0, memcmp(accumulated_point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point)));      // NOLINT
    TEST_ASSERT_NOT_EQUAL_INT32(0, memcmp(&transformed_accumulated_point, &point_clouds[0].radar_points[0], // NOLINT
                                          sizeof(provizio_radar_point)));
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].x_meters - // NOLINT
                                fix_when_received[1].position.east_meters + fix_when_received[0].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].y_meters - // NOLINT
                                fix_when_received[1].position.north_meters + fix_when_received[0].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].z_meters - // NOLINT
                                fix_when_received[1].position.up_meters + fix_when_received[0].position.up_meters,
                            transformed_accumulated_point.z_meters);
    // TODO(APT-746): Velocities need to be updated for accumulated points
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s, // NOLINT
                            transformed_accumulated_point.radar_relative_radial_velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].signal_to_noise_ratio, // NOLINT
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
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[0].x_meters - // NOLINT
                                fix_when_received[2].position.east_meters + fix_when_received[1].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[0].y_meters - // NOLINT
                                fix_when_received[2].position.north_meters + fix_when_received[1].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[0].z_meters - // NOLINT
                                fix_when_received[2].position.up_meters + fix_when_received[1].position.up_meters,
                            transformed_accumulated_point.z_meters);

    // Point 1, using new fix
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[1].x_meters - // NOLINT
                                fix_when_received[2].position.east_meters + fix_when_received[1].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[1].y_meters - // NOLINT
                                fix_when_received[2].position.north_meters + fix_when_received[1].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[1].radar_points[1].z_meters - // NOLINT
                                fix_when_received[2].position.up_meters + fix_when_received[1].position.up_meters,
                            transformed_accumulated_point.z_meters);

    // Point 2, using new fix
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].x_meters - // NOLINT
                                fix_when_received[2].position.east_meters + fix_when_received[0].position.east_meters,
                            transformed_accumulated_point.x_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].y_meters - // NOLINT
                                fix_when_received[2].position.north_meters + fix_when_received[0].position.north_meters,
                            transformed_accumulated_point.y_meters);
    TEST_ASSERT_EQUAL_FLOAT(point_clouds[0].radar_points[0].z_meters - // NOLINT
                                fix_when_received[2].position.up_meters + fix_when_received[0].position.up_meters,
                            transformed_accumulated_point.z_meters);

    // Accumulate the second cloud again (with only frame index increased) to check it drops the oldest cloud correctly
    ++point_clouds[1].frame_index;
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    TEST_ASSERT_EQUAL_size_t(2, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        4, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(1, iterator.point_cloud_index);
    TEST_ASSERT_EQUAL_size_t(0, iterator.point_index);

    // Try accumulating an empty point cloud: should just be ignored and return the end iterator
    point_clouds[1].num_points_received = 0;
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    TEST_ASSERT_EQUAL_size_t(2, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds,
                                                                              (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        4, provizio_accumulated_radar_points_count(accumulated_point_clouds, (size_t)num_accumulated_point_clouds));
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            num_accumulated_point_clouds));

    free(transformed_accumulated_point_cloud);
    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_cloud_rotation_yaw(void)
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
    point_clouds[0].radar_points[0].x_meters = 101.0F;                                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].y_meters = 102.0F;                                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].z_meters = 103.0F;                                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F;                 // NOLINT: fine in tests
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;                              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation); // NOLINT: fine in tests

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].y_meters = 120.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].z_meters = 130.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 6.0),      // NOLINT: fine in tests
                                         &fix_when_received[1].orientation);

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].y_meters = 300.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].z_meters = 400.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 4.0),      // NOLINT: fine in tests
                                         &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received[0], accumulated_point_clouds,
                                              (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(138.4686F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(37.83459F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);    // NOLINT: fine in tests

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 2
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(137.31F, transformed_accumulated_point.x_meters);   // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(87.44099F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);    // NOLINT: fine in tests
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(143.5427F, transformed_accumulated_point.x_meters);  // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(0.7071018F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);     // NOLINT: fine in tests

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_cloud_rotation_pitch(void)
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
    point_clouds[0].radar_points[0].x_meters = 101.0F;                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].y_meters = 102.0F;                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].z_meters = 103.0F;                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation);

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].y_meters = 120.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].z_meters = 130.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles(0.0F, (float)(M_PI / 6.0), 0.0F,      // NOLINT: fine in tests
                                         &fix_when_received[1].orientation);

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].y_meters = 300.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].z_meters = 400.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles(0.0F, (float)(M_PI / 4.0), 0.0F,      // NOLINT: fine in tests
                                         &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received[0], accumulated_point_clouds,
                                              (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(35.96857F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);    // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(139.7F, transformed_accumulated_point.z_meters);    // NOLINT: fine in tests

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 2
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(72.60535F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters);    // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(154.04F, transformed_accumulated_point.z_meters);   // NOLINT: fine in tests
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(-1.414223F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters);     // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(144.25F, transformed_accumulated_point.z_meters);    // NOLINT: fine in tests

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_cloud_rotation_roll(void)
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
    point_clouds[0].radar_points[0].x_meters = 101.0F;                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].y_meters = 102.0F;                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].z_meters = 103.0F;                         // NOLINT: fine in tests
    point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation);

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].y_meters = 120.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].z_meters = 130.0F;                         // NOLINT: fine in tests
    point_clouds[1].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles((float)(M_PI / 6.0), 0.0F, 0.0F,      // NOLINT: fine in tests
                                         &fix_when_received[1].orientation);

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].y_meters = 300.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].z_meters = 400.0F;                         // NOLINT: fine in tests
    point_clouds[2].radar_points[0].radar_relative_radial_velocity_m_s = 5.0F; // NOLINT: fine in tests
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;              // NOLINT: fine in tests
    provizio_quaternion_set_euler_angles((float)(M_PI / 4.0), 0.0F, 0.0F,      // NOLINT: fine in tests
                                         &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received[0], accumulated_point_clouds,
                                              (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);    // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(139.8346F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(38.20061F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 2
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters);    // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(149.5576F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(94.51205F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, NULL);
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters);     // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(144.9569F, transformed_accumulated_point.y_meters);  // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(0.7070923F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_cloud_rotation_and_move_simple(void)
{
    provizio_accumulated_radar_point_cloud *accumulated_point_cloud =
        (provizio_accumulated_radar_point_cloud *)malloc(sizeof(provizio_accumulated_radar_point_cloud));
    provizio_radar_point_cloud *point_cloud = (provizio_radar_point_cloud *)malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));

    point_cloud->num_points_expected = point_cloud->num_points_received = 1;
    point_cloud->radar_points[0].x_meters = 1.0F;                            // NOLINT: magic numbers are fine in tests
    point_cloud->radar_points[0].y_meters = 2.0F;                            // NOLINT: magic numbers are fine in tests
    point_cloud->radar_points[0].z_meters = 3.0F;                            // NOLINT: magic numbers are fine in tests
    point_cloud->radar_points[0].radar_relative_radial_velocity_m_s = 10.0F; // NOLINT: magic numbers are fine in tests
    point_cloud->radar_points[0].signal_to_noise_ratio = 10.0F;              // NOLINT: magic numbers are fine in tests

    provizio_enu_fix fix_when_received;
    fix_when_received.position.east_meters = 10.0F;  // NOLINT: magic numbers are fine in tests
    fix_when_received.position.north_meters = 20.0F; // NOLINT: magic numbers are fine in tests
    fix_when_received.position.up_meters = 0.0F;     // NOLINT: magic numbers are fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)M_PI * 3.0F / 4.0F, // NOLINT
                                         &fix_when_received.orientation);

    provizio_enu_fix current_fix;
    current_fix.position.east_meters = 20.0F;  // NOLINT: magic numbers are fine in tests
    current_fix.position.north_meters = 10.0F; // NOLINT: magic numbers are fine in tests
    current_fix.position.up_meters = 0.0F;     // NOLINT: magic numbers are fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)M_PI / 4.0F, &current_fix.orientation); // NOLINT

    provizio_accumulated_radar_point_clouds_init(accumulated_point_cloud, 1);
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(point_cloud, &fix_when_received, accumulated_point_cloud, 1, NULL, NULL);

    provizio_radar_point out_point;
    mat4x4 out_transformation_matrix;
    memset(out_transformation_matrix, 0, sizeof(out_transformation_matrix));
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &current_fix, accumulated_point_cloud, 1,
                                                              &out_point, (float *)out_transformation_matrix);

    TEST_ASSERT_EQUAL_FLOAT(-point_cloud->radar_points[0].y_meters, out_point.x_meters);                // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(point_cloud->radar_points[0].x_meters + 10 * sqrtf(2), out_point.y_meters); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(point_cloud->radar_points[0].z_meters, out_point.z_meters);                 // NOLINT
    check_transformation_matrix(
        -point_cloud->radar_points[0].y_meters, point_cloud->radar_points[0].x_meters + 10 * sqrtf(2), // NOLINT
        point_cloud->radar_points[0].z_meters, &point_cloud->radar_points[0], out_transformation_matrix);

    free(point_cloud);
    free(accumulated_point_cloud);
}

static void test_accumulate_radar_point_cloud_rotation_and_move(void)
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
    mat4x4 transformation_matrix;
    memset(transformation_matrix, 0, sizeof(transformation_matrix));

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
    point_clouds[0].radar_points[0].x_meters = 101.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[0].radar_points[0].y_meters = 102.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[0].radar_points[0].z_meters = 103.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s =
        5.0F;                                                     // NOLINT: magic numbers are fine in tests
    point_clouds[0].radar_points[0].signal_to_noise_ratio = 5.0F; // NOLINT: magic numbers are fine in tests
    fix_when_received[0].position.east_meters = 879.020F;         // NOLINT: magic numbers are fine in tests
    fix_when_received[0].position.north_meters = 529.971F;        // NOLINT: magic numbers are fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received[0].orientation); // NOLINT

    // Point Cloud 1
    point_clouds[1].frame_index = 1;
    point_clouds[1].num_points_expected = point_clouds[1].num_points_received = 1;
    point_clouds[1].radar_points[0].x_meters = 110.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[1].radar_points[0].y_meters = 120.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[1].radar_points[0].z_meters = 130.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[1].radar_points[0].radar_relative_radial_velocity_m_s =
        5.0F;                                                     // NOLINT: magic numbers are fine in tests
    point_clouds[1].radar_points[0].signal_to_noise_ratio = 5.0F; // NOLINT: magic numbers are fine in tests
    fix_when_received[1].position.east_meters = 871.156F;         // NOLINT: magic numbers are fine in tests
    fix_when_received[1].position.north_meters = 548.981F;        // NOLINT: magic numbers are fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 6.0), &fix_when_received[1].orientation); // NOLINT

    // Point Cloud 2
    point_clouds[2].frame_index = 2;
    point_clouds[2].num_points_expected = point_clouds[2].num_points_received = 1;
    point_clouds[2].radar_points[0].x_meters = 200.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[2].radar_points[0].y_meters = 300.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[2].radar_points[0].z_meters = 400.0F; // NOLINT: magic numbers are fine in tests
    point_clouds[2].radar_points[0].radar_relative_radial_velocity_m_s =
        5.0F;                                                             // NOLINT: magic numbers are fine in tests
    point_clouds[2].radar_points[0].signal_to_noise_ratio = 5.0F;         // NOLINT: magic numbers are fine in tests
    fix_when_received[2].position.east_meters = 899.447F;                 // NOLINT: magic numbers are fine in tests
    fix_when_received[2].position.north_meters = 562.369F;                // NOLINT: magic numbers are fine in tests
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)(M_PI / 4.0), // NOLINT
                                         &fix_when_received[2].orientation);

    // Accumulate and check: iteration 0
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received[0], accumulated_point_clouds,
                                              (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 0
    const provizio_radar_point *point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[0], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, (float *)transformation_matrix);
    TEST_ASSERT_EQUAL(0, memcmp(point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point))); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(101.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(102.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    check_transformation_matrix(
        101.0F, 102.0F, 103.0F, // NOLINT: magic numbers are fine in tests
        &accumulated_point_clouds[iterator.point_cloud_index].point_cloud.radar_points[iterator.point_index],
        transformation_matrix);

    // Accumulate and check: iteration 1
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received[1], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 1
    point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, (float *)transformation_matrix);
    TEST_ASSERT_EQUAL(0, memcmp(point, &point_clouds[1].radar_points[0], sizeof(provizio_radar_point))); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(110.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(120.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    check_transformation_matrix(
        110.0F, 120.0F, 130.0F, // NOLINT: magic numbers are fine in tests
        &accumulated_point_clouds[iterator.point_cloud_index].point_cloud.radar_points[iterator.point_index],
        transformation_matrix);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[1], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, (float *)transformation_matrix);
    TEST_ASSERT_EQUAL(0, memcmp(point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point))); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(135.774F, transformed_accumulated_point.x_meters);  // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(17.43943F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);    // NOLINT: fine in tests
    check_transformation_matrix(
        135.774F, 17.43943F, 103.0F, // NOLINT: magic numbers are fine in tests
        &accumulated_point_clouds[iterator.point_cloud_index].point_cloud.radar_points[iterator.point_index],
        transformation_matrix);

    // Accumulate and check: iteration 2
    iterator = provizio_accumulate_radar_point_cloud(&point_clouds[2], &fix_when_received[2], accumulated_point_clouds,
                                                     (size_t)num_accumulated_point_clouds, NULL, NULL);
    // Point 2
    point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, (float *)transformation_matrix);
    TEST_ASSERT_EQUAL(0, memcmp(point, &point_clouds[2].radar_points[0], sizeof(provizio_radar_point))); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(200.0F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(300.0F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(400.0F, transformed_accumulated_point.z_meters); // NOLINT: fine in tests
    check_transformation_matrix(
        200.0F, 300.0F, 400.0F, // NOLINT: magic numbers are fine in tests
        &accumulated_point_clouds[iterator.point_cloud_index].point_cloud.radar_points[iterator.point_index],
        transformation_matrix);
    // Point 1
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, (float *)transformation_matrix);
    TEST_ASSERT_EQUAL(0, memcmp(point, &point_clouds[1].radar_points[0], sizeof(provizio_radar_point))); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(107.8386F, transformed_accumulated_point.x_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(97.979F, transformed_accumulated_point.y_meters);   // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(130.0F, transformed_accumulated_point.z_meters);    // NOLINT: fine in tests
    check_transformation_matrix(
        107.8386F, 97.979F, 130.0F, // NOLINT: magic numbers are fine in tests
        &accumulated_point_clouds[iterator.point_cloud_index].point_cloud.radar_points[iterator.point_index],
        transformation_matrix);
    // Point 0
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               (size_t)num_accumulated_point_clouds);
    point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, &fix_when_received[2], accumulated_point_clouds, (size_t)num_accumulated_point_clouds,
        &transformed_accumulated_point, (float *)transformation_matrix);
    TEST_ASSERT_EQUAL(0, memcmp(point, &point_clouds[0].radar_points[0], sizeof(provizio_radar_point))); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(106.19F, transformed_accumulated_point.x_meters); // NOLINT: magic numbers are fine in tests
    TEST_ASSERT_EQUAL_FLOAT(-7.7577F, transformed_accumulated_point.y_meters); // NOLINT: fine in tests
    TEST_ASSERT_EQUAL_FLOAT(103.0F, transformed_accumulated_point.z_meters);   // NOLINT: fine in tests
    check_transformation_matrix(
        106.19F, -7.7577F, 103.0F, // NOLINT: magic numbers are fine in tests
        &accumulated_point_clouds[iterator.point_cloud_index].point_cloud.radar_points[iterator.point_index],
        transformation_matrix);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_accumulate_radar_point_cloud_overflow(void)
{
    enum
    {
        num_accumulated_point_clouds = 2
    };

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_accumulated_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_accumulated_point_clouds * sizeof(provizio_radar_point_cloud));

    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(fix_when_received));
    provizio_quaternion_set_identity(&fix_when_received.orientation);

    // Very high (though not maximal) frame-index point cloud
    point_clouds[0].frame_index = UINT32_MAX - 2; // Given the API supports safely missing quite a few packets, there
                                                  // should be no need for it to work to be UINT32_MAX
    point_clouds[0].timestamp = 1;
    point_clouds[0].num_points_received = point_clouds[0].num_points_expected = 1;
    provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received, accumulated_point_clouds,
                                          num_accumulated_point_clouds, NULL, NULL);
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_point_clouds));

    // A low (though not minumal) frame-index point cloud
    point_clouds[1].frame_index = 3;
    point_clouds[1].timestamp = 2;
    point_clouds[1].num_points_received = point_clouds[1].num_points_expected = 2;
    provizio_set_on_warning(&test_provizio_on_warning);
    provizio_accumulate_radar_point_cloud(&point_clouds[1], &fix_when_received, accumulated_point_clouds,
                                          num_accumulated_point_clouds, NULL, NULL);

    // Makes sure the past (overflown) accumulation got dropped and reset
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        2, provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_STRING(
        "provizio_accumulate_radar_point_cloud: frame indices overflow detected - resetting accumulation",
        provizio_test_warning);

    provizio_set_on_warning(NULL);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_provizio_accumulate_radar_point_cloud_static(void)
{
    const float default_signal_to_noise_ratio = 10.0F;
    const float ego_velocity_m_s = 10.0F;
    const uint64_t time_between_frames =
        1000ULL *
        (uint64_t)(1000000.0F * (sqrtf(2.0F) / ego_velocity_m_s)); // Based on the fact that ego position grows by 1 on
                                                                   // both x and y every frame (sqrt(2))
    const size_t num_accumulated_clouds = 3;
    const float epsilon = 0.00001F; // Max float imprecision for this test

    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(provizio_enu_fix));

    provizio_radar_point_cloud *point_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds =
        malloc(num_accumulated_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_clouds);

    provizio_accumulated_radar_point_cloud_iterator iterator;
    memset(&iterator, 0, sizeof(iterator));

    provizio_radar_point transformed_point;
    memset(&transformed_point, 0, sizeof(transformed_point));

    point_cloud->num_points_received = point_cloud->num_points_expected = 2;
    // All 3 times, there is a static point at EgoRelative(1, 0)
    point_cloud->radar_points[0].x_meters = 1.0F;
    point_cloud->radar_points[0].y_meters = 0.0F;
    point_cloud->radar_points[0].signal_to_noise_ratio = default_signal_to_noise_ratio;
    // Ground-relative velocity is 0 m/s
    point_cloud->radar_points[0].radar_relative_radial_velocity_m_s = -ego_velocity_m_s + 0.0F;
    // All 3 times, there is a dynamic point at EgoRelative(0, 1)
    point_cloud->radar_points[1].x_meters = 0.0F;
    point_cloud->radar_points[1].y_meters = 1.0F;
    point_cloud->radar_points[1].signal_to_noise_ratio = default_signal_to_noise_ratio;
    // Ground-relative velocity is 10 m/s
    point_cloud->radar_points[1].radar_relative_radial_velocity_m_s = -ego_velocity_m_s + 10.0F; // NOLINT: fine in test

    // 1st reading: ENU(1, 1, 0), heading = 0, static point at ENU(2, 1, 0), moving point at ENU(1, 2, 0)
    fix_when_received.position.east_meters = fix_when_received.position.north_meters = 1.0F;
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, 0.0F, &fix_when_received.orientation);
    ++point_cloud->frame_index;
    point_cloud->timestamp += time_between_frames;
    iterator = provizio_accumulate_radar_point_cloud_static(point_cloud, &fix_when_received, accumulated_point_clouds,
                                                            num_accumulated_clouds);
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_clouds));
    TEST_ASSERT_EQUAL_size_t(1,
                             provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_clouds));
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             num_accumulated_clouds));
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &fix_when_received, accumulated_point_clouds,
                                                              num_accumulated_clouds, &transformed_point, NULL);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 1.0F, transformed_point.x_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.y_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.z_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, -ego_velocity_m_s + 0.0F, transformed_point.radar_relative_radial_velocity_m_s);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, default_signal_to_noise_ratio, transformed_point.signal_to_noise_ratio);
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               num_accumulated_clouds);
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            num_accumulated_clouds));

    // 2nd reading: ENU(2, 2), heading = PI / 2, static point at ENU(1, 2, 0), moving point at ENU(2, 3, 0)
    fix_when_received.position.east_meters = fix_when_received.position.north_meters = 2.0F; // NOLINT: fine in test
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)M_PI_2, &fix_when_received.orientation);
    ++point_cloud->frame_index;
    point_cloud->timestamp += time_between_frames;
    point_cloud->num_points_received = point_cloud->num_points_expected = 2;
    iterator = provizio_accumulate_radar_point_cloud_static(point_cloud, &fix_when_received, accumulated_point_clouds,
                                                            num_accumulated_clouds);
    TEST_ASSERT_EQUAL_size_t(
        2, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_clouds));
    TEST_ASSERT_EQUAL_size_t(2,
                             provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_clouds));
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             num_accumulated_clouds));
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &fix_when_received, accumulated_point_clouds,
                                                              num_accumulated_clouds, &transformed_point, NULL);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 1.0F, transformed_point.x_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.y_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.z_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, -ego_velocity_m_s + 0.0F, transformed_point.radar_relative_radial_velocity_m_s);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, default_signal_to_noise_ratio, transformed_point.signal_to_noise_ratio);
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               num_accumulated_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             num_accumulated_clouds));
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &fix_when_received, accumulated_point_clouds,
                                                              num_accumulated_clouds, &transformed_point, NULL);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, -1.0F, transformed_point.x_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.y_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.z_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, -ego_velocity_m_s + 0.0F, transformed_point.radar_relative_radial_velocity_m_s);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, default_signal_to_noise_ratio, transformed_point.signal_to_noise_ratio);
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               num_accumulated_clouds);
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            num_accumulated_clouds));

    // 3rd reading: ENU(3, 3), heading = PI, static point at ENU(2, 3, 0), moving point at ENU(3, 2, 0)
    fix_when_received.position.east_meters = fix_when_received.position.north_meters = 3.0F; // NOLINT: fine in test
    provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)M_PI, &fix_when_received.orientation);
    ++point_cloud->frame_index;
    point_cloud->timestamp += time_between_frames;
    point_cloud->num_points_received = point_cloud->num_points_expected = 2;
    iterator = provizio_accumulate_radar_point_cloud_static(point_cloud, &fix_when_received, accumulated_point_clouds,
                                                            num_accumulated_clouds);
    TEST_ASSERT_EQUAL_size_t(
        3, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_clouds));
    TEST_ASSERT_EQUAL_size_t(3,
                             provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_clouds));
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             num_accumulated_clouds));
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &fix_when_received, accumulated_point_clouds,
                                                              num_accumulated_clouds, &transformed_point, NULL);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 1.0F, transformed_point.x_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.y_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.z_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, -ego_velocity_m_s + 0.0F, transformed_point.radar_relative_radial_velocity_m_s);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, default_signal_to_noise_ratio, transformed_point.signal_to_noise_ratio);
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               num_accumulated_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             num_accumulated_clouds));
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &fix_when_received, accumulated_point_clouds,
                                                              num_accumulated_clouds, &transformed_point, NULL);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 1.0F, transformed_point.x_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.y_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.z_meters);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, -ego_velocity_m_s + 0.0F, transformed_point.radar_relative_radial_velocity_m_s);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, default_signal_to_noise_ratio, transformed_point.signal_to_noise_ratio);
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               num_accumulated_clouds);
    TEST_ASSERT_FALSE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                             num_accumulated_clouds));
    provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, &fix_when_received, accumulated_point_clouds,
                                                              num_accumulated_clouds, &transformed_point, NULL);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 1.0F, transformed_point.x_meters); // NOLINT
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 2.0F, transformed_point.y_meters); // NOLINT
    TEST_ASSERT_FLOAT_WITHIN(epsilon, 0.0F, transformed_point.z_meters); // NOLINT
    TEST_ASSERT_FLOAT_WITHIN(epsilon, -ego_velocity_m_s + 0.0F, transformed_point.radar_relative_radial_velocity_m_s);
    TEST_ASSERT_FLOAT_WITHIN(epsilon, default_signal_to_noise_ratio, transformed_point.signal_to_noise_ratio);
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               num_accumulated_clouds);
    TEST_ASSERT_TRUE(provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                            num_accumulated_clouds));

    free(accumulated_point_clouds);
    free(point_cloud);
}

static void test_accumulate_radar_point_cloud_all_filtered_out(void)
{
    enum
    {
        num_accumulated_point_clouds = 1
    };

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);

    provizio_radar_point_cloud *point_clouds =
        (provizio_radar_point_cloud *)malloc(num_accumulated_point_clouds * sizeof(provizio_radar_point_cloud));
    memset(point_clouds, 0, num_accumulated_point_clouds * sizeof(provizio_radar_point_cloud));

    provizio_enu_fix fix_when_received;
    memset(&fix_when_received, 0, sizeof(fix_when_received));
    provizio_quaternion_set_identity(&fix_when_received.orientation);

    point_clouds[0].frame_index = 1;
    point_clouds[0].timestamp = 1;
    point_clouds[0].num_points_received = point_clouds[0].num_points_expected = 10; // NOLINT
    const float base_x = 300.0F;
    for (uint16_t i = 0; i < point_clouds[0].num_points_received; ++i)
    {
        point_clouds[0].radar_points[i].x_meters = (float)i + base_x;
    }
    provizio_set_on_warning(&test_provizio_on_warning);
    provizio_accumulated_radar_point_cloud_iterator iterator =
        provizio_accumulate_radar_point_cloud(&point_clouds[0], &fix_when_received, accumulated_point_clouds,
                                              num_accumulated_point_clouds, filter_out_all, NULL);
    // Despite all of the 10 points were filtered out, the first one of them got accumulated
    TEST_ASSERT_EQUAL_STRING("provizio_accumulate_radar_point_cloud: filter removed all points, which is not "
                             "supported, so accumulating the first point instead",
                             provizio_test_warning);
    provizio_set_on_warning(NULL);
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_size_t(
        1, provizio_accumulated_radar_points_count(accumulated_point_clouds, num_accumulated_point_clouds));
    TEST_ASSERT_EQUAL_FLOAT( // NOLINT
        base_x, provizio_accumulated_radar_point_cloud_iterator_get_point(&iterator, NULL, accumulated_point_clouds,
                                                                          num_accumulated_point_clouds, NULL, NULL)
                    ->x_meters);

    free(point_clouds);
    free(accumulated_point_clouds);
}

static void test_provizio_accumulated_radar_point_cloud_iterator_next_point_cloud_empty(void)
{
    provizio_set_on_error(&test_provizio_on_error);
    provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(NULL, NULL, 0);
    TEST_ASSERT_EQUAL_STRING(
        "provizio_accumulated_radar_point_cloud_iterator_next_point_cloud: num_accumulated_point_clouds can't be 0",
        provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_accumulated_radar_point_cloud_iterator_next_point_empty(void)
{
    provizio_set_on_error(&test_provizio_on_error);
    provizio_accumulated_radar_point_cloud_iterator_next_point(NULL, NULL, 0);
    TEST_ASSERT_EQUAL_STRING(
        "provizio_accumulated_radar_point_cloud_iterator_next_point: num_accumulated_point_clouds can't be 0",
        provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_accumulated_radar_point_cloud_iterator_next_point_end(void)
{
    enum
    {
        num_accumulated_point_clouds = 1
    };

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    iterator.point_cloud_index = iterator.point_index = 0;

    provizio_set_on_error(&test_provizio_on_error);
    provizio_accumulated_radar_point_cloud_iterator_next_point(&iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds);
    TEST_ASSERT_EQUAL_STRING(
        "provizio_accumulated_radar_point_cloud_iterator_next_point: can't go next point on an end iterator",
        provizio_test_error);
    provizio_set_on_error(NULL);

    free(accumulated_point_clouds);
}

static void test_provizio_accumulated_radar_point_cloud_iterator_get_point_cloud_end(void)
{
    enum
    {
        num_accumulated_point_clouds = 1
    };

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    iterator.point_cloud_index = iterator.point_index = 0;

    provizio_radar_point_cloud *transformed_cloud = malloc(sizeof(provizio_radar_point_cloud));
    memset(transformed_cloud, UINT8_MAX, sizeof(provizio_radar_point_cloud)); // At first filled with all-ones
    mat4x4 transformation_matrix;
    memset(transformation_matrix, UINT8_MAX, sizeof(transformation_matrix)); // At first filled with all-ones

    const provizio_accumulated_radar_point_cloud *cloud =
        provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(&iterator, NULL, accumulated_point_clouds,
                                                                        num_accumulated_point_clouds, transformed_cloud,
                                                                        (float *)transformation_matrix);
    TEST_ASSERT_NULL(cloud);
    TEST_ASSERT_TRUE( // It checks it's all zeroes now
        *((uint8_t *)transformed_cloud) == 0 &&
        memcmp(transformed_cloud, (uint8_t *)transformed_cloud + 1, sizeof(provizio_radar_point_cloud) - 1) == 0);
    TEST_ASSERT_TRUE( // It checks it's all zeroes now
        *((uint8_t *)transformation_matrix) == 0 &&
        memcmp(transformation_matrix, (uint8_t *)transformation_matrix + 1, // NOLINT
               sizeof(transformation_matrix) - 1) == 0);

    free(transformed_cloud);
    free(accumulated_point_clouds);
}

static void test_provizio_accumulated_radar_point_cloud_iterator_get_point_end(void)
{
    enum
    {
        num_accumulated_point_clouds = 1
    };

    provizio_accumulated_radar_point_cloud *accumulated_point_clouds = (provizio_accumulated_radar_point_cloud *)malloc(
        num_accumulated_point_clouds * sizeof(provizio_accumulated_radar_point_cloud));
    provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, (size_t)num_accumulated_point_clouds);
    provizio_accumulated_radar_point_cloud_iterator iterator;
    iterator.point_cloud_index = iterator.point_index = 0;

    provizio_radar_point transformed_point;
    memset(&transformed_point, UINT8_MAX, sizeof(transformed_point)); // At first filled with all-ones
    mat4x4 transformation_matrix;
    memset(transformation_matrix, UINT8_MAX, sizeof(transformation_matrix)); // At first filled with all-ones

    const provizio_radar_point *point = provizio_accumulated_radar_point_cloud_iterator_get_point(
        &iterator, NULL, accumulated_point_clouds, num_accumulated_point_clouds, &transformed_point,
        (float *)transformation_matrix);
    TEST_ASSERT_NULL(point);
    TEST_ASSERT_EQUAL_FLOAT(0, transformed_point.x_meters);                           // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(0, transformed_point.y_meters);                           // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(0, transformed_point.z_meters);                           // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(0, transformed_point.radar_relative_radial_velocity_m_s); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(0, transformed_point.signal_to_noise_ratio);              // NOLINT
    TEST_ASSERT_TRUE(                                                                 // It checks it's all zeroes now
        *((uint8_t *)transformation_matrix) == 0 &&
        memcmp(transformation_matrix, (uint8_t *)transformation_matrix + 1, // NOLINT
               sizeof(transformation_matrix) - 1) == 0);

    free(accumulated_point_clouds);
}

int provizio_run_test_points_accumulation(void)
{
    memset(provizio_test_error, 0, sizeof(provizio_test_error));
    memset(provizio_test_warning, 0, sizeof(provizio_test_warning));

    UNITY_BEGIN();

    RUN_TEST(test_accumulate_radar_point_cloud_0_accumulated_point_clouds);
    RUN_TEST(test_accumulate_radar_point_cloud_invalid_orientation);
    RUN_TEST(test_accumulate_radar_point_cloud_obsolete_frame);
    RUN_TEST(test_accumulate_radar_point_cloud_move);
    RUN_TEST(test_accumulate_radar_point_cloud_rotation_yaw);
    RUN_TEST(test_accumulate_radar_point_cloud_rotation_pitch);
    RUN_TEST(test_accumulate_radar_point_cloud_rotation_roll);
    RUN_TEST(test_accumulate_radar_point_cloud_rotation_and_move_simple);
    RUN_TEST(test_accumulate_radar_point_cloud_rotation_and_move);
    RUN_TEST(test_accumulate_radar_point_cloud_overflow);
    RUN_TEST(test_provizio_accumulate_radar_point_cloud_static);
    RUN_TEST(test_accumulate_radar_point_cloud_all_filtered_out);
    RUN_TEST(test_provizio_accumulated_radar_point_cloud_iterator_next_point_cloud_empty);
    RUN_TEST(test_provizio_accumulated_radar_point_cloud_iterator_next_point_empty);
    RUN_TEST(test_provizio_accumulated_radar_point_cloud_iterator_next_point_end);
    RUN_TEST(test_provizio_accumulated_radar_point_cloud_iterator_get_point_cloud_end);
    RUN_TEST(test_provizio_accumulated_radar_point_cloud_iterator_get_point_end);

    return UNITY_END();
}
