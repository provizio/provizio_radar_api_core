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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "unity/unity.h"

#include "provizio/radar_api/errno.h"
#include "provizio/radar_api/radar_point_cloud.h"
#include "provizio/socket.h"
#include "provizio/util.h"

#include "test_point_cloud_callbacks.h"

#pragma pack(push, 1)
typedef struct provizio_radar_point_protocol_v1
{
    float x_meters;                           // Forward, radar relative
    float y_meters;                           // Left, radar relative
    float z_meters;                           // Up, radar relative
    float radar_relative_radial_velocity_m_s; // Forward, radar relative
    float signal_to_noise_ratio;
} provizio_radar_point_protocol_v1;

#define PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET_PROTOCOL_V1 72

typedef struct provizio_radar_point_cloud_packet_protocol_v1
{
    provizio_radar_point_cloud_packet_header header;
    provizio_radar_point_protocol_v1 radar_points[PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET_PROTOCOL_V1];
} provizio_radar_point_cloud_packet_protocol_v1;
#pragma pack(pop)

enum
{
    test_message_length = 1024
};
static char provizio_test_warning[test_message_length]; // NOLINT: non-const global by design
static char provizio_test_error[test_message_length];   // NOLINT: non-const global by design

static void test_provizio_on_warning(const char *warning)
{
    strncpy(provizio_test_warning, warning, test_message_length - 1);
}

static void test_provizio_on_error(const char *error)
{
    strncpy(provizio_test_error, error, test_message_length - 1);
}

void test_provizio_radar_point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
                                              provizio_radar_point_cloud_api_context *context)
{
    test_provizio_radar_point_cloud_callback_data *data =
        (test_provizio_radar_point_cloud_callback_data *)context->user_data;

    ++data->called_times;

    // Shift last_point_clouds
    memmove(&data->last_point_clouds[1], &data->last_point_clouds[0],
            sizeof(provizio_radar_point_cloud) * (PROVIZIO__TEST_CALLBACK_DATA_NUM_POINT_CLOUDS - 1));

    // Update the most recent of last_point_clouds
    data->last_point_clouds[0] = *point_cloud;
}

static int32_t create_test_pointcloud_packet(provizio_radar_point_cloud_packet *packet, const uint32_t frame_index,
                                             const uint64_t timestamp, const uint16_t radar_position_id,
                                             const uint16_t radar_mode, const uint16_t total_points_in_frame,
                                             const uint16_t num_points_in_packet)
{
    const float x_meters_min = -100.0F;
    const float x_meters_max = 100.0F;
    const float x_meters_step = 12.33F;
    const float y_meters_min = -15.0F;
    const float y_meters_max = 15.0F;
    const float y_meters_step = 1.17F;
    const float z_meters_min = -2.0F;
    const float z_meters_max = 25.0F;
    const float z_meters_step = 0.47F;
    const float relative_velocity_min = -30.0F;
    const float relative_velocity_max = 30.0F;
    const float relative_velocity_step = 5.31F;
    const float ground_velocity_min = -3.0F;
    const float ground_velocity_max = 13.0F;
    const float ground_velocity_step = 3.14F;
    const float signal_to_noise_ratio_min = 2.0F;
    const float signal_to_noise_ratio_max = 50.0F;
    const float signal_to_noise_ratio_step = 3.73F;
    const float half = 0.5F;

    float x_meters = (x_meters_min + x_meters_max) * half;
    float y_meters = (y_meters_min + y_meters_max) * half;
    float z_meters = (z_meters_min + z_meters_max) * half;
    float relative_velocity = (relative_velocity_min + relative_velocity_max) * half;
    float ground_velocity = (ground_velocity_min + ground_velocity_max) * half;
    float signal_to_noise_ratio = (signal_to_noise_ratio_min + signal_to_noise_ratio_max) * half;

    memset(packet, 0, sizeof(provizio_radar_point_cloud_packet));

    provizio_set_protocol_field_uint16_t(&packet->header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&packet->header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
    provizio_set_protocol_field_uint32_t(&packet->header.frame_index, frame_index);
    provizio_set_protocol_field_uint64_t(&packet->header.timestamp, timestamp);
    provizio_set_protocol_field_uint16_t(&packet->header.radar_position_id, radar_position_id);
    provizio_set_protocol_field_uint16_t(&packet->header.radar_mode, radar_mode);
    provizio_set_protocol_field_uint16_t(&packet->header.total_points_in_frame, total_points_in_frame);
    provizio_set_protocol_field_uint16_t(&packet->header.num_points_in_packet, num_points_in_packet);

#pragma unroll(8)
    for (uint16_t j = 0; j < num_points_in_packet; ++j) // NOLINT: The loop is just fine
    {
#define PROVIZIO__NEXT_TEST_VALUE(v) ((v##_min) + fmodf((v) + (v##_step) - (v##_min), (v##_max) - (v##_min)))
        x_meters = PROVIZIO__NEXT_TEST_VALUE(x_meters);
        y_meters = PROVIZIO__NEXT_TEST_VALUE(y_meters);
        z_meters = PROVIZIO__NEXT_TEST_VALUE(z_meters);
        relative_velocity = PROVIZIO__NEXT_TEST_VALUE(relative_velocity);
        ground_velocity = PROVIZIO__NEXT_TEST_VALUE(ground_velocity);
        signal_to_noise_ratio = PROVIZIO__NEXT_TEST_VALUE(signal_to_noise_ratio);
#undef PROVIZIO__NEXT_TEST_VALUE

        provizio_set_protocol_field_float(&packet->radar_points[j].x_meters, x_meters);
        provizio_set_protocol_field_float(&packet->radar_points[j].y_meters, y_meters);
        provizio_set_protocol_field_float(&packet->radar_points[j].z_meters, z_meters);
        provizio_set_protocol_field_float(&packet->radar_points[j].radar_relative_radial_velocity_m_s,
                                          relative_velocity);
        provizio_set_protocol_field_float(&packet->radar_points[j].signal_to_noise_ratio, signal_to_noise_ratio);
        provizio_set_protocol_field_float(&packet->radar_points[j].ground_relative_radial_velocity_m_s,
                                          ground_velocity);
    }

    return 0;
}

static int32_t create_test_pointcloud_packet_v1(provizio_radar_point_cloud_packet_protocol_v1 *packet,
                                                const uint32_t frame_index, const uint64_t timestamp,
                                                const uint16_t radar_position_id, const uint16_t radar_mode,
                                                const uint16_t total_points_in_frame,
                                                const uint16_t num_points_in_packet)
{
    const float x_meters_min = -100.0F;
    const float x_meters_max = 100.0F;
    const float x_meters_step = 12.33F;
    const float y_meters_min = -15.0F;
    const float y_meters_max = 15.0F;
    const float y_meters_step = 1.17F;
    const float z_meters_min = -2.0F;
    const float z_meters_max = 25.0F;
    const float z_meters_step = 0.47F;
    const float velocity_min = -30.0F;
    const float velocity_max = 30.0F;
    const float velocity_step = 5.31F;
    const float signal_to_noise_ratio_min = 2.0F;
    const float signal_to_noise_ratio_max = 50.0F;
    const float signal_to_noise_ratio_step = 3.73F;
    const float half = 0.5F;

    float x_meters = (x_meters_min + x_meters_max) * half;
    float y_meters = (y_meters_min + y_meters_max) * half;
    float z_meters = (z_meters_min + z_meters_max) * half;
    float velocity = (velocity_min + velocity_max) * half;
    float signal_to_noise_ratio = (signal_to_noise_ratio_min + signal_to_noise_ratio_max) * half;

    memset(packet, 0, sizeof(provizio_radar_point_cloud_packet_protocol_v1));

    provizio_set_protocol_field_uint16_t(&packet->header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&packet->header.protocol_header.protocol_version, 1);
    provizio_set_protocol_field_uint32_t(&packet->header.frame_index, frame_index);
    provizio_set_protocol_field_uint64_t(&packet->header.timestamp, timestamp);
    provizio_set_protocol_field_uint16_t(&packet->header.radar_position_id, radar_position_id);
    provizio_set_protocol_field_uint16_t(&packet->header.radar_mode, radar_mode);
    provizio_set_protocol_field_uint16_t(&packet->header.total_points_in_frame, total_points_in_frame);
    provizio_set_protocol_field_uint16_t(&packet->header.num_points_in_packet, num_points_in_packet);

#pragma unroll(8)
    for (uint16_t j = 0; j < num_points_in_packet; ++j) // NOLINT: The loop is just fine
    {
#define PROVIZIO__NEXT_TEST_VALUE(v) ((v##_min) + fmodf((v) + (v##_step) - (v##_min), (v##_max) - (v##_min)))
        x_meters = PROVIZIO__NEXT_TEST_VALUE(x_meters);
        y_meters = PROVIZIO__NEXT_TEST_VALUE(y_meters);
        z_meters = PROVIZIO__NEXT_TEST_VALUE(z_meters);
        velocity = PROVIZIO__NEXT_TEST_VALUE(velocity);
        signal_to_noise_ratio = PROVIZIO__NEXT_TEST_VALUE(signal_to_noise_ratio);
#undef PROVIZIO__NEXT_TEST_VALUE

        provizio_set_protocol_field_float(&packet->radar_points[j].x_meters, x_meters);
        provizio_set_protocol_field_float(&packet->radar_points[j].y_meters, y_meters);
        provizio_set_protocol_field_float(&packet->radar_points[j].z_meters, z_meters);
        provizio_set_protocol_field_float(&packet->radar_points[j].radar_relative_radial_velocity_m_s, velocity);
        provizio_set_protocol_field_float(&packet->radar_points[j].signal_to_noise_ratio, signal_to_noise_ratio);
    }

    return 0;
}

static void test_provizio_radar_point_cloud_packet_size(void)
{
    provizio_radar_point_cloud_packet_header header;
    memset(&header, 0, sizeof(header));

    provizio_set_protocol_field_uint16_t(&header.num_points_in_packet, 0);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header), provizio_radar_point_cloud_packet_size(&header));

    provizio_set_protocol_field_uint16_t(&header.num_points_in_packet, 1);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header) + sizeof(provizio_radar_point),
                             provizio_radar_point_cloud_packet_size(&header));

    provizio_set_protocol_field_uint16_t(&header.num_points_in_packet, 2);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header) + sizeof(provizio_radar_point) * 2,
                             provizio_radar_point_cloud_packet_size(&header));

    provizio_set_protocol_field_uint16_t(&header.num_points_in_packet, PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header) + sizeof(provizio_radar_point) * PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET,
                             provizio_radar_point_cloud_packet_size(&header));

    provizio_set_on_warning(&test_provizio_on_warning);
    provizio_set_protocol_field_uint16_t(&header.num_points_in_packet, PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET + 1);
    TEST_ASSERT_EQUAL_UINT64(0, provizio_radar_point_cloud_packet_size(&header));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_point_cloud_packet_size: num_points_in_packet exceeds "
                             "PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET!",
                             provizio_test_warning);
    provizio_set_on_warning(NULL);
}

static void test_provizio_check_radar_point_cloud_packet(void)
{
    const uint16_t num_points = 10;

    provizio_set_on_error(&test_provizio_on_error);

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);

    provizio_radar_point_cloud_packet packet;
    memset(&packet, 0, sizeof(packet));

    // Check failure due to size < sizeof(provizio_radar_api_protocol_header)
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_radar_point_cloud_packet(&api_context, &packet,
                                                                     sizeof(provizio_radar_api_protocol_header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: insufficient packet_size", provizio_test_error);

    // Check incorrect packet type
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE + 1);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_radar_point_cloud_packet(&api_context, &packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: unexpected packet_type", provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);

    // Check incompatible protocol version
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION + 1);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_radar_point_cloud_packet(&api_context, &packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: Incompatible protocol version",
                             provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);

    // Check failure due to size < sizeof(provizio_radar_point_cloud_packet_header)
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_radar_point_cloud_packet(
                                &api_context, &packet, sizeof(provizio_radar_point_cloud_packet_header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: insufficient packet_size", provizio_test_error);

    // Check size mismatch as specified and as estimated by header (i.e. number of points in packet)
    provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, num_points);
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_radar_point_cloud_packet(
            &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header) - 1)); // 1 byte less
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: incorrect packet_size", provizio_test_error);
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_radar_point_cloud_packet(
            &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header) + 1)); // 1 byte more
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: incorrect packet_size", provizio_test_error);

    // Check fails on radar_position_id = provizio_radar_position_unknown
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, provizio_radar_position_unknown);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_radar_point_cloud_packet(
                                &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: the value of radar_position_id can't be "
                             "provizio_radar_position_unknown",
                             provizio_test_error);

    // Empty frame
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, provizio_radar_position_front_center);
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, 0);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_SKIPPED,
                            provizio_handle_radar_point_cloud_packet(
                                &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));

    // All check pass well
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, provizio_radar_position_front_center);
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
    TEST_ASSERT_EQUAL_INT32(0, provizio_handle_radar_point_cloud_packet(
                                   &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));

    free(callback_data);

    provizio_set_on_error(NULL);
}

static void test_provizio_handle_radar_point_cloud_packet_warnings(void)
{
    const uint32_t frame_index = 1;
    const uint64_t timestamp = 2;
    const uint16_t radar_position_id = provizio_radar_position_front_center;
    const uint16_t num_points = 4;

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(NULL, NULL, &api_context);

    provizio_radar_point_cloud_packet packet;
    memset(&packet, 0, sizeof(packet));
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
    provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);
    provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_id);
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
    provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, 1);
    provizio_set_protocol_field_uint16_t(&packet.header.radar_mode, provizio_radar_mode_medium_range);

    // Send first point
    TEST_ASSERT_EQUAL_INT32(0, provizio_handle_radar_point_cloud_packet(
                                   &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));

    provizio_set_on_warning(&test_provizio_on_warning);

    // num_points_expected mismatch
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points + 1);
    TEST_ASSERT_EQUAL_INT32(0, provizio_handle_radar_point_cloud_packet(
                                   &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_get_point_cloud_being_received: num_points_expected mismatch across different "
                             "packets of the same frame",
                             provizio_test_warning);
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);

    // radar_mode mismatch
    provizio_set_protocol_field_uint16_t(&packet.header.radar_mode, provizio_radar_mode_ultra_long_range);
    TEST_ASSERT_EQUAL_INT32(0, provizio_handle_radar_point_cloud_packet(
                                   &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING(
        "provizio_get_point_cloud_being_received: radar_mode mismatch across different packets of the same frame",
        provizio_test_warning);

    provizio_set_on_warning(NULL);
}

static void test_provizio_handle_radars_point_cloud_packet_bad_packet(void)
{
    provizio_radar_point_cloud_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_radars_point_cloud_packet(
                                NULL, 0, &bad_packet, sizeof(bad_packet.header.protocol_header) - 1)); // Size too small
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: insufficient packet_size", provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_radar_point_cloud_api_context_assign(void)
{
    const uint32_t frame_index = 17;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_front_left;
    const uint16_t num_points = 20;

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(NULL, NULL, &api_context);

    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_point_cloud_api_context_assign(&api_context, radar_position_id));

    provizio_radar_point_cloud_packet packet;
    memset(&packet, 0, sizeof(packet));

    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
    provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);
    provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
    provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, num_points - 1);

    // Expected radar position
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_id);
    TEST_ASSERT_EQUAL_INT32(0, provizio_handle_radar_point_cloud_packet(
                                   &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));

    // Unexpected radar position
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_id + 1);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_SKIPPED,
                            provizio_handle_radar_point_cloud_packet(
                                &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));

    // Reassign to the same position - OK
    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_point_cloud_api_context_assign(&api_context, radar_position_id));

    // Reassign to another position once assigned - Fails
    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_NOT_PERMITTED,
                            provizio_radar_point_cloud_api_context_assign(&api_context, radar_position_id + 1));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_point_cloud_api_context_assign: already assigned", provizio_test_error);
    provizio_set_on_error(NULL);

    // Unassign - Fails
    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_ARGUMENT, provizio_radar_point_cloud_api_context_assign(
                                                     &api_context, provizio_radar_position_unknown));
    TEST_ASSERT_EQUAL_STRING(
        "provizio_radar_point_cloud_api_context_assign: can't assign to provizio_radar_position_unknown",
        provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_size(void)
{
    provizio_radar_point_cloud_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_SKIPPED,
                            provizio_handle_possible_radar_point_cloud_packet(
                                NULL, &bad_packet, sizeof(provizio_radar_point_cloud_packet_header) - 1));
}

static void test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_type(void)
{
    provizio_radar_point_cloud_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    provizio_set_protocol_field_uint16_t(&bad_packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE + 1);

    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_SKIPPED,
                            provizio_handle_possible_radar_point_cloud_packet(NULL, &bad_packet, sizeof(bad_packet)));
}

static void test_provizio_handle_possible_radar_point_cloud_packet_wrong_protocol_version(void)
{
    provizio_radar_point_cloud_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    provizio_set_protocol_field_uint16_t(&bad_packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&bad_packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION + 1);

    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_possible_radar_point_cloud_packet(NULL, &bad_packet, sizeof(bad_packet)));
}

static void test_provizio_handle_possible_radar_point_cloud_packet_ok(void)
{
    const uint32_t frame_index = 1000;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t num_points = 11;
    const uint16_t points_in_packet = 10;

    provizio_radar_point_cloud_packet packet;
    memset(&packet, 0, sizeof(packet));

    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
    provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);
    provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_id);
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
    provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, points_in_packet);

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(NULL, NULL, &api_context);

    TEST_ASSERT_EQUAL_INT32(0, provizio_handle_possible_radar_point_cloud_packet(
                                   &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));
}

static void test_provizio_handle_possible_radars_point_cloud_packet_ok(void)
{
    const uint32_t frame_index = 1000;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_ids[2] = {provizio_radar_position_rear_left, provizio_radar_position_rear_right};
    const uint16_t num_points = 11;
    const uint16_t points_in_packet = 10;

    const size_t num_contexts = 2;
    provizio_radar_point_cloud_api_context *api_contexts =
        (provizio_radar_point_cloud_api_context *)malloc(sizeof(provizio_radar_point_cloud_api_context) * num_contexts);
    TEST_ASSERT_NOT_EQUAL(NULL, api_contexts);
    provizio_radar_point_cloud_api_contexts_init(NULL, NULL, api_contexts, num_contexts);

    for (size_t i = 0; i < num_contexts; ++i) // NOLINT: Don't unroll the loop
    {
        provizio_radar_point_cloud_packet packet;
        memset(&packet, 0, sizeof(packet));

        provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                             PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
        provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                             PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
        provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);
        provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
        provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_ids[i]);
        provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
        provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, points_in_packet);

        TEST_ASSERT_EQUAL_INT32(
            0, provizio_handle_possible_radars_point_cloud_packet(
                   api_contexts, num_contexts, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));
    }
    free(api_contexts);
}

static void test_provizio_handle_possible_radars_point_cloud_packet_ground_velocity(void)
{
    const uint32_t frame_index = 1000;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_ids[2] = {provizio_radar_position_rear_left, provizio_radar_position_rear_right};
    const uint16_t radar_modes[2] = {provizio_radar_mode_long_range, provizio_radar_mode_medium_range};
    const uint16_t num_points = 10;
    const uint16_t points_in_packet = 10;

    const size_t num_contexts = 2;
    provizio_radar_point_cloud_api_context *api_contexts =
        (provizio_radar_point_cloud_api_context *)malloc(sizeof(provizio_radar_point_cloud_api_context) * num_contexts);
    TEST_ASSERT_NOT_EQUAL(NULL, api_contexts);

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    TEST_ASSERT_NOT_EQUAL(NULL, callback_data);
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));

    provizio_radar_point_cloud_api_contexts_init(&test_provizio_radar_point_cloud_callback, callback_data, api_contexts,
                                                 num_contexts);

    for (size_t i = 0; i < num_contexts; ++i) // NOLINT: Don't unroll the loop
    {
        provizio_radar_point_cloud_packet packet;
        memset(&packet, 0, sizeof(packet));

        TEST_ASSERT_EQUAL_INT32(0, // NOLINT
                                create_test_pointcloud_packet(&packet, frame_index, timestamp, radar_position_ids[i],
                                                              radar_modes[i], num_points, points_in_packet));

        TEST_ASSERT_EQUAL_INT32(
            0, // NOLINT
            provizio_handle_possible_radars_point_cloud_packet(api_contexts, num_contexts, &packet,
                                                               provizio_radar_point_cloud_packet_size(&packet.header)));

        TEST_ASSERT_EQUAL_FLOAT( // NOLINT
            8.14, callback_data->last_point_clouds[0].radar_points[0].ground_relative_radial_velocity_m_s);
    }

    free(api_contexts);
    free(callback_data);
}

static void test_provizio_check_radar_point_cloud_packet_v1(void)
{
    TEST_ASSERT_EQUAL_INT32(0, offsetof(provizio_radar_point_cloud_packet_protocol_v1, header));
    TEST_ASSERT_EQUAL_INT32(24, offsetof(provizio_radar_point_cloud_packet_protocol_v1, radar_points));
    TEST_ASSERT_EQUAL_INT32(1464, sizeof(provizio_radar_point_cloud_packet_protocol_v1));
    TEST_ASSERT_EQUAL_INT32(0, offsetof(provizio_radar_point_protocol_v1, x_meters));
    TEST_ASSERT_EQUAL_INT32(4, offsetof(provizio_radar_point_protocol_v1, y_meters));
    TEST_ASSERT_EQUAL_INT32(8, offsetof(provizio_radar_point_protocol_v1, z_meters));
    TEST_ASSERT_EQUAL_INT32(12, offsetof(provizio_radar_point_protocol_v1, radar_relative_radial_velocity_m_s));
    TEST_ASSERT_EQUAL_INT32(16, offsetof(provizio_radar_point_protocol_v1, signal_to_noise_ratio));
    TEST_ASSERT_EQUAL_INT32(20, sizeof(provizio_radar_point_protocol_v1));
}

static void test_provizio_handle_possible_radars_point_cloud_packet_v1(void)
{
    const uint32_t frame_index = 1000;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_ids[2] = {provizio_radar_position_rear_left, provizio_radar_position_rear_right};
    const uint16_t radar_modes[2] = {provizio_radar_mode_long_range, provizio_radar_mode_medium_range};
    const uint16_t points_in_packet = 72;
    const uint16_t num_points = 72;

    const size_t num_contexts = 2;
    provizio_radar_point_cloud_api_context *api_contexts =
        (provizio_radar_point_cloud_api_context *)malloc(sizeof(provizio_radar_point_cloud_api_context) * num_contexts);
    TEST_ASSERT_NOT_EQUAL(NULL, api_contexts);

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    TEST_ASSERT_NOT_EQUAL(NULL, callback_data);
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));

    provizio_radar_point_cloud_api_contexts_init(&test_provizio_radar_point_cloud_callback, callback_data, api_contexts,
                                                 num_contexts);

    for (size_t i = 0; i < num_contexts; ++i) // NOLINT: Don't unroll the loop
    {
        provizio_radar_point_cloud_packet_protocol_v1 packet;
        memset(&packet, 0, sizeof(packet));

        TEST_ASSERT_EQUAL_INT32(0, // NOLINT
                                create_test_pointcloud_packet_v1(&packet, frame_index, timestamp, radar_position_ids[i],
                                                                 radar_modes[i], num_points, points_in_packet));

        TEST_ASSERT_EQUAL_INT32( // NOLINT
            0, provizio_handle_possible_radars_point_cloud_packet(
                   api_contexts, num_contexts, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));

        for (size_t j = 0; j < num_points; ++j)
        {
            TEST_ASSERT_EQUAL_FLOAT( // NOLINT
                nanf(""), callback_data->last_point_clouds[0].radar_points[j].ground_relative_radial_velocity_m_s);
        }

        // test greater than max allowed radar points in udp packet for protocol version 1 (code coverage)
        packet.header.frame_index += 1;
        packet.header.num_points_in_packet += 1;

        TEST_ASSERT_EQUAL_INT32( // NOLINT
            PROVIZIO_E_SKIPPED,
            provizio_handle_possible_radars_point_cloud_packet(api_contexts, num_contexts, &packet,
                                                               provizio_radar_point_cloud_packet_size(&packet.header)));
    }

    free(api_contexts);
    free(callback_data);
}

int provizio_run_test_radar_point_cloud(void)
{
    memset(provizio_test_warning, 0, sizeof(provizio_test_warning));
    memset(provizio_test_error, 0, sizeof(provizio_test_error));

    UNITY_BEGIN();

    RUN_TEST(test_provizio_radar_point_cloud_packet_size);
    RUN_TEST(test_provizio_check_radar_point_cloud_packet);
    RUN_TEST(test_provizio_handle_radar_point_cloud_packet_warnings);
    RUN_TEST(test_provizio_handle_radars_point_cloud_packet_bad_packet);
    RUN_TEST(test_provizio_radar_point_cloud_api_context_assign);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_size);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_type);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_protocol_version);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_ok);
    RUN_TEST(test_provizio_handle_possible_radars_point_cloud_packet_ok);
    RUN_TEST(test_provizio_handle_possible_radars_point_cloud_packet_ground_velocity);
    RUN_TEST(test_provizio_handle_possible_radars_point_cloud_packet_v1);
    RUN_TEST(test_provizio_check_radar_point_cloud_packet_v1);

    return UNITY_END();
}
