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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unity/unity.h"

#include "provizio/radar_api/errno.h"
#include "provizio/radar_api/radar_point_cloud.h"
#include "provizio/socket.h"
#include "provizio/util.h"

#include "test_point_cloud_callbacks.h"

#define PROVIZIO__TEST_MAX_MESSAGE_LENGTH 1024
static char provizio_test_warning[PROVIZIO__TEST_MAX_MESSAGE_LENGTH]; // NOLINT: non-const global by design
static char provizio_test_error[PROVIZIO__TEST_MAX_MESSAGE_LENGTH];   // NOLINT: non-const global by design

static void test_provizio_on_warning(const char *warning)
{
    strncpy(provizio_test_warning, warning, PROVIZIO__TEST_MAX_MESSAGE_LENGTH - 1);
}

static void test_provizio_on_error(const char *error)
{
    strncpy(provizio_test_error, error, PROVIZIO__TEST_MAX_MESSAGE_LENGTH - 1);
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

    // Check failure due to size < sizeof(provizio_radar_point_cloud_packet_protocol_header)
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_radar_point_cloud_packet(
                                &api_context, &packet, sizeof(provizio_radar_point_cloud_packet_protocol_header) - 1));
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

    provizio_radar_point_cloud_api_context api_contexts[2];
    const size_t num_contexts = sizeof(api_contexts) / sizeof(api_contexts[0]);
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

        provizio_radar_point_cloud_api_context api_context;
        provizio_radar_point_cloud_api_context_init(NULL, NULL, &api_context);

        TEST_ASSERT_EQUAL_INT32(
            0, provizio_handle_possible_radars_point_cloud_packet(
                   api_contexts, num_contexts, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));
    }
}

int provizio_run_test_radar_point_cloud(void)
{
    memset(provizio_test_warning, 0, sizeof(provizio_test_warning));
    memset(provizio_test_error, 0, sizeof(provizio_test_error));

    UNITY_BEGIN();

    RUN_TEST(test_provizio_radar_point_cloud_packet_size);
    RUN_TEST(test_provizio_check_radar_point_cloud_packet);
    RUN_TEST(test_provizio_handle_radars_point_cloud_packet_bad_packet);
    RUN_TEST(test_provizio_radar_point_cloud_api_context_assign);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_size);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_type);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_protocol_version);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_ok);
    RUN_TEST(test_provizio_handle_possible_radars_point_cloud_packet_ok);

    return UNITY_END();
}
