// Copyright 2025 Provizio Ltd.
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

#include "provizio/radar_api/entities.h"
#include "provizio/radar_api/errno.h"
#include "provizio/util.h"

#include "test_entities_callbacks.h"
#include "test_entities_helpers.h"

enum
{
    test_message_length = 1024
};
static char provizio_test_warning[test_message_length]; // NOLINT: non-const global by design
static char provizio_test_error[test_message_length];   // NOLINT: non-const global by design

static const float provizio_test_entities_offset_step = 1.0F;
static const float provizio_test_entities_reset_offset_multiplier = 2.0F;
static const float provizio_test_entities_helper_min_value = 5.0F;
static const float provizio_test_entities_helper_inverted_min_value = 10.0F;
static const float provizio_test_entities_zero = 0.0F;
static const float provizio_test_entities_helper_tolerance = 0.0001F;

static void test_provizio_on_warning(const char *warning)
{
    strncpy(provizio_test_warning, warning, test_message_length - 1);
}

static void test_provizio_on_error(const char *error)
{
    strncpy(provizio_test_error, error, test_message_length - 1);
}

void test_provizio_radar_entities_callback(const provizio_radar_entities_frame *entities_frame,
                                           provizio_radar_entities_api_context *context)
{
    test_provizio_radar_entities_callback_data *data = (test_provizio_radar_entities_callback_data *)context->user_data;

    ++data->called_times;

    memmove(&data->last_entities_frames[1], &data->last_entities_frames[0],
            sizeof(provizio_radar_entities_frame) * (PROVIZIO__TEST_CALLBACK_DATA_NUM_ENTITIES_FRAMES - 1));

    data->last_entities_frames[0] = *entities_frame;
}

static void test_provizio_radar_entities_packet_size(void)
{
    provizio_radar_entities_packet_header header;
    memset(&header, 0, sizeof(header));

    provizio_set_protocol_field_uint16_t(&header.num_entities_in_packet, 0);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header), provizio_radar_entities_packet_size(&header));

    provizio_set_protocol_field_uint16_t(&header.num_entities_in_packet, 1);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header) + sizeof(provizio_radar_entity),
                             provizio_radar_entities_packet_size(&header));

    provizio_set_protocol_field_uint16_t(&header.num_entities_in_packet, 2);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header) + sizeof(provizio_radar_entity) * 2, // NOLINT
                             provizio_radar_entities_packet_size(&header));

    provizio_set_protocol_field_uint16_t(&header.num_entities_in_packet, PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET);
    TEST_ASSERT_EQUAL_UINT64(sizeof(header) +
                                 (sizeof(provizio_radar_entity) * PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET),
                             provizio_radar_entities_packet_size(&header));

    provizio_set_on_warning(&test_provizio_on_warning);
    provizio_set_protocol_field_uint16_t(&header.num_entities_in_packet,
                                         PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET + 1);
    TEST_ASSERT_EQUAL_UINT64(0, provizio_radar_entities_packet_size(&header));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_entities_packet_size: num_entities_in_packet exceeds "
                             "PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET!",
                             provizio_test_warning);
    provizio_set_on_warning(NULL);
}

static void test_provizio_check_radar_entities_packet(void)
{
    const uint16_t num_entities = 8;

    provizio_set_on_error(&test_provizio_on_error);

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(&test_provizio_radar_entities_callback, callback_data, &api_context);
    TEST_ASSERT_NOT_EQUAL(NULL, api_context.callback);

    provizio_radar_entities_packet packet;
    memset(&packet, 0, sizeof(packet));

    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_entities_packet(&api_context, &packet, sizeof(provizio_radar_api_protocol_header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: insufficient packet_size", provizio_test_error);

    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_ENTITIES_PACKET_TYPE + 1);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_entities_packet(&api_context, &packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: unexpected packet_type", provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_ENTITIES_PACKET_TYPE);

    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_ENTITY_PROTOCOL_VERSION + 1);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_entities_packet(&api_context, &packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: Incompatible protocol version",
                             provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_ENTITY_PROTOCOL_VERSION);

    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_entities_packet(&api_context, &packet, sizeof(provizio_radar_entities_packet_header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: insufficient packet_size", provizio_test_error);

    provizio_set_protocol_field_uint16_t(&packet.header.num_entities_in_packet,
                                         PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET + 1);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_entities_packet(&api_context, &packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: incorrect num_entities_in_packet",
                             provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.num_entities_in_packet, num_entities);

    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_entities_packet(&api_context, &packet,
                                                            provizio_radar_entities_packet_size(&packet.header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: incorrect packet_size", provizio_test_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL,
                            provizio_handle_entities_packet(&api_context, &packet,
                                                            provizio_radar_entities_packet_size(&packet.header) + 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: incorrect packet_size", provizio_test_error);

    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, provizio_radar_position_unknown);
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: the value of radar_position_id can't be "
                             "provizio_radar_position_unknown",
                             provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, provizio_radar_position_front_center);

    provizio_set_protocol_field_uint16_t(&packet.header.total_entities_in_frame,
                                         PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME + 1);
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: total_entities_in_frame exceeds "
                             "PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME",
                             provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.total_entities_in_frame, num_entities);

    provizio_set_protocol_field_uint16_t(&packet.header.num_entities_in_packet, num_entities + 1);
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: num_entities_in_packet exceeds "
                             "total_entities_in_frame",
                             provizio_test_error);
    provizio_set_protocol_field_uint16_t(&packet.header.num_entities_in_packet, num_entities);

    free(callback_data);
    provizio_set_on_error(NULL);
}

static void test_provizio_handle_entities_packet_warnings(void)
{
    const uint32_t frame_index = 1;
    const uint64_t timestamp = 2;
    const uint16_t radar_position_id = provizio_radar_position_front_center;
    const uint16_t radar_range = provizio_radar_range_long;
    const uint16_t num_entities = 4;

    provizio_radar_entities_api_context api_context;
    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));
    provizio_radar_entities_api_context_init(&test_provizio_radar_entities_callback, callback_data, &api_context);

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, frame_index, timestamp, radar_position_id,
                                                                    radar_range, num_entities, 1, 0.0F));

    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    provizio_set_on_warning(&test_provizio_on_warning);

    packet.radar_entities[0].x_meters +=
        provizio_test_entities_offset_step; // So the packet is not detected as duplicated
    provizio_set_protocol_field_uint16_t(&packet.header.total_entities_in_frame, num_entities + 1);
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_get_entities_frame_being_received: num_entities_expected mismatch across "
                             "different packets of the same frame",
                             provizio_test_warning);
    provizio_set_protocol_field_uint16_t(&packet.header.total_entities_in_frame, num_entities);

    packet.radar_entities[0].x_meters +=
        provizio_test_entities_offset_step; // So the packet is not detected as duplicated
    provizio_set_protocol_field_uint16_t(&packet.header.radar_range, provizio_radar_range_medium);
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_get_entities_frame_being_received: radar_range mismatch across different "
                             "packets of the same frame",
                             provizio_test_warning);

    provizio_set_on_warning(NULL);
    free(callback_data);
}

static void test_provizio_return_entities_frame_returns_older_first(void)
{
    const uint32_t frame_indices[2] = {10U, 11U};
    const uint16_t radar_position_id = provizio_radar_position_front_left;
    const uint16_t total_entities_in_first_frame = 3;
    const uint16_t total_entities_in_second_frame = 1;

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    TEST_ASSERT_NOT_EQUAL(NULL, callback_data);
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(&test_provizio_radar_entities_callback, callback_data, &api_context);

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(
                                   &packet, frame_indices[0], 100U, radar_position_id, provizio_radar_range_short,
                                   total_entities_in_first_frame, total_entities_in_first_frame - 1, 0.0F));
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_INT32(0, callback_data->called_times);

    TEST_ASSERT_EQUAL_INT32(
        0, provizio_test_create_entities_packet(&packet, frame_indices[1], 200U, radar_position_id,
                                                provizio_radar_range_short, total_entities_in_second_frame,
                                                total_entities_in_second_frame, provizio_test_entities_offset_step));
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    TEST_ASSERT_EQUAL_INT32(2, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[0], callback_data->last_entities_frames[1].frame_index);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[1], callback_data->last_entities_frames[0].frame_index);

    free(callback_data);
}

static void test_provizio_handle_entities_packet_empty_frame(void)
{
    const uint32_t frame_index = 42U;
    const uint64_t timestamp = 200U;
    const uint16_t radar_position_id = provizio_radar_position_rear_right;

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    TEST_ASSERT_NOT_EQUAL(NULL, callback_data);
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(&test_provizio_radar_entities_callback, callback_data, &api_context);

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, frame_index, timestamp, radar_position_id,
                                                                    provizio_radar_range_medium, 0, 0, 0.0F));

    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_SKIPPED,
        provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_INT32(0, callback_data->called_times);

    free(callback_data);
}

static void test_provizio_handle_radars_entities_packet_reuses_existing_context(void)
{
    const uint16_t radar_position_id = provizio_radar_position_front_right;
    const size_t num_contexts = 2;

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    TEST_ASSERT_NOT_EQUAL(NULL, callback_data);
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context *contexts = (provizio_radar_entities_api_context *)malloc(
        sizeof(provizio_radar_entities_api_context) * num_contexts); // NOLINT: intentional heap allocation
    TEST_ASSERT_NOT_EQUAL(NULL, contexts);
    provizio_radar_entities_api_contexts_init(&test_provizio_radar_entities_callback, callback_data, contexts,
                                              num_contexts);

    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_entities_api_context_assign(&contexts[0], radar_position_id));

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, 5U, 6U, radar_position_id,
                                                                    provizio_radar_range_short, 1, 1, 0.0F));

    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_radars_entities_packet(contexts, num_contexts, &packet,
                                                  provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_INT32(1, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT16(radar_position_id, callback_data->last_entities_frames[0].radar_position_id);

    free(contexts);
    free(callback_data);
}

static void test_provizio_handle_radars_entities_packet_out_of_contexts(void)
{
    const uint16_t radar_position_id = provizio_radar_position_front_left;
    const uint16_t unexpected_radar_position_id = provizio_radar_position_rear_left;
    const size_t num_contexts = 1;

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    TEST_ASSERT_NOT_EQUAL(NULL, callback_data);
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context *contexts = (provizio_radar_entities_api_context *)malloc(
        sizeof(provizio_radar_entities_api_context) * num_contexts); // NOLINT: intentional heap allocation
    TEST_ASSERT_NOT_EQUAL(NULL, contexts);
    provizio_radar_entities_api_contexts_init(&test_provizio_radar_entities_callback, callback_data, contexts,
                                              num_contexts);

    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_entities_api_context_assign(&contexts[0], radar_position_id));

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, 100U, 200U, unexpected_radar_position_id,
                                                                    provizio_radar_range_medium, 1, 1, 0.0F));

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_OUT_OF_CONTEXTS,
                            provizio_handle_radars_entities_packet(
                                contexts, num_contexts, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_get_radar_entities_api_context_by_position_id: Out of available contexts",
                             provizio_test_error);
    provizio_set_on_error(NULL);

    free(contexts);
    free(callback_data);
}

static void test_provizio_test_next_value_handles_nonpositive_range(void)
{
    TEST_ASSERT_FLOAT_WITHIN(
        provizio_test_entities_helper_tolerance, provizio_test_entities_helper_min_value,
        provizio_test_next_value(provizio_test_entities_offset_step, provizio_test_entities_helper_min_value,
                                 provizio_test_entities_helper_min_value, provizio_test_entities_offset_step));

    TEST_ASSERT_FLOAT_WITHIN(
        provizio_test_entities_helper_tolerance, provizio_test_entities_helper_inverted_min_value,
        provizio_test_next_value(provizio_test_entities_zero, provizio_test_entities_helper_inverted_min_value,
                                 provizio_test_entities_helper_min_value, provizio_test_entities_offset_step));
}

static void test_provizio_radar_entities_api_context_assign(void)
{
    const uint32_t frame_index = 5;
    const uint64_t timestamp = 10;
    const uint16_t radar_position_id = provizio_radar_position_front_left;
    const uint16_t num_entities = 3;

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(NULL, NULL, &api_context);

    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_entities_api_context_assign(&api_context, radar_position_id));

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, frame_index, timestamp, radar_position_id,
                                                                    provizio_radar_range_medium, num_entities,
                                                                    num_entities - 1, 0.0F));

    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_id + 1);
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_SKIPPED,
        provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_entities_api_context_assign(&api_context, radar_position_id));

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_NOT_SUPPORTED,
                            provizio_radar_entities_api_context_assign(&api_context, radar_position_id + 1));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_entities_api_context_assign: already assigned", provizio_test_error);

    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_ARGUMENT,
                            provizio_radar_entities_api_context_assign(&api_context, provizio_radar_position_unknown));
    TEST_ASSERT_EQUAL_STRING(
        "provizio_radar_entities_api_context_assign: can't assign to provizio_radar_position_unknown",
        provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_handle_radars_entities_packet_bad_packet(void)
{
    provizio_radar_entities_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_radars_entities_packet(NULL, 0, &bad_packet, sizeof(bad_packet.header.protocol_header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_entities_packet: insufficient packet_size", provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_handle_possible_radar_entities_packet_wrong_packet_size(void)
{
    provizio_radar_entities_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_SKIPPED,
                            provizio_handle_possible_radar_entities_packet(
                                NULL, &bad_packet, sizeof(provizio_radar_entities_packet_header) - 1));
}

static void test_provizio_handle_possible_radar_entities_packet_wrong_packet_type(void)
{
    provizio_radar_entities_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    provizio_set_protocol_field_uint16_t(&bad_packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_ENTITIES_PACKET_TYPE + 1);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_SKIPPED, provizio_handle_possible_radar_entities_packet(
                                                    NULL, &bad_packet, sizeof(bad_packet.header)));
}

static void test_provizio_handle_possible_radar_entities_packet_wrong_protocol_version(void)
{
    provizio_radar_entities_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    provizio_set_protocol_field_uint16_t(&bad_packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_ENTITIES_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&bad_packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_ENTITY_PROTOCOL_VERSION + 1);

    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL, provizio_handle_possible_radar_entities_packet(
                                                     NULL, &bad_packet, sizeof(bad_packet.header)));
}

static void test_provizio_handle_possible_radar_entities_packet_ok(void)
{
    const uint16_t num_entities = 6;

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(&test_provizio_radar_entities_callback, callback_data, &api_context);

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, 10, 20, provizio_radar_position_rear_left,
                                                                    provizio_radar_range_long, num_entities,
                                                                    num_entities, 0.0F));

    TEST_ASSERT_EQUAL_INT32(0, provizio_handle_possible_radar_entities_packet(
                                   &api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    free(callback_data);
}

static void test_provizio_handle_possible_radars_entities_packet_ok(void)
{
    const uint16_t radar_position_ids[2] = {provizio_radar_position_rear_left, provizio_radar_position_rear_right};
    const uint16_t num_entities = 5;
    const size_t num_contexts = sizeof(radar_position_ids) / sizeof(radar_position_ids[0]);

    provizio_radar_entities_api_context *contexts =
        (provizio_radar_entities_api_context *)malloc(sizeof(provizio_radar_entities_api_context) * num_contexts);
    TEST_ASSERT_NOT_EQUAL(NULL, contexts);
    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));
    provizio_radar_entities_api_contexts_init(&test_provizio_radar_entities_callback, callback_data, contexts,
                                              num_contexts);
    TEST_ASSERT_NOT_EQUAL(NULL, contexts[0].callback);

    for (size_t i = 0; i < num_contexts; ++i)
    {
        provizio_radar_entities_packet packet;
        TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, 10, 20, radar_position_ids[i],
                                                                        provizio_radar_range_short, num_entities,
                                                                        num_entities, (float)i));

        TEST_ASSERT_EQUAL_INT32(
            0, provizio_handle_possible_radars_entities_packet(contexts, num_contexts, &packet,
                                                               provizio_radar_entities_packet_size(&packet.header)));
    }

    free(contexts);
    free(callback_data);
}

static void test_provizio_handle_entities_packet_frame_indices_overflow(void)
{
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint32_t frame_indices[3] = {0xfffffffeU, 0xffffffffU, 0};
    const uint16_t num_entities = 6;

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(&test_provizio_radar_entities_callback, callback_data, &api_context);

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, frame_indices[0], 1, radar_position_id,
                                                                    provizio_radar_range_medium, num_entities,
                                                                    num_entities, 0.0F));
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(
                                   &packet, frame_indices[1], 2, radar_position_id, provizio_radar_range_medium,
                                   num_entities, num_entities - 1, provizio_test_entities_offset_step));
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    provizio_set_on_warning(&test_provizio_on_warning);
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_test_create_entities_packet(
               &packet, frame_indices[2], 3, radar_position_id, provizio_radar_range_medium, num_entities, num_entities,
               provizio_test_entities_offset_step * provizio_test_entities_reset_offset_multiplier));
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING(
        "provizio_get_entities_frame_being_received: frame indices overflow detected - resetting API state",
        provizio_test_warning);
    provizio_set_on_warning(NULL);

    TEST_ASSERT_EQUAL_INT32(2, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[0], callback_data->last_entities_frames[1].frame_index);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[2], callback_data->last_entities_frames[0].frame_index);

    free(callback_data);
}

static void test_provizio_handle_entities_packet_drop_obsolete_incomplete_frame(void)
{
    const uint16_t radar_position_id = provizio_radar_position_front_left;
    const uint32_t frame_indices[3] = {17, 18, 19};
    const uint16_t num_entities = 10;

    test_provizio_radar_entities_callback_data *callback_data =
        (test_provizio_radar_entities_callback_data *)malloc(sizeof(test_provizio_radar_entities_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_entities_callback_data));

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(&test_provizio_radar_entities_callback, callback_data, &api_context);

    provizio_radar_entities_packet packet;
    for (size_t i = 0; i < sizeof(frame_indices) / sizeof(frame_indices[0]); ++i)
    {
        TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, frame_indices[i], (uint64_t)i,
                                                                        radar_position_id, provizio_radar_range_medium,
                                                                        num_entities, num_entities - 1, (float)i));
        TEST_ASSERT_EQUAL_INT32(0, provizio_handle_entities_packet(
                                       &api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    }

    TEST_ASSERT_EQUAL_INT32(1, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[0], callback_data->last_entities_frames[0].frame_index);
    TEST_ASSERT_EQUAL_UINT16(num_entities, callback_data->last_entities_frames[0].num_entities_expected);
    TEST_ASSERT_EQUAL_UINT16(num_entities - 1, callback_data->last_entities_frames[0].num_entities_received);

    free(callback_data);
}

static void test_provizio_handle_entities_packet_too_many_entities(void)
{
    const uint16_t radar_position_id = provizio_radar_position_front_center;
    const uint16_t num_entities = 6;

    provizio_radar_entities_api_context api_context;
    provizio_radar_entities_api_context_init(NULL, NULL, &api_context);

    provizio_radar_entities_packet packet;
    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, 100, 200, radar_position_id,
                                                                    provizio_radar_range_short, num_entities,
                                                                    num_entities - 1, 0.0F));
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));

    TEST_ASSERT_EQUAL_INT32(0, provizio_test_create_entities_packet(&packet, 100, 200, radar_position_id,
                                                                    provizio_radar_range_short, num_entities,
                                                                    num_entities, provizio_test_entities_offset_step));

    provizio_set_on_error(&test_provizio_on_error);
    provizio_test_error[0] = '\0';
    TEST_ASSERT_EQUAL_INT32(
        PROVIZIO_E_PROTOCOL,
        provizio_handle_entities_packet(&api_context, &packet, provizio_radar_entities_packet_size(&packet.header)));
    const char *expected_message = "provizio_check_for_too_many_entities: Too many entities received"
#ifndef PROVIZIO__AVOID_PACKETS_DUPLICATION
                                   ", consider enabling AVOID_PACKETS_DUPLICATION option"
#endif
        ;
    TEST_ASSERT_EQUAL_STRING(expected_message, provizio_test_error);
    provizio_set_on_error(NULL);
}

int provizio_run_test_radar_entities(void)
{
    memset(provizio_test_warning, 0, sizeof(provizio_test_warning));
    memset(provizio_test_error, 0, sizeof(provizio_test_error));

    UNITY_BEGIN();

    RUN_TEST(test_provizio_radar_entities_packet_size);
    RUN_TEST(test_provizio_check_radar_entities_packet);
    RUN_TEST(test_provizio_handle_entities_packet_warnings);
    RUN_TEST(test_provizio_radar_entities_api_context_assign);
    RUN_TEST(test_provizio_handle_radars_entities_packet_bad_packet);
    RUN_TEST(test_provizio_handle_possible_radar_entities_packet_wrong_packet_size);
    RUN_TEST(test_provizio_handle_possible_radar_entities_packet_wrong_packet_type);
    RUN_TEST(test_provizio_handle_possible_radar_entities_packet_wrong_protocol_version);
    RUN_TEST(test_provizio_handle_possible_radar_entities_packet_ok);
    RUN_TEST(test_provizio_handle_possible_radars_entities_packet_ok);
    RUN_TEST(test_provizio_handle_entities_packet_frame_indices_overflow);
    RUN_TEST(test_provizio_handle_entities_packet_drop_obsolete_incomplete_frame);
    RUN_TEST(test_provizio_handle_entities_packet_too_many_entities);
    RUN_TEST(test_provizio_return_entities_frame_returns_older_first);
    RUN_TEST(test_provizio_handle_entities_packet_empty_frame);
    RUN_TEST(test_provizio_handle_radars_entities_packet_reuses_existing_context);
    RUN_TEST(test_provizio_handle_radars_entities_packet_out_of_contexts);
    RUN_TEST(test_provizio_test_next_value_handles_nonpositive_range);

    return UNITY_END();
}
