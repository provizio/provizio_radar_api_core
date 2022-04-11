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

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "unity/unity.h"

#include "provizio/radar_api/radar_point_cloud.h"
#include "provizio/socket.h"
#include "provizio/util.h"

#define PROVIZIO__TEST_MAX_MESSAGE_LENGTH 1024
char provizio_test_radar_point_cloud_warning[PROVIZIO__TEST_MAX_MESSAGE_LENGTH];
char provizio_test_radar_point_cloud_error[PROVIZIO__TEST_MAX_MESSAGE_LENGTH];

static void test_provizio_radar_point_cloud_on_warning(const char *warning)
{
    strncpy(provizio_test_radar_point_cloud_warning, warning, PROVIZIO__TEST_MAX_MESSAGE_LENGTH);
}

static void test_provizio_radar_point_cloud_on_error(const char *error)
{
    strncpy(provizio_test_radar_point_cloud_error, error, PROVIZIO__TEST_MAX_MESSAGE_LENGTH);
}

typedef int32_t (*provizio_radar_point_cloud_packet_callback)(const provizio_radar_point_cloud_packet *packet,
                                                              void *user_data);

static int32_t make_test_pointcloud(const uint32_t frame_index, const uint64_t timestamp,
                                    const uint16_t *radar_position_ids, const size_t num_radars,
                                    const uint16_t num_points, const uint16_t drop_after_num_points,
                                    provizio_radar_point_cloud_packet_callback callback, void *user_data)
{
    if (drop_after_num_points > num_points)
    {
        return EINVAL; // LCOV_EXCL_LINE: just a safety net, no need to achieve full coverage of test code
    }

    const float x_min = -100.0F;
    const float x_max = 100.0F;
    const float x_step = 12.33F;
    const float y_min = -15.0F;
    const float y_max = 15.0F;
    const float y_step = 1.17F;
    const float z_min = -2.0F;
    const float z_max = 25.0F;
    const float z_step = 0.47F;
    const float velocity_min = -30.0F;
    const float velocity_max = 30.0F;
    const float velocity_step = 5.31F;
    const float signal_to_noise_ratio_min = 2.0F;
    const float signal_to_noise_ratio_max = 50.0F;
    const float signal_to_noise_ratio_step = 3.73F;

    float frame_x = (x_min + x_max) / 2.0F;
    float frame_y = (y_min + y_max) / 2.0F;
    float frame_z = (z_min + z_max) / 2.0F;
    float frame_velocity = (velocity_min + velocity_max) / 2.0F;
    float frame_signal_to_noise_ratio = (signal_to_noise_ratio_min + signal_to_noise_ratio_max) / 2.0F;

    uint16_t num_points_left = drop_after_num_points;
    while (num_points_left > 0)
    {
        const uint16_t points_in_packet = num_points_left < PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET
                                              ? num_points_left
                                              : PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET;

        float x;
        float y;
        float z;
        float velocity;
        float signal_to_noise_ratio;

        for (size_t i = 0; i < num_radars; ++i)
        {
            x = frame_x;
            y = frame_y;
            z = frame_z;
            velocity = frame_velocity;
            signal_to_noise_ratio = frame_signal_to_noise_ratio;

            provizio_radar_point_cloud_packet packet;
            provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                                 PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
            provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                                 PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
            provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);
            provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
            provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_ids[i]);
            provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
            provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, points_in_packet);

            for (uint16_t i = 0; i < points_in_packet; ++i)
            {
#define PROVIZIO__NEXT_TEST_VALUE(v) v##_min + fmodf(v + v##_step - v##_min, v##_max - v##_min)
                x = PROVIZIO__NEXT_TEST_VALUE(x);
                y = PROVIZIO__NEXT_TEST_VALUE(y);
                z = PROVIZIO__NEXT_TEST_VALUE(z);
                velocity = PROVIZIO__NEXT_TEST_VALUE(velocity);
                signal_to_noise_ratio = PROVIZIO__NEXT_TEST_VALUE(signal_to_noise_ratio);
#undef PROVIZIO__NEXT_TEST_VALUE

                provizio_set_protocol_field_float(&packet.radar_points[i].x_meters, x);
                provizio_set_protocol_field_float(&packet.radar_points[i].y_meters, y);
                provizio_set_protocol_field_float(&packet.radar_points[i].z_meters, z);
                provizio_set_protocol_field_float(&packet.radar_points[i].velocity_m_s, velocity);
                provizio_set_protocol_field_float(&packet.radar_points[i].signal_to_noise_ratio, signal_to_noise_ratio);
            }

            const int32_t result = callback(&packet, user_data);
            if (result != 0)
            {
                return result;
            }
        }

        frame_x = x;
        frame_y = y;
        frame_z = z;
        frame_velocity = velocity;
        frame_signal_to_noise_ratio = signal_to_noise_ratio;

        num_points_left -= points_in_packet;
    }

    // All sent
    return 0;
}

typedef struct send_point_cloud_packet_data
{
    PROVIZIO__SOCKET sock;
    struct sockaddr *target_address;

    provizio_radar_point_cloud_packet_callback further_callback;
    void *user_data;
} send_point_cloud_packet_data;

static int32_t send_point_cloud_packet(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    send_point_cloud_packet_data *data = (send_point_cloud_packet_data *)user_data;

    const int32_t sent = sendto(data->sock, packet, provizio_radar_point_cloud_packet_size(&packet->header), 0,
                                data->target_address, sizeof(struct sockaddr_in));

    if (sent < 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("send_test_pointcloud: Failed to send provizio_radar_point_cloud_packet");
        return sent;
        // LCOV_EXCL_STOP
    }

    if (data->further_callback)
    {
        return data->further_callback(packet, data->user_data);
    }

    return 0; // LCOV_EXCL_LINE: just a safety net, no need to achieve full coverage of test code
}

static int32_t send_test_point_cloud(const uint16_t port, const uint32_t frame_index, const uint64_t timestamp,
                                     const uint16_t *radar_position_ids, const size_t num_radars,
                                     const uint16_t num_points, const uint16_t drop_after_num_points,
                                     provizio_radar_point_cloud_packet_callback on_packet_sent, void *user_data)
{
    PROVIZIO__SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (!provizio_socket_valid(sock))
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("send_test_pointcloud: Failed to create a UDP socket!");
        return 1;
        // LCOV_EXCL_STOP
    }

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    my_address.sin_port = 0;                 // Any port
    my_address.sin_addr.s_addr = INADDR_ANY; // Any address

    int32_t status;
    if ((status = bind(sock, (struct sockaddr *)&my_address, sizeof(my_address))) != 0)
    {
        // LCOV_EXCL_START: No need to achieve 100% coverage in test code
        provizio_error("send_test_pointcloud: Failed to bind socket");
        provizio_socket_close(sock);
        return status;
        // LCOV_EXCL_STOP
    }

    struct sockaddr_in target_address;
    memset(&target_address, 0, sizeof(target_address));
    target_address.sin_family = AF_INET;
    target_address.sin_port = htons(port);
    target_address.sin_addr.s_addr = inet_addr("127.0.0.1");

    send_point_cloud_packet_data send_data;
    send_data.sock = sock;
    send_data.target_address = (struct sockaddr *)&target_address;
    send_data.further_callback = on_packet_sent;
    send_data.user_data = user_data;

    if ((status = make_test_pointcloud(frame_index, timestamp, radar_position_ids, num_radars, num_points,
                                       drop_after_num_points, &send_point_cloud_packet, &send_data)) != 0)
    {
        provizio_socket_close(sock);
        return status;
    }

    if ((status = provizio_socket_close(sock)) != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("send_test_pointcloud: Failed to close the socket");
        return status;
        // LCOV_EXCL_STOP
    }

    return 0;
}

static int32_t send_test_point_clouds_until_stopped(const uint16_t port, const uint32_t first_frame_index,
                                                    const uint64_t initial_timestamp,
                                                    const uint16_t *radar_position_ids, const size_t num_radars,
                                                    const uint16_t num_points,
                                                    provizio_radar_point_cloud_packet_callback on_packet_sent,
                                                    void *user_data)
{
    const uint64_t time_between_frames = 100000000ULL;
    struct timespec time_between_frames_timespec;
    time_between_frames_timespec.tv_sec = 0;
    time_between_frames_timespec.tv_nsec = time_between_frames;

    uint32_t frame_index = first_frame_index;
    uint64_t timestamp = initial_timestamp;
    int32_t status = 0;
    while ((status = send_test_point_cloud(port, frame_index, timestamp, radar_position_ids, num_radars, num_points,
                                           num_points, on_packet_sent, user_data)) == 0)
    {
        ++frame_index;
        timestamp += time_between_frames;
        nanosleep(&time_between_frames_timespec, NULL);
    }

    return status;
}

#define PROVIZIO__TEST_CALLBACK_DATA_NUM_POINT_CLOUDS 4
typedef struct test_provizio_radar_point_cloud_callback_data
{
    int32_t called_times;
    provizio_radar_point_cloud last_point_clouds[PROVIZIO__TEST_CALLBACK_DATA_NUM_POINT_CLOUDS];
} test_provizio_radar_point_cloud_callback_data;

typedef struct test_receive_packet_on_packet_sent_callback_data
{
    provizio_radar_point_cloud_api_context *contexts;
    size_t num_contexts;
    provizio_radar_point_cloud_api_connection *connection;
} test_receive_packet_on_packet_sent_callback_data;

static void test_provizio_radar_point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
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

static int32_t test_receive_packet_on_packet_sent(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    (void)packet;

    test_receive_packet_on_packet_sent_callback_data *data =
        (test_receive_packet_on_packet_sent_callback_data *)user_data;

    return (data->num_contexts == 1
                ? provizio_radar_point_cloud_api_context_receive_packet(data->contexts, data->connection)
                : provizio_radar_point_cloud_api_contexts_receive_packet(data->contexts, data->num_contexts,
                                                                         data->connection));
}

typedef struct test_stop_when_ordered_callback_data
{
    pthread_mutex_t *mutex;
    int32_t *stop_flag;
} test_stop_when_ordered_callback_data;

static int32_t test_stop_when_ordered(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    (void)packet;

    test_stop_when_ordered_callback_data *data = (test_stop_when_ordered_callback_data *)user_data;

    pthread_mutex_lock(data->mutex);
    const int32_t stop = *data->stop_flag;
    pthread_mutex_unlock(data->mutex);

    return stop;
}

typedef struct test_stop_when_ordered_thread_data
{
    uint16_t port;
    uint32_t first_frame_index;
    uint64_t initial_timestamp;
    uint16_t *radar_position_ids;
    size_t num_radars;
    uint16_t num_points;
    test_stop_when_ordered_callback_data stop_condition;
} test_stop_when_ordered_thread_data;

static void *test_stop_when_ordered_thread(void *thread_data)
{
    test_stop_when_ordered_thread_data *data = (test_stop_when_ordered_thread_data *)thread_data;
    send_test_point_clouds_until_stopped(data->port, data->first_frame_index, data->initial_timestamp,
                                         data->radar_position_ids, data->num_radars, data->num_points,
                                         &test_stop_when_ordered, &data->stop_condition);
    return NULL;
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

    provizio_set_on_warning(&test_provizio_radar_point_cloud_on_warning);
    provizio_set_protocol_field_uint16_t(&header.num_points_in_packet, PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET + 1);
    TEST_ASSERT_EQUAL_UINT64(0, provizio_radar_point_cloud_packet_size(&header));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_point_cloud_packet_size: num_points_in_packet exceeds "
                             "PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET!",
                             provizio_test_radar_point_cloud_warning);
    provizio_set_on_warning(NULL);
}

static void test_provizio_check_radar_point_cloud_packet(void)
{
    const uint16_t num_points = 10;

    provizio_set_on_error(&test_provizio_radar_point_cloud_on_error);

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);

    provizio_radar_point_cloud_packet packet;
    memset(&packet, 0, sizeof(packet));

    // Check failure due to size < sizeof(provizio_radar_point_cloud_packet_protocol_header)
    TEST_ASSERT_EQUAL_INT32(EPROTO,
                            provizio_handle_radar_point_cloud_packet(
                                &api_context, &packet, sizeof(provizio_radar_point_cloud_packet_protocol_header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: insufficient packet_size",
                             provizio_test_radar_point_cloud_error);

    // Check incorrect packet type
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE + 1);
    TEST_ASSERT_EQUAL_INT32(EPROTO, provizio_handle_radar_point_cloud_packet(&api_context, &packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: unexpected packet_type",
                             provizio_test_radar_point_cloud_error);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);

    // Check incompatible protocol version
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION + 1);
    TEST_ASSERT_EQUAL_INT32(EPROTO, provizio_handle_radar_point_cloud_packet(&api_context, &packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: Incompatible protocol version",
                             provizio_test_radar_point_cloud_error);
    provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);

    // Check failure due to size < sizeof(provizio_radar_point_cloud_packet_header)
    TEST_ASSERT_EQUAL_INT32(EPROTO, provizio_handle_radar_point_cloud_packet(
                                        &api_context, &packet, sizeof(provizio_radar_point_cloud_packet_header) - 1));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: insufficient packet_size",
                             provizio_test_radar_point_cloud_error);

    // Check size mismatch as specified and as estimated by header (i.e. number of points in packet)
    provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, num_points);
    TEST_ASSERT_EQUAL_INT32(
        EPROTO, provizio_handle_radar_point_cloud_packet(
                    &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header) - 1)); // 1 byte less
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: incorrect packet_size",
                             provizio_test_radar_point_cloud_error);
    TEST_ASSERT_EQUAL_INT32(
        EPROTO, provizio_handle_radar_point_cloud_packet(
                    &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header) + 1)); // 1 byte more
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: incorrect packet_size",
                             provizio_test_radar_point_cloud_error);

    // Check fails on radar_position_id = provizio_radar_position_unknown
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, provizio_radar_position_unknown);
    TEST_ASSERT_EQUAL_INT32(EPROTO, provizio_handle_radar_point_cloud_packet(
                                        &api_context, &packet, provizio_radar_point_cloud_packet_size(&packet.header)));
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: the value of radar_position_id can't be "
                             "provizio_radar_position_unknown",
                             provizio_test_radar_point_cloud_error);

    // Empty frame
    provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, provizio_radar_position_front_center);
    provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, 0);
    TEST_ASSERT_EQUAL_INT32(EAGAIN, provizio_handle_radar_point_cloud_packet(
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

    provizio_set_on_error(&test_provizio_radar_point_cloud_on_error);
    TEST_ASSERT_EQUAL_INT32(EPROTO,
                            provizio_handle_radars_point_cloud_packet(
                                NULL, 0, &bad_packet, sizeof(bad_packet.header.protocol_header) - 1)); // Size too small
    TEST_ASSERT_EQUAL_STRING("provizio_check_radar_point_cloud_packet: insufficient packet_size",
                             provizio_test_radar_point_cloud_error);
    provizio_set_on_error(NULL);
}

static void test_receives_single_radar_point_cloud_from_single_radar(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_index = 17;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t num_points = 32768;

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);
    provizio_radar_point_cloud_api_connection connection;
    int32_t status = provizio_radar_point_cloud_api_connect(port_number, 0, 0, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock));

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_id, 1, num_points, num_points,
                                   &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_radar_point_cloud_api_close(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    TEST_ASSERT_EQUAL_INT32(1, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_index, callback_data->last_point_clouds[0].frame_index);
    TEST_ASSERT_EQUAL_UINT64(timestamp, callback_data->last_point_clouds[0].timestamp);
    TEST_ASSERT_EQUAL_UINT16(radar_position_id, callback_data->last_point_clouds[0].radar_position_id);
    TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[0].num_points_expected);
    TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[0].num_points_received);

    // First point
    TEST_ASSERT_EQUAL_FLOAT(12.33F, callback_data->last_point_clouds[0].radar_points[0].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(1.17F, callback_data->last_point_clouds[0].radar_points[0].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(11.97F, callback_data->last_point_clouds[0].radar_points[0].z_meters);
    TEST_ASSERT_EQUAL_FLOAT(5.31F, callback_data->last_point_clouds[0].radar_points[0].velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(29.73F, callback_data->last_point_clouds[0].radar_points[0].signal_to_noise_ratio);

    // Last point
    TEST_ASSERT_EQUAL_FLOAT(29.5F, callback_data->last_point_clouds[0].radar_points[num_points - 1].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(-1.4375F, callback_data->last_point_clouds[0].radar_points[num_points - 1].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(22.4543F, callback_data->last_point_clouds[0].radar_points[num_points - 1].z_meters);
    TEST_ASSERT_EQUAL_FLOAT(-1.95574F, callback_data->last_point_clouds[0].radar_points[num_points - 1].velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(42.63091F,
                            callback_data->last_point_clouds[0].radar_points[num_points - 1].signal_to_noise_ratio);

    // Next after last (should be all zero)
    TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[0].radar_points[num_points].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[0].radar_points[num_points].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[0].radar_points[num_points].z_meters);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[0].radar_points[num_points].velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[0].radar_points[num_points].signal_to_noise_ratio);

    free(callback_data);
}

static void test_receives_single_radar_point_cloud_from_2_radars(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_index = 17;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_ids[2] = {provizio_radar_position_rear_left, provizio_radar_position_front_center};
    const uint16_t num_radars = sizeof(radar_position_ids) / sizeof(radar_position_ids[0]);
    const uint16_t num_contexts = num_radars;
    const uint16_t num_points = 32768;

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));

    provizio_radar_point_cloud_api_context *api_contexts =
        (provizio_radar_point_cloud_api_context *)malloc(sizeof(provizio_radar_point_cloud_api_context) * num_contexts);
    provizio_radar_point_cloud_api_contexts_init(&test_provizio_radar_point_cloud_callback, callback_data, api_contexts,
                                                 num_contexts);
    provizio_radar_point_cloud_api_connection connection;
    int32_t status = provizio_radar_point_cloud_api_connect(port_number, 0, 0, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock));

    send_test_callback_data.contexts = api_contexts;
    send_test_callback_data.num_contexts = num_contexts;
    send_test_callback_data.connection = &connection;

    status = send_test_point_cloud(port_number, frame_index, timestamp, radar_position_ids, num_radars, num_points,
                                   num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_radar_point_cloud_api_close(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // 2 point clouds received - one per radar
    TEST_ASSERT_EQUAL_INT32(num_radars, callback_data->called_times);

    for (size_t i = 0; i < num_radars; ++i)
    {
        const size_t radar_index = 1 - i; // As last_point_clouds is sorted from latest to oldest
        TEST_ASSERT_EQUAL_UINT32(frame_index, callback_data->last_point_clouds[i].frame_index);
        TEST_ASSERT_EQUAL_UINT64(timestamp, callback_data->last_point_clouds[i].timestamp);
        TEST_ASSERT_EQUAL_UINT16(radar_position_ids[radar_index],
                                 callback_data->last_point_clouds[i].radar_position_id);
        TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[i].num_points_expected);
        TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[i].num_points_received);

        // First point
        TEST_ASSERT_EQUAL_FLOAT(12.33F, callback_data->last_point_clouds[i].radar_points[0].x_meters);
        TEST_ASSERT_EQUAL_FLOAT(1.17F, callback_data->last_point_clouds[i].radar_points[0].y_meters);
        TEST_ASSERT_EQUAL_FLOAT(11.97F, callback_data->last_point_clouds[i].radar_points[0].z_meters);
        TEST_ASSERT_EQUAL_FLOAT(5.31F, callback_data->last_point_clouds[i].radar_points[0].velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(29.73F, callback_data->last_point_clouds[i].radar_points[0].signal_to_noise_ratio);

        // Last point
        TEST_ASSERT_EQUAL_FLOAT(29.5F, callback_data->last_point_clouds[i].radar_points[num_points - 1].x_meters);
        TEST_ASSERT_EQUAL_FLOAT(-1.4375F, callback_data->last_point_clouds[i].radar_points[num_points - 1].y_meters);
        TEST_ASSERT_EQUAL_FLOAT(22.4543F, callback_data->last_point_clouds[i].radar_points[num_points - 1].z_meters);
        TEST_ASSERT_EQUAL_FLOAT(-1.95574F,
                                callback_data->last_point_clouds[i].radar_points[num_points - 1].velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(42.63091F,
                                callback_data->last_point_clouds[i].radar_points[num_points - 1].signal_to_noise_ratio);

        // Next after last (should be all zero)
        TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[i].radar_points[num_points].x_meters);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[i].radar_points[num_points].y_meters);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[i].radar_points[num_points].z_meters);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, callback_data->last_point_clouds[i].radar_points[num_points].velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(0.0F,
                                callback_data->last_point_clouds[i].radar_points[num_points].signal_to_noise_ratio);
    }

    free(api_contexts);
    free(callback_data);
}

static void test_receive_radar_point_cloud_timeout_ok(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint16_t first_frame_index = 500;
    const uint64_t initial_timestamp = 0x0123456789abcdef;
    uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint64_t receive_timeout_ns = 5000000000ULL; // 0.5s
    const uint16_t num_points = 1000;

    int32_t stop_flag = 0;
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));

    test_stop_when_ordered_thread_data thread_data;
    thread_data.port = port_number;
    thread_data.first_frame_index = first_frame_index;
    thread_data.initial_timestamp = initial_timestamp;
    thread_data.radar_position_ids = &radar_position_id;
    thread_data.num_radars = 1;
    thread_data.num_points = num_points;
    thread_data.stop_condition.mutex = &mutex;
    thread_data.stop_condition.stop_flag = &stop_flag;
    pthread_t thread;
    TEST_ASSERT_EQUAL(0, pthread_create(&thread, NULL, &test_stop_when_ordered_thread, &thread_data));

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);

    provizio_radar_point_cloud_api_connection connection;
    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_point_cloud_api_connect(port_number, receive_timeout_ns, 1, &connection));

    while (callback_data->called_times == 0)
    {
        TEST_ASSERT_EQUAL_INT32(0, provizio_radar_point_cloud_api_context_receive_packet(&api_context, &connection));
    }

    TEST_ASSERT_EQUAL(0, provizio_radar_point_cloud_api_close(&connection));

    TEST_ASSERT_EQUAL(0, pthread_mutex_lock(&mutex));
    stop_flag = ECANCELED;
    TEST_ASSERT_EQUAL(0, pthread_mutex_unlock(&mutex));

    TEST_ASSERT_EQUAL(0, pthread_join(thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));

    TEST_ASSERT_GREATER_OR_EQUAL(1, callback_data->called_times);
    TEST_ASSERT_LESS_OR_EQUAL(2, callback_data->called_times); // Can be called twice as it could start receiving after
                                                               // some packets of its first frame had been sent
    TEST_ASSERT_EQUAL(num_points, callback_data->last_point_clouds[0].num_points_received);
    TEST_ASSERT_TRUE(callback_data->called_times == 1 ||
                     callback_data->last_point_clouds[1].num_points_received < num_points);

    free(callback_data);
}

static void test_receive_radar_point_cloud_timeout_fails(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint64_t receive_timeout_ns = 100000000ULL; // 0.1s

    provizio_radar_point_cloud_api_context context;
    provizio_radar_point_cloud_api_context_init(NULL, NULL, &context);

    struct timeval time_was;
    struct timeval time_now;

    // Make sure provizio_radar_point_cloud_api_connect fails due to timeout when checking connection is enabled
    TEST_ASSERT_EQUAL_INT32(0, (int32_t)gettimeofday(&time_was, NULL));
    provizio_radar_point_cloud_api_connection connection;
    TEST_ASSERT_EQUAL_INT32(EAGAIN,
                            provizio_radar_point_cloud_api_connect(port_number, receive_timeout_ns, 1, &connection));
    TEST_ASSERT_EQUAL_INT32(0, (int32_t)gettimeofday(&time_now, NULL));
    int32_t took_time_ms =
        (int32_t)((time_now.tv_sec - time_was.tv_sec) * 1000 + (time_now.tv_usec - time_was.tv_usec) / 1000);
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(100, took_time_ms);
    TEST_ASSERT_LESS_THAN_INT32(200, took_time_ms);

    // It doesn't fail when checking connection is disabled
    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_point_cloud_api_connect(port_number, receive_timeout_ns, 0, &connection));

    // But fails to receive on time, when requested
    TEST_ASSERT_EQUAL_INT32(0, (int32_t)gettimeofday(&time_was, NULL));
    TEST_ASSERT_EQUAL_INT32(EAGAIN, provizio_radar_point_cloud_api_context_receive_packet(&context, &connection));
    TEST_ASSERT_EQUAL_INT32(0, (int32_t)gettimeofday(&time_now, NULL));
    took_time_ms = (int32_t)((time_now.tv_sec - time_was.tv_sec) * 1000 + (time_now.tv_usec - time_was.tv_usec) / 1000);
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(100, took_time_ms);
    TEST_ASSERT_LESS_THAN_INT32(200, took_time_ms);

    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_point_cloud_api_close(&connection));
}

static void test_receive_radar_point_cloud_frame_indices_overflow(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_indices[3] = {0xfffffffe, 0xffffffff, 0};
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t num_points = 32768;

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);
    provizio_radar_point_cloud_api_connection connection;
    int32_t status = provizio_radar_point_cloud_api_connect(port_number, 0, 0, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock));

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send and receive a frame
    status = send_test_point_cloud(port_number, frame_indices[0], timestamp, &radar_position_id, 1, num_points,
                                   num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send and partly receive an incomplete frame
    status = send_test_point_cloud(port_number, frame_indices[1], timestamp, &radar_position_id, 1, num_points,
                                   num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send a complete frame, and make sure it got received while the incomplete one got dropped
    status = send_test_point_cloud(port_number, frame_indices[2], timestamp, &radar_position_id, 1, num_points,
                                   num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_radar_point_cloud_api_close(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // The second frame should be dropped due to the state reset on frame numbers overflow - it's by design
    TEST_ASSERT_EQUAL_INT32(2, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[0], callback_data->last_point_clouds[1].frame_index);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[2], callback_data->last_point_clouds[0].frame_index);

    free(callback_data);
}

static void test_receive_radar_point_cloud_frame_position_ids_mismatch(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_index = 100;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_ids[2] = {provizio_radar_position_rear_left, provizio_radar_position_rear_right};
    const uint16_t num_points = 100;

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);
    provizio_radar_point_cloud_api_connection connection;
    int32_t status = provizio_radar_point_cloud_api_connect(port_number, 0, 0, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock));

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send and partly receive a frame
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_ids[0], 1, num_points,
                                   num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send the last missing point of the frame, but make sure it's ingored due to the radar position mismatch
    provizio_set_on_warning(&test_provizio_radar_point_cloud_on_warning);
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_ids[1], 1, 1, 1,
                                   &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(EAGAIN, status);
    TEST_ASSERT_EQUAL_STRING("provizio_get_point_cloud_being_received: context received a packet from a wrong radar",
                             provizio_test_radar_point_cloud_warning);
    provizio_set_on_warning(NULL);

    // Send and correctly receive a frame now as the radar position is correct
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_ids[0], 1, 1, 1,
                                   &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_radar_point_cloud_api_close(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    TEST_ASSERT_EQUAL_INT32(1, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_index, callback_data->last_point_clouds[0].frame_index);

    free(callback_data);
}

static void test_receive_radar_point_cloud_drop_obsolete_incomplete_frame(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_indices[3] = {17, 18, 19};
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t num_points = 200;

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);
    provizio_radar_point_cloud_api_connection connection;
    int32_t status = provizio_radar_point_cloud_api_connect(port_number, 0, 0, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock));

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send 3 incomplete frames
    for (size_t i = 0; i < sizeof(frame_indices) / sizeof(frame_indices[0]); ++i)
    {
        status = send_test_point_cloud(port_number, frame_indices[i], timestamp, &radar_position_id, 1, num_points,
                                       num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
        TEST_ASSERT_EQUAL_INT32(0, status);
    }

    status = provizio_radar_point_cloud_api_close(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Make sure the very first frame got returned even though it was incomplete
    TEST_ASSERT_EQUAL_INT32(1, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[0], callback_data->last_point_clouds[0].frame_index);
    TEST_ASSERT_EQUAL_UINT32(num_points, callback_data->last_point_clouds[0].num_points_expected);
    TEST_ASSERT_EQUAL_UINT32(num_points - 1,
                             callback_data->last_point_clouds[0].num_points_received); // Returned incomplete

    free(callback_data);
}

static void test_receive_radar_point_cloud_too_many_points(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_index = 120;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t num_points = 200;
    const uint16_t num_extra_points = 2;

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(NULL, NULL, &api_context);
    provizio_radar_point_cloud_api_connection connection;
    int32_t status = provizio_radar_point_cloud_api_connect(port_number, 0, 0, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock));

    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));
    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send all but 1 last point
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_id, 1, num_points,
                                   num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send 2 more points, i.e. 1 too many
    provizio_set_on_error(&test_provizio_radar_point_cloud_on_error);
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_id, 1, num_points,
                                   num_extra_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(EPROTO, status);
    TEST_ASSERT_EQUAL_STRING("provizio_handle_radar_point_cloud_packet_checked: Too many points received",
                             provizio_test_radar_point_cloud_error);
    provizio_set_on_error(NULL);

    status = provizio_radar_point_cloud_api_close(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
}

static void test_receive_radar_point_cloud_not_enough_contexts(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_index = 17;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_ids[3] = {provizio_radar_position_rear_left, provizio_radar_position_front_center,
                                            provizio_radar_position_front_right};
    const uint16_t num_radars = sizeof(radar_position_ids) / sizeof(radar_position_ids[0]);
    const uint16_t num_contexts = num_radars - 1; // One context too few
    const uint16_t num_points = 32768;

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));

    provizio_radar_point_cloud_api_context *api_contexts =
        (provizio_radar_point_cloud_api_context *)malloc(sizeof(provizio_radar_point_cloud_api_context) * num_contexts);
    provizio_radar_point_cloud_api_contexts_init(&test_provizio_radar_point_cloud_callback, callback_data, api_contexts,
                                                 num_contexts);
    provizio_radar_point_cloud_api_connection connection;
    int32_t status = provizio_radar_point_cloud_api_connect(port_number, 0, 0, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock));

    send_test_callback_data.contexts = api_contexts;
    send_test_callback_data.num_contexts = num_contexts;
    send_test_callback_data.connection = &connection;

    provizio_set_on_error(&test_provizio_radar_point_cloud_on_error);
    status = send_test_point_cloud(port_number, frame_index, timestamp, radar_position_ids, num_radars, num_points,
                                   num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(EBUSY, status);
    TEST_ASSERT_EQUAL_STRING(
        "provizio_get_provizio_radar_point_cloud_api_context_by_position_id: Out of available contexts",
        provizio_test_radar_point_cloud_error);
    provizio_set_on_error(NULL);

    status = provizio_radar_point_cloud_api_close(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    free(api_contexts);
    free(callback_data);
}

static void test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_size(void)
{
    provizio_radar_point_cloud_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    TEST_ASSERT_EQUAL_INT32(EAGAIN, provizio_handle_possible_radar_point_cloud_packet(
                                        NULL, &bad_packet, sizeof(provizio_radar_point_cloud_packet_header) - 1));
}

static void test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_type(void)
{
    provizio_radar_point_cloud_packet bad_packet;
    memset(&bad_packet, 0, sizeof(bad_packet));

    provizio_set_protocol_field_uint16_t(&bad_packet.header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE + 1);

    TEST_ASSERT_EQUAL_INT32(EAGAIN,
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

    TEST_ASSERT_EQUAL_INT32(EPROTO,
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

    for (size_t i = 0; i < num_contexts; ++i)
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

static void test_provizio_radar_point_cloud_api_connect_fails_due_to_port_taken(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;

    provizio_radar_point_cloud_api_connection ok_connection;
    provizio_radar_point_cloud_api_connection failed_connection;

    TEST_ASSERT_EQUAL_INT32(0, provizio_radar_point_cloud_api_connect(port_number, 0, 0, &ok_connection));

    provizio_set_on_error(&test_provizio_radar_point_cloud_on_error);
    TEST_ASSERT_NOT_EQUAL_INT32(0, provizio_radar_point_cloud_api_connect(port_number, 0, 0, &failed_connection));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_point_cloud_api_connect: Failed to bind a UDP socket!",
                             provizio_test_radar_point_cloud_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_radar_point_cloud_api_contexts_receive_packet_fails_as_not_connected(void)
{
    provizio_radar_point_cloud_api_connection api_connetion;
    memset(&api_connetion, 0, sizeof(api_connetion));
    api_connetion.sock = PROVIZIO__INVALID_SOCKET;

    provizio_set_on_error(&test_provizio_radar_point_cloud_on_error);
    TEST_ASSERT_NOT_EQUAL_INT32(0, provizio_radar_point_cloud_api_contexts_receive_packet(NULL, 0, &api_connetion));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_point_cloud_api_context_receive_packet: Failed to receive",
                             provizio_test_radar_point_cloud_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_radar_point_cloud_api_close_fails_as_not_connected(void)
{
    provizio_radar_point_cloud_api_connection api_connetion;
    memset(&api_connetion, 0, sizeof(api_connetion));
    api_connetion.sock = PROVIZIO__INVALID_SOCKET;

    provizio_set_on_error(&test_provizio_radar_point_cloud_on_error);
    TEST_ASSERT_NOT_EQUAL_INT32(0, provizio_radar_point_cloud_api_close(&api_connetion));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_point_cloud_api_close: provizio_socket_close failed!",
                             provizio_test_radar_point_cloud_error);
    provizio_set_on_error(NULL);
}

int32_t provizio_run_test_radar_point_cloud(void)
{
    memset(provizio_test_radar_point_cloud_warning, 0, sizeof(provizio_test_radar_point_cloud_warning));

    UNITY_BEGIN();

    RUN_TEST(test_provizio_radar_point_cloud_packet_size);
    RUN_TEST(test_provizio_check_radar_point_cloud_packet);
    RUN_TEST(test_provizio_handle_radars_point_cloud_packet_bad_packet);
    RUN_TEST(test_receives_single_radar_point_cloud_from_single_radar);
    RUN_TEST(test_receives_single_radar_point_cloud_from_2_radars);
    RUN_TEST(test_receive_radar_point_cloud_timeout_ok);
    RUN_TEST(test_receive_radar_point_cloud_timeout_fails);
    RUN_TEST(test_receive_radar_point_cloud_frame_indices_overflow);
    RUN_TEST(test_receive_radar_point_cloud_frame_position_ids_mismatch);
    RUN_TEST(test_receive_radar_point_cloud_drop_obsolete_incomplete_frame);
    RUN_TEST(test_receive_radar_point_cloud_too_many_points);
    RUN_TEST(test_receive_radar_point_cloud_not_enough_contexts);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_size);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_packet_type);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_wrong_protocol_version);
    RUN_TEST(test_provizio_handle_possible_radar_point_cloud_packet_ok);
    RUN_TEST(test_provizio_handle_possible_radars_point_cloud_packet_ok);
    RUN_TEST(test_provizio_radar_point_cloud_api_connect_fails_due_to_port_taken);
    RUN_TEST(test_provizio_radar_point_cloud_api_contexts_receive_packet_fails_as_not_connected);
    RUN_TEST(test_provizio_radar_point_cloud_api_close_fails_as_not_connected);

    return UNITY_END();
}
