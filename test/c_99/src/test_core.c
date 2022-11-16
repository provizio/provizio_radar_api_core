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

#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "provizio/radar_api/core.h"
#include "provizio/util.h"

#include "test_point_cloud_callbacks.h"

typedef struct test_receive_packet_on_packet_sent_callback_data // NOLINT: it's aligned exactly as it's supposed to
{
    provizio_radar_point_cloud_api_context *contexts;
    size_t num_contexts;
    provizio_radar_api_connection *connection;
} test_receive_packet_on_packet_sent_callback_data;

typedef struct test_stop_when_ordered_callback_data // NOLINT: it's aligned exactly as it's supposed to
{
    pthread_mutex_t *mutex;
    int32_t *stop_flag;
} test_stop_when_ordered_callback_data;

typedef struct test_stop_when_ordered_thread_data // NOLINT: it's aligned exactly as it's supposed to
{
    uint16_t port;
    uint32_t first_frame_index;
    uint64_t initial_timestamp;
    uint16_t *radar_position_ids;
    uint16_t *radar_modes;
    size_t num_radars;
    uint16_t num_points;
    test_stop_when_ordered_callback_data stop_condition;
} test_stop_when_ordered_thread_data;

typedef int32_t (*provizio_radar_point_cloud_packet_callback)(const provizio_radar_point_cloud_packet *packet,
                                                              void *user_data);

typedef struct send_point_cloud_packet_data // NOLINT: it's aligned exactly as it's supposed to
{
    PROVIZIO__SOCKET sock;
    struct sockaddr *target_address;

    provizio_radar_point_cloud_packet_callback further_callback;
    void *user_data;
} send_point_cloud_packet_data;

enum
{
    test_message_length = 1024
};
static char provizio_test_error[test_message_length]; // NOLINT: non-const global by design

static void test_provizio_on_error(const char *error)
{
    strncpy(provizio_test_error, error, test_message_length - 1);
}

#ifdef WIN32
/**
 * @brief Tests-purpose implementation of *nix nanosleep for Windows (though only supports milliseconds precision)
 *
 * @param req How long to sleep
 * @param rem Ignored
 */
static void nanosleep(const struct timespec *req, struct timespec *rem)
{
    // We don't use rem in these tests anyway
    (void)rem;

    const time_t sec_conversion = 1000;
    const time_t nsec_conversion = 1000000;
    Sleep((uint32_t)(req->tv_sec * sec_conversion + req->tv_nsec / nsec_conversion));
}
#endif

static int32_t test_stop_when_ordered(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    (void)packet;

    test_stop_when_ordered_callback_data *data = (test_stop_when_ordered_callback_data *)user_data;

    pthread_mutex_lock(data->mutex);
    const int32_t stop = *data->stop_flag;
    pthread_mutex_unlock(data->mutex);

    return stop;
}

static int32_t make_test_pointcloud(const uint32_t frame_index, const uint64_t timestamp,
                                    const uint16_t *radar_position_ids, const uint16_t *radar_modes,
                                    const size_t num_radars, const uint16_t num_points,
                                    const uint16_t drop_after_num_points,
                                    provizio_radar_point_cloud_packet_callback callback, void *user_data)
{
    if (drop_after_num_points > num_points)
    {
        return PROVIZIO_E_ARGUMENT; // LCOV_EXCL_LINE: just a safety net, no need to achieve full coverage of test code
    }

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

    float frame_x = (x_meters_min + x_meters_max) * half;
    float frame_y = (y_meters_min + y_meters_max) * half;
    float frame_z = (z_meters_min + z_meters_max) * half;
    float frame_velocity = (velocity_min + velocity_max) * half;
    float frame_signal_to_noise_ratio = (signal_to_noise_ratio_min + signal_to_noise_ratio_max) * half;

    for (uint16_t num_points_left = drop_after_num_points; num_points_left > 0;) // NOLINT: The loop is just fine
    {
        const uint16_t points_in_packet = num_points_left < PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET
                                              ? num_points_left
                                              : PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET;

        float x_meters = 0;
        float y_meters = 0;
        float z_meters = 0;
        float velocity = 0;
        float signal_to_noise_ratio = 0;

        for (size_t i = 0; i < num_radars; ++i)
        {
            x_meters = frame_x;
            y_meters = frame_y;
            z_meters = frame_z;
            velocity = frame_velocity;
            signal_to_noise_ratio = frame_signal_to_noise_ratio;

            provizio_radar_point_cloud_packet packet;
            memset(&packet, 0, sizeof(packet));
            provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                                 PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
            provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                                 PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
            provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);
            provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
            provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_ids[i]);
            provizio_set_protocol_field_uint16_t(&packet.header.radar_mode,
                                                 radar_modes != NULL ? radar_modes[i] : provizio_radar_mode_unknown);
            provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
            provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, points_in_packet);

#pragma unroll(8)
            for (uint16_t j = 0; j < points_in_packet; ++j) // NOLINT: The loop is just fine
            {
#define PROVIZIO__NEXT_TEST_VALUE(v) ((v##_min) + fmodf((v) + (v##_step) - (v##_min), (v##_max) - (v##_min)))
                x_meters = PROVIZIO__NEXT_TEST_VALUE(x_meters);
                y_meters = PROVIZIO__NEXT_TEST_VALUE(y_meters);
                z_meters = PROVIZIO__NEXT_TEST_VALUE(z_meters);
                velocity = PROVIZIO__NEXT_TEST_VALUE(velocity);
                signal_to_noise_ratio = PROVIZIO__NEXT_TEST_VALUE(signal_to_noise_ratio);
#undef PROVIZIO__NEXT_TEST_VALUE

                provizio_set_protocol_field_float(&packet.radar_points[j].x_meters, x_meters);
                provizio_set_protocol_field_float(&packet.radar_points[j].y_meters, y_meters);
                provizio_set_protocol_field_float(&packet.radar_points[j].z_meters, z_meters);
                provizio_set_protocol_field_float(&packet.radar_points[j].radar_relative_radial_velocity_m_s, velocity);
                provizio_set_protocol_field_float(&packet.radar_points[j].signal_to_noise_ratio, signal_to_noise_ratio);
                provizio_set_protocol_field_float(&packet.radar_points[j].ground_relative_radial_velocity_m_s,
                                                  nanf(""));
            }

            const int32_t result = callback(&packet, user_data);
            if (result != 0)
            {
                return result;
            }
        }

        frame_x = x_meters;
        frame_y = y_meters;
        frame_z = z_meters;
        frame_velocity = velocity;
        frame_signal_to_noise_ratio = signal_to_noise_ratio;

        num_points_left -= points_in_packet;
    }

    // All sent
    return 0;
}

static int32_t send_point_cloud_packet(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    send_point_cloud_packet_data *data = (send_point_cloud_packet_data *)user_data;

    // Conversion to int32_t may be required as sendto may return different types in different platforms
    const int32_t sent = (int32_t)sendto(data->sock, (const char *)packet, // NOLINT
                                         (uint16_t)provizio_radar_point_cloud_packet_size(&packet->header), 0,
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
                                     const uint16_t *radar_position_ids, const uint16_t *radar_modes,
                                     const size_t num_radars, const uint16_t num_points,
                                     const uint16_t drop_after_num_points,
                                     provizio_radar_point_cloud_packet_callback on_packet_sent, void *user_data)
{
    PROVIZIO__SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (!provizio_socket_valid(sock))
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        const int32_t status = (int32_t)errno;
        provizio_error("send_test_pointcloud: Failed to create a UDP socket!");
        return status != 0 ? status : (int32_t)-1;
        // LCOV_EXCL_STOP
    }

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    my_address.sin_port = 0;                 // Any port
    my_address.sin_addr.s_addr = INADDR_ANY; // Any address

    int32_t status = bind(sock, (struct sockaddr *)&my_address, sizeof(my_address));
    if (status != 0)
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
    // linting disabled in next line as htons implementation is up to a platform (it uses asm instructions in some
    // platforms, which clang-tidy hates)
    target_address.sin_port = htons(port); // NOLINT
    target_address.sin_addr.s_addr = inet_addr("127.0.0.1");

    send_point_cloud_packet_data send_data;
    memset(&send_data, 0, sizeof(send_data));
    send_data.sock = sock;
    send_data.target_address = (struct sockaddr *)&target_address;
    send_data.further_callback = on_packet_sent;
    send_data.user_data = user_data;

    status = make_test_pointcloud(frame_index, timestamp, radar_position_ids, radar_modes, num_radars, num_points,
                                  drop_after_num_points, &send_point_cloud_packet, &send_data);
    if (status != 0)
    {
        provizio_socket_close(sock);
        return status;
    }

    status = provizio_socket_close(sock);
    if (status != 0)
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
                                                    const uint16_t *radar_position_ids, const uint16_t *radar_modes,
                                                    const size_t num_radars, const uint16_t num_points,
                                                    provizio_radar_point_cloud_packet_callback on_packet_sent,
                                                    void *user_data)
{
    const uint64_t time_between_frames_ns = 100000000ULL;
    struct timespec time_between_frames_timespec;
    time_between_frames_timespec.tv_sec = 0;
    time_between_frames_timespec.tv_nsec = (int32_t)time_between_frames_ns;

    uint32_t frame_index = first_frame_index;
    uint64_t timestamp = initial_timestamp;
    int32_t status = 0;
    while ((status = send_test_point_cloud(port, frame_index, timestamp, // NOLINT: don't unroll the loop
                                           radar_position_ids, radar_modes, num_radars, num_points, num_points,
                                           on_packet_sent, user_data)) == 0)
    {
        ++frame_index;
        timestamp += time_between_frames_ns;
        nanosleep(&time_between_frames_timespec, NULL);
    }

    return status;
}

static void *test_stop_when_ordered_thread(void *thread_data)
{
    test_stop_when_ordered_thread_data *data = (test_stop_when_ordered_thread_data *)thread_data;
    send_test_point_clouds_until_stopped(data->port, data->first_frame_index, data->initial_timestamp,
                                         data->radar_position_ids, data->radar_modes, data->num_radars,
                                         data->num_points, &test_stop_when_ordered, &data->stop_condition);
    return NULL;
}

static int32_t test_receive_packet_on_packet_sent(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    (void)packet;

    test_receive_packet_on_packet_sent_callback_data *data =
        (test_receive_packet_on_packet_sent_callback_data *)user_data;

    return provizio_radar_api_receive_packet(data->connection);
}

static void test_receives_single_radar_point_cloud_from_single_radar(void)
{
    const uint16_t port_number = 10001 + PROVIZIO__RADAR_API_DEFAULT_PORT;
    const uint32_t frame_index = 17;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t radar_mode = provizio_radar_mode_long_range;
    const uint16_t num_points = 32768;

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);
    provizio_radar_api_connection connection;
    int32_t status = provizio_open_radar_connection(port_number, 0, 0, &api_context, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock)); // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_id, &radar_mode, 1, num_points,
                                   num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_close_radar_connection(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    TEST_ASSERT_EQUAL_INT32(1, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_index, callback_data->last_point_clouds[0].frame_index);
    TEST_ASSERT_EQUAL_UINT64(timestamp, callback_data->last_point_clouds[0].timestamp);
    TEST_ASSERT_EQUAL_UINT16(radar_position_id, callback_data->last_point_clouds[0].radar_position_id);
    TEST_ASSERT_EQUAL_UINT16(radar_mode, callback_data->last_point_clouds[0].radar_mode);
    TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[0].num_points_expected);
    TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[0].num_points_received);

    // First point
    TEST_ASSERT_EQUAL_FLOAT(12.33F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[0].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(1.17F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[0].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(11.97F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[0].z_meters);
    TEST_ASSERT_EQUAL_FLOAT(5.31F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[0].radar_relative_radial_velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(29.73F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[0].signal_to_noise_ratio);

    // Last point
    TEST_ASSERT_EQUAL_FLOAT(29.5F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points - 1].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(-1.4375F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points - 1].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(22.4543F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points - 1].z_meters);
    TEST_ASSERT_EQUAL_FLOAT( // NOLINT
        -1.95574F, callback_data->last_point_clouds[0].radar_points[num_points - 1].radar_relative_radial_velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(42.63091F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points - 1].signal_to_noise_ratio);

    // Next after last (should be all zero)
    TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points].x_meters);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points].y_meters);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points].z_meters);
    TEST_ASSERT_EQUAL_FLOAT( // NOLINT
        0.0F, callback_data->last_point_clouds[0].radar_points[num_points].radar_relative_radial_velocity_m_s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                            callback_data->last_point_clouds[0].radar_points[num_points].signal_to_noise_ratio);

    free(callback_data);
}

static void test_receives_single_radar_point_cloud_from_2_radars(void)
{
    const uint16_t port_number = 10002 + PROVIZIO__RADAR_API_DEFAULT_PORT;
    const uint32_t frame_index = 17;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_ids[2] = {provizio_radar_position_rear_left, provizio_radar_position_front_center};
    const uint16_t radar_modes[2] = {provizio_radar_mode_short_range, provizio_radar_mode_ultra_long_range};
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
    provizio_radar_api_connection connection;
    int32_t status = provizio_open_radars_connection(port_number, 0, 0, api_contexts, num_contexts, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock)); // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE

    send_test_callback_data.contexts = api_contexts;
    send_test_callback_data.num_contexts = num_contexts;
    send_test_callback_data.connection = &connection;

    status =
        send_test_point_cloud(port_number, frame_index, timestamp, radar_position_ids, radar_modes, num_radars,
                              num_points, num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_close_radars_connection(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // 2 point clouds received - one per radar
    TEST_ASSERT_EQUAL_INT32(num_radars, callback_data->called_times);

    for (size_t i = 0; i < num_radars; ++i) // NOLINT: The loop is just fine
    {
        const size_t radar_index = 1 - i; // As last_point_clouds is sorted from latest to oldest
        TEST_ASSERT_EQUAL_UINT32(frame_index, callback_data->last_point_clouds[i].frame_index);
        TEST_ASSERT_EQUAL_UINT64(timestamp, callback_data->last_point_clouds[i].timestamp);
        TEST_ASSERT_EQUAL_UINT16(radar_position_ids[radar_index],
                                 callback_data->last_point_clouds[i].radar_position_id);
        TEST_ASSERT_EQUAL_UINT16(radar_modes[radar_index], callback_data->last_point_clouds[i].radar_mode);
        TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[i].num_points_expected);
        TEST_ASSERT_EQUAL_UINT16(num_points, callback_data->last_point_clouds[i].num_points_received);

        // First point
        TEST_ASSERT_EQUAL_FLOAT(12.33F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[0].x_meters);
        TEST_ASSERT_EQUAL_FLOAT(1.17F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[0].y_meters);
        TEST_ASSERT_EQUAL_FLOAT(11.97F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[0].z_meters);
        TEST_ASSERT_EQUAL_FLOAT(5.31F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[0].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(29.73F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[0].signal_to_noise_ratio);

        // Last point
        TEST_ASSERT_EQUAL_FLOAT(29.5F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points - 1].x_meters);
        TEST_ASSERT_EQUAL_FLOAT(-1.4375F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points - 1].y_meters);
        TEST_ASSERT_EQUAL_FLOAT(22.4543F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points - 1].z_meters);
        TEST_ASSERT_EQUAL_FLOAT( // NOLINT
            -1.95574F,
            callback_data->last_point_clouds[i].radar_points[num_points - 1].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(42.63091F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points - 1].signal_to_noise_ratio);

        // Next after last (should be all zero)
        TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points].x_meters);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points].y_meters);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points].z_meters);
        TEST_ASSERT_EQUAL_FLOAT( // NOLINT
            0.0F, callback_data->last_point_clouds[i].radar_points[num_points].radar_relative_radial_velocity_m_s);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, // NOLINT
                                callback_data->last_point_clouds[i].radar_points[num_points].signal_to_noise_ratio);
    }

    free(api_contexts);
    free(callback_data);
}

static void test_receive_radar_point_cloud_frame_indices_overflow(void)
{
    const uint16_t port_number = 10005 + PROVIZIO__RADAR_API_DEFAULT_PORT;
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
    provizio_radar_api_connection connection;
    int32_t status = provizio_open_radar_connection(port_number, 0, 0, &api_context, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock)); // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send and receive a frame
    status = send_test_point_cloud(port_number, frame_indices[0], timestamp, &radar_position_id, NULL, 1, num_points,
                                   num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send and partly receive an incomplete frame
    status = send_test_point_cloud(port_number, frame_indices[1], timestamp, &radar_position_id, NULL, 1, num_points,
                                   num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send a complete frame, and make sure it got received while the incomplete one got dropped
    status = send_test_point_cloud(port_number, frame_indices[2], timestamp, &radar_position_id, NULL, 1, num_points,
                                   num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_close_radar_connection(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // The second frame should be dropped due to the state reset on frame numbers overflow - it's by design
    TEST_ASSERT_EQUAL_INT32(2, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[0], callback_data->last_point_clouds[1].frame_index);
    TEST_ASSERT_EQUAL_UINT32(frame_indices[2], callback_data->last_point_clouds[0].frame_index);

    free(callback_data);
}

static void test_receive_radar_point_cloud_frame_position_ids_mismatch(void)
{
    const uint16_t port_number = 10006 + PROVIZIO__RADAR_API_DEFAULT_PORT;
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
    provizio_radar_api_connection connection;
    int32_t status = provizio_open_radar_connection(port_number, 0, 0, &api_context, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock)); // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send and partly receive a frame
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_ids[0], NULL, 1, num_points,
                                   num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send the last missing point of the frame, but make sure it's ingored due to the radar position mismatch
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_ids[1], NULL, 1, num_points, 1,
                                   &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_SKIPPED, status);

    // Send and correctly receive a frame now as the radar position is correct
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_ids[0], NULL, 1, num_points, 1,
                                   &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_close_radar_connection(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    TEST_ASSERT_EQUAL_INT32(1, callback_data->called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_index, callback_data->last_point_clouds[0].frame_index);

    free(callback_data);
}

static void test_receive_radar_point_cloud_drop_obsolete_incomplete_frame(void)
{
    const uint16_t port_number = 10007 + PROVIZIO__RADAR_API_DEFAULT_PORT;
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
    provizio_radar_api_connection connection;
    int32_t status = provizio_open_radar_connection(port_number, 0, 0, &api_context, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock)); // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE

    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send 3 incomplete frames
    for (size_t i = 0; i < sizeof(frame_indices) / sizeof(frame_indices[0]); ++i) // NOLINT: Don't unroll the loop
    {
        status =
            send_test_point_cloud(port_number, frame_indices[i], timestamp, &radar_position_id, NULL, 1, num_points,
                                  num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
        TEST_ASSERT_EQUAL_INT32(0, status);
    }

    status = provizio_close_radar_connection(&connection);
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
    const uint16_t port_number = 10008 + PROVIZIO__RADAR_API_DEFAULT_PORT;
    const uint32_t frame_index = 120;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t num_points = 200;
    const uint16_t num_extra_points = 2;

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(NULL, NULL, &api_context);
    provizio_radar_api_connection connection;
    int32_t status = provizio_open_radar_connection(port_number, 0, 0, &api_context, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock)); // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE

    test_receive_packet_on_packet_sent_callback_data send_test_callback_data;
    memset(&send_test_callback_data, 0, sizeof(send_test_callback_data));
    send_test_callback_data.contexts = &api_context;
    send_test_callback_data.num_contexts = 1;
    send_test_callback_data.connection = &connection;

    // Send all but 1 last point
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_id, NULL, 1, num_points,
                                   num_points - 1, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(0, status);

    // Send 2 more points, i.e. 1 too many
    provizio_set_on_error(&test_provizio_on_error);
    status = send_test_point_cloud(port_number, frame_index, timestamp, &radar_position_id, NULL, 1, num_points,
                                   num_extra_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_PROTOCOL, status);
    TEST_ASSERT_EQUAL_STRING("provizio_handle_radar_point_cloud_packet_checked: Too many points received",
                             provizio_test_error);
    provizio_set_on_error(NULL);

    status = provizio_close_radar_connection(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
}

static void test_receive_radar_point_cloud_not_enough_contexts(void)
{
    const uint16_t port_number = 10009 + PROVIZIO__RADAR_API_DEFAULT_PORT;
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
    provizio_radar_api_connection connection;
    int32_t status = provizio_open_radars_connection(port_number, 0, 0, api_contexts, num_contexts, &connection);
    TEST_ASSERT_EQUAL_INT32(0, status);
    TEST_ASSERT_TRUE(provizio_socket_valid(connection.sock)); // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE

    send_test_callback_data.contexts = api_contexts;
    send_test_callback_data.num_contexts = num_contexts;
    send_test_callback_data.connection = &connection;

    provizio_set_on_error(&test_provizio_on_error);
    status =
        send_test_point_cloud(port_number, frame_index, timestamp, radar_position_ids, NULL, num_radars, num_points,
                              num_points, &test_receive_packet_on_packet_sent, &send_test_callback_data);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_OUT_OF_CONTEXTS, status);
    TEST_ASSERT_EQUAL_STRING("provizio_get_radar_point_cloud_api_context_by_position_id: Out of available contexts",
                             provizio_test_error);
    provizio_set_on_error(NULL);

    status = provizio_close_radars_connection(&connection);
    TEST_ASSERT_EQUAL_INT32(0, status);

    free(api_contexts);
    free(callback_data);
}

static void test_receive_radar_point_cloud_timeout_ok(void)
{
    const uint16_t port_number = 10003 + PROVIZIO__RADAR_API_DEFAULT_PORT;
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
    thread_data.radar_modes = NULL;
    thread_data.num_radars = 1;
    thread_data.num_points = num_points;
    thread_data.stop_condition.mutex = &mutex;
    thread_data.stop_condition.stop_flag = &stop_flag;
    pthread_t thread; // NOLINT: Its value is set in the very next line
    TEST_ASSERT_EQUAL(0, pthread_create(&thread, NULL, &test_stop_when_ordered_thread, &thread_data));

    test_provizio_radar_point_cloud_callback_data *callback_data =
        (test_provizio_radar_point_cloud_callback_data *)malloc(sizeof(test_provizio_radar_point_cloud_callback_data));
    memset(callback_data, 0, sizeof(test_provizio_radar_point_cloud_callback_data));
    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(&test_provizio_radar_point_cloud_callback, callback_data, &api_context);

    provizio_radar_api_connection connection;
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_open_radar_connection(port_number, receive_timeout_ns, 1, &api_context, &connection));

    while (callback_data->called_times == 0) // NOLINT: No need to unroll
    {
        TEST_ASSERT_EQUAL_INT32(0, provizio_radar_api_receive_packet(&connection));
    }

    TEST_ASSERT_EQUAL(0, provizio_close_radar_connection(&connection));

    TEST_ASSERT_EQUAL(0, pthread_mutex_lock(&mutex));
    stop_flag = ECANCELED;
    TEST_ASSERT_EQUAL(0, pthread_mutex_unlock(&mutex));

    TEST_ASSERT_EQUAL(0, pthread_join(thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));

    TEST_ASSERT_GREATER_OR_EQUAL(1, callback_data->called_times);
    TEST_ASSERT_LESS_OR_EQUAL(2, callback_data->called_times); // Can be called twice as it could start receiving after
                                                               // some packets of its first frame had been sent
    TEST_ASSERT_EQUAL(num_points, callback_data->last_point_clouds[0].num_points_received);
    TEST_ASSERT_TRUE(callback_data->called_times == 1 || // NOLINT: clang-tidy doesn't like TEST_ASSERT_TRUE
                     callback_data->last_point_clouds[1].num_points_received < num_points);

    free(callback_data);
}

static void test_receive_radar_point_cloud_timeout_fails(void)
{
    const uint16_t port_number = 10004 + PROVIZIO__RADAR_API_DEFAULT_PORT;
    const uint64_t receive_timeout_ns = 100000000ULL; // 0.1s
    const int32_t thousand = 1000;

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_init(NULL, NULL, &api_context);

    struct timeval time_was;
    struct timeval time_now;

    // Make sure provizio_open_radar_connection fails due to timeout when checking connection is enabled
    TEST_ASSERT_EQUAL_INT32(0, provizio_gettimeofday(&time_was));
    provizio_radar_api_connection connection;
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_TIMEOUT, provizio_open_radar_connection(port_number, receive_timeout_ns, 1,
                                                                               &api_context, &connection));
    TEST_ASSERT_EQUAL_INT32(0, provizio_gettimeofday(&time_now));
    int32_t took_time_ms =
        (int32_t)((time_now.tv_sec - time_was.tv_sec) * thousand + (time_now.tv_usec - time_was.tv_usec) / thousand);
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(90, took_time_ms); // Allow missing 10ms due to system timer's inaccuracy
    TEST_ASSERT_LESS_THAN_INT32(200, took_time_ms);

    // It doesn't fail when checking connection is disabled
    TEST_ASSERT_EQUAL_INT32(
        0, provizio_open_radar_connection(port_number, receive_timeout_ns, 0, &api_context, &connection));

    // But fails to receive on time, when requested
    TEST_ASSERT_EQUAL_INT32(0, provizio_gettimeofday(&time_was));
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_TIMEOUT, provizio_radar_api_receive_packet(&connection));
    TEST_ASSERT_EQUAL_INT32(0, provizio_gettimeofday(&time_now));
    took_time_ms =
        (int32_t)((time_now.tv_sec - time_was.tv_sec) * thousand + (time_now.tv_usec - time_was.tv_usec) / thousand);
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(100, took_time_ms); // Allow missing 10ms due to system timer's inaccuracy
    TEST_ASSERT_LESS_THAN_INT32(200, took_time_ms);

    TEST_ASSERT_EQUAL_INT32(0, provizio_close_radar_connection(&connection));
}

static void test_provizio_radar_point_cloud_api_contexts_receive_packet_fails_as_not_connected(void)
{
    provizio_radar_api_connection api_connetion;
    memset(&api_connetion, 0, sizeof(api_connetion));
    api_connetion.sock = PROVIZIO__INVALID_SOCKET;

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_ARGUMENT, provizio_radar_api_receive_packet(&api_connetion));
    TEST_ASSERT_EQUAL_STRING("provizio_radar_api_receive_packet: Not connected", provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_radar_point_cloud_api_close_fails_as_not_connected(void)
{
    provizio_radar_api_connection api_connetion;
    memset(&api_connetion, 0, sizeof(api_connetion));
    api_connetion.sock = PROVIZIO__INVALID_SOCKET;

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL_INT32(PROVIZIO_E_ARGUMENT, provizio_close_radar_connection(&api_connetion));
    TEST_ASSERT_EQUAL_STRING("provizio_close_radars_connection: Not connected", provizio_test_error);
    provizio_set_on_error(NULL);
}

typedef struct test_provizio_set_radar_mode_radar_thread_data
{
    pthread_mutex_t *mutex;
    int32_t ready_flag;

    uint16_t port_number;
    uint16_t packet_type;
    uint16_t protocol_version;
    uint16_t radar_position_id;
    uint16_t requested_radar_mode;
    int32_t error_code;

    provizio_set_radar_mode_packet requested_packet;
} test_provizio_set_radar_mode_radar_thread_data;

static void *test_provizio_set_radar_mode_radar_thread(void *thread_data_void)
{
    test_provizio_set_radar_mode_radar_thread_data *thread_data =
        (test_provizio_set_radar_mode_radar_thread_data *)thread_data_void;

    PROVIZIO__SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    my_address.sin_port = htons(thread_data->port_number); // NOLINT: clang-tidy hates htons
    my_address.sin_addr.s_addr = INADDR_ANY;               // Any address

    if (bind(sock, (struct sockaddr *)&my_address, sizeof(my_address)) == 0)
    {
        pthread_mutex_lock(thread_data->mutex);
        thread_data->ready_flag = 1;
        pthread_mutex_unlock(thread_data->mutex);

        struct sockaddr from;
        socklen_t from_len = (socklen_t)sizeof(from);
        if ((size_t)recvfrom(sock, (char *)&thread_data->requested_packet, sizeof(provizio_set_radar_mode_packet), 0,
                             &from, &from_len) == sizeof(provizio_set_radar_mode_packet))
        {
            provizio_set_radar_mode_acknowledgement_packet acknowledgement_packet;
            memset(&acknowledgement_packet, 0, sizeof(acknowledgement_packet));
            provizio_set_protocol_field_uint16_t(&acknowledgement_packet.protocol_header.packet_type,
                                                 thread_data->packet_type);
            provizio_set_protocol_field_uint16_t(&acknowledgement_packet.protocol_header.protocol_version,
                                                 thread_data->protocol_version);
            provizio_set_protocol_field_uint16_t(&acknowledgement_packet.radar_position_id,
                                                 thread_data->radar_position_id);
            provizio_set_protocol_field_uint16_t(&acknowledgement_packet.requested_radar_mode,
                                                 thread_data->requested_radar_mode);
            provizio_set_protocol_field_uint32_t((uint32_t *)&acknowledgement_packet.error_code,
                                                 (uint32_t)thread_data->error_code);

            (void)sendto(sock, (const char *)&acknowledgement_packet, sizeof(acknowledgement_packet), 0, &from,
                         from_len);
        }
    }

    provizio_socket_close(sock);

    // To avoid hanging forever in case binding a socket failed
    pthread_mutex_lock(thread_data->mutex);
    thread_data->ready_flag = 1;
    pthread_mutex_unlock(thread_data->mutex);

    return NULL;
}

static void wait_till_test_provizio_set_radar_mode_radar_thread_ready(
    test_provizio_set_radar_mode_radar_thread_data *thread_data)
{
    const uint64_t sleep_time_ns = 20000000ULL;
    struct timespec sleep_timespec;
    sleep_timespec.tv_sec = 0;
    sleep_timespec.tv_nsec = (int32_t)sleep_time_ns;

    int32_t ready = 0;
    while (!ready)
    {
        nanosleep(&sleep_timespec, NULL);

        pthread_mutex_lock(thread_data->mutex);
        ready = thread_data->ready_flag;
        pthread_mutex_unlock(thread_data->mutex);
    }
}

static void test_provizio_set_radar_mode_ok(void)
{
    const uint16_t port_number = 10011 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    // Start the test radar thread
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));
    test_provizio_set_radar_mode_radar_thread_data thread_data;
    thread_data.mutex = &mutex;
    thread_data.ready_flag = 0;
    thread_data.port_number = port_number;
    thread_data.packet_type = PROVIZIO__RADAR_API_SET_MODE_ACKNOWLEDGEMENT_PACKET_TYPE;
    thread_data.protocol_version = PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION;
    thread_data.radar_position_id = radar_position_id;
    thread_data.requested_radar_mode = mode;
    thread_data.error_code = 0;
    pthread_t radar_thread; // NOLINT: Initialized in the next line
    TEST_ASSERT_EQUAL_INT32(
        0, pthread_create(&radar_thread, NULL, &test_provizio_set_radar_mode_radar_thread, &thread_data));
    wait_till_test_provizio_set_radar_mode_radar_thread_ready(&thread_data);

    TEST_ASSERT_EQUAL(0, provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));

    TEST_ASSERT_EQUAL(0, pthread_join(radar_thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));

    TEST_ASSERT_EQUAL(PROVIZIO__RADAR_API_SET_MODE_PACKET_TYPE,
                      provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.protocol_header.packet_type));
    TEST_ASSERT_EQUAL(
        PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION,
        provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.protocol_header.protocol_version));
    TEST_ASSERT_EQUAL((uint16_t)radar_position_id,
                      provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.radar_position_id));
    TEST_ASSERT_EQUAL((uint16_t)mode, provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.radar_mode));
}

static void test_provizio_set_radar_mode_broadcasting_ok(void)
{
    const uint16_t port_number = 10012 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    // Start the test radar thread
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));
    test_provizio_set_radar_mode_radar_thread_data thread_data;
    thread_data.mutex = &mutex;
    thread_data.ready_flag = 0;
    thread_data.port_number = port_number;
    thread_data.packet_type = PROVIZIO__RADAR_API_SET_MODE_ACKNOWLEDGEMENT_PACKET_TYPE;
    thread_data.protocol_version = PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION;
    thread_data.radar_position_id = radar_position_id;
    thread_data.requested_radar_mode = mode;
    thread_data.error_code = 0;
    pthread_t radar_thread; // NOLINT: Initialized in the next line
    TEST_ASSERT_EQUAL_INT32(
        0, pthread_create(&radar_thread, NULL, &test_provizio_set_radar_mode_radar_thread, &thread_data));
    wait_till_test_provizio_set_radar_mode_radar_thread_ready(&thread_data);

    TEST_ASSERT_EQUAL(0, provizio_set_radar_mode(radar_position_id, mode, port_number, "255.255.255.255"));

    TEST_ASSERT_EQUAL(0, pthread_join(radar_thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));

    TEST_ASSERT_EQUAL(PROVIZIO__RADAR_API_SET_MODE_PACKET_TYPE,
                      provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.protocol_header.packet_type));
    TEST_ASSERT_EQUAL(
        PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION,
        provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.protocol_header.protocol_version));
    TEST_ASSERT_EQUAL((uint16_t)radar_position_id,
                      provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.radar_position_id));
    TEST_ASSERT_EQUAL((uint16_t)mode, provizio_get_protocol_field_uint16_t(&thread_data.requested_packet.radar_mode));
}

static void test_provizio_set_radar_mode_invalid_mode(void)
{
    const uint16_t port_number = 10013 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    // radar_position_id, mode
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_unknown;

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL(PROVIZIO_E_ARGUMENT, provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));
    TEST_ASSERT_EQUAL_STRING("provizio_set_radar_mode: provizio_radar_mode_unknown is not a valid mode option!",
                             provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_set_radar_mode_timeout(void)
{
    const uint16_t port_number = 10014 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL(PROVIZIO_E_TIMEOUT, provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));
    TEST_ASSERT_EQUAL_STRING("provizio_set_radar_mode: No acknowledgement received, likely due to a connection issue",
                             provizio_test_error);
    provizio_set_on_error(NULL);
}

static void test_provizio_set_radar_mode_invalid_ack_packet_type(void)
{
    const uint16_t port_number = 10015 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    // Start the test radar thread
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));
    test_provizio_set_radar_mode_radar_thread_data thread_data;
    thread_data.mutex = &mutex;
    thread_data.ready_flag = 0;
    thread_data.port_number = port_number;
    thread_data.packet_type = PROVIZIO__RADAR_API_SET_MODE_ACKNOWLEDGEMENT_PACKET_TYPE + 1;
    thread_data.protocol_version = PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION;
    thread_data.radar_position_id = radar_position_id;
    thread_data.requested_radar_mode = mode;
    thread_data.error_code = 0;
    pthread_t radar_thread; // NOLINT: Initialized in the next line
    TEST_ASSERT_EQUAL_INT32(
        0, pthread_create(&radar_thread, NULL, &test_provizio_set_radar_mode_radar_thread, &thread_data));
    wait_till_test_provizio_set_radar_mode_radar_thread_ready(&thread_data);

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL(PROVIZIO_E_PROTOCOL, provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));
    TEST_ASSERT_EQUAL_STRING("provizio_set_radar_mode: Invalid acknowledgement packet type received",
                             provizio_test_error);
    provizio_set_on_error(NULL);

    TEST_ASSERT_EQUAL(0, pthread_join(radar_thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));
}

static void test_provizio_set_radar_mode_invalid_ack_protocol_version(void)
{
    const uint16_t port_number = 10016 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    // Start the test radar thread
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));
    test_provizio_set_radar_mode_radar_thread_data thread_data;
    thread_data.mutex = &mutex;
    thread_data.ready_flag = 0;
    thread_data.port_number = port_number;
    thread_data.packet_type = PROVIZIO__RADAR_API_SET_MODE_ACKNOWLEDGEMENT_PACKET_TYPE;
    thread_data.protocol_version = PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION + 1;
    thread_data.radar_position_id = radar_position_id;
    thread_data.requested_radar_mode = mode;
    thread_data.error_code = 0;
    pthread_t radar_thread; // NOLINT: Initialized in the next line
    TEST_ASSERT_EQUAL_INT32(
        0, pthread_create(&radar_thread, NULL, &test_provizio_set_radar_mode_radar_thread, &thread_data));
    wait_till_test_provizio_set_radar_mode_radar_thread_ready(&thread_data);

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL(PROVIZIO_E_PROTOCOL, provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));
    TEST_ASSERT_EQUAL_STRING("provizio_set_radar_mode: Incompatible protocol version", provizio_test_error);
    provizio_set_on_error(NULL);

    TEST_ASSERT_EQUAL(0, pthread_join(radar_thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));
}

static void test_provizio_set_radar_mode_timeout_due_to_incorrect_position(void)
{
    const uint16_t port_number = 10017 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    // Start the test radar thread
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));
    test_provizio_set_radar_mode_radar_thread_data thread_data;
    thread_data.mutex = &mutex;
    thread_data.ready_flag = 0;
    thread_data.port_number = port_number;
    thread_data.packet_type = PROVIZIO__RADAR_API_SET_MODE_ACKNOWLEDGEMENT_PACKET_TYPE;
    thread_data.protocol_version = PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION;
    thread_data.radar_position_id = (uint16_t)(radar_position_id + 1);
    thread_data.requested_radar_mode = mode;
    thread_data.error_code = 0;
    pthread_t radar_thread; // NOLINT: Initialized in the next line
    TEST_ASSERT_EQUAL_INT32(
        0, pthread_create(&radar_thread, NULL, &test_provizio_set_radar_mode_radar_thread, &thread_data));
    wait_till_test_provizio_set_radar_mode_radar_thread_ready(&thread_data);

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL(PROVIZIO_E_TIMEOUT, provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));
    TEST_ASSERT_EQUAL_STRING("provizio_set_radar_mode: No acknowledgement received, likely due to a connection issue",
                             provizio_test_error);
    provizio_set_on_error(NULL);

    TEST_ASSERT_EQUAL(0, pthread_join(radar_thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));
}

static void test_provizio_set_radar_mode_timeout_due_to_incorrect_mode(void)
{
    const uint16_t port_number = 10018 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    // Start the test radar thread
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));
    test_provizio_set_radar_mode_radar_thread_data thread_data;
    thread_data.mutex = &mutex;
    thread_data.ready_flag = 0;
    thread_data.port_number = port_number;
    thread_data.packet_type = PROVIZIO__RADAR_API_SET_MODE_ACKNOWLEDGEMENT_PACKET_TYPE;
    thread_data.protocol_version = PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION;
    thread_data.radar_position_id = radar_position_id;
    thread_data.requested_radar_mode = (uint16_t)(mode + 1);
    thread_data.error_code = 0;
    pthread_t radar_thread; // NOLINT: Initialized in the next line
    TEST_ASSERT_EQUAL_INT32(
        0, pthread_create(&radar_thread, NULL, &test_provizio_set_radar_mode_radar_thread, &thread_data));
    wait_till_test_provizio_set_radar_mode_radar_thread_ready(&thread_data);

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL(PROVIZIO_E_TIMEOUT, provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));
    TEST_ASSERT_EQUAL_STRING("provizio_set_radar_mode: No acknowledgement received, likely due to a connection issue",
                             provizio_test_error);
    provizio_set_on_error(NULL);

    TEST_ASSERT_EQUAL(0, pthread_join(radar_thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));
}

static void test_provizio_set_radar_mode_unsupported_mode(void)
{
    const uint16_t port_number = 10019 + PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT;
    const provizio_radar_position radar_position_id = provizio_radar_position_rear_right;
    const provizio_radar_mode mode = provizio_radar_mode_long_range;

    // Start the test radar thread
    pthread_mutex_t mutex;
    TEST_ASSERT_EQUAL(0, pthread_mutex_init(&mutex, NULL));
    test_provizio_set_radar_mode_radar_thread_data thread_data;
    thread_data.mutex = &mutex;
    thread_data.ready_flag = 0;
    thread_data.port_number = port_number;
    thread_data.packet_type = PROVIZIO__RADAR_API_SET_MODE_ACKNOWLEDGEMENT_PACKET_TYPE;
    thread_data.protocol_version = PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION;
    thread_data.radar_position_id = radar_position_id;
    thread_data.requested_radar_mode = mode;
    thread_data.error_code = PROVIZIO_E_NOT_PERMITTED;
    pthread_t radar_thread; // NOLINT: Initialized in the next line
    TEST_ASSERT_EQUAL_INT32(
        0, pthread_create(&radar_thread, NULL, &test_provizio_set_radar_mode_radar_thread, &thread_data));
    wait_till_test_provizio_set_radar_mode_radar_thread_ready(&thread_data);

    provizio_set_on_error(&test_provizio_on_error);
    TEST_ASSERT_EQUAL(PROVIZIO_E_NOT_PERMITTED,
                      provizio_set_radar_mode(radar_position_id, mode, port_number, "127.0.0.1"));
    TEST_ASSERT_EQUAL_STRING("provizio_set_radar_mode: Failed to set the requested mode", provizio_test_error);
    provizio_set_on_error(NULL);

    TEST_ASSERT_EQUAL(0, pthread_join(radar_thread, NULL));
    TEST_ASSERT_EQUAL(0, pthread_mutex_destroy(&mutex));
}

int provizio_run_test_core(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_receives_single_radar_point_cloud_from_single_radar);
    RUN_TEST(test_receives_single_radar_point_cloud_from_2_radars);
    RUN_TEST(test_receive_radar_point_cloud_frame_indices_overflow);
    RUN_TEST(test_receive_radar_point_cloud_frame_position_ids_mismatch);
    RUN_TEST(test_receive_radar_point_cloud_drop_obsolete_incomplete_frame);
    RUN_TEST(test_receive_radar_point_cloud_too_many_points);
    RUN_TEST(test_receive_radar_point_cloud_not_enough_contexts);
    RUN_TEST(test_receive_radar_point_cloud_timeout_ok);
    RUN_TEST(test_receive_radar_point_cloud_timeout_fails);
    RUN_TEST(test_provizio_radar_point_cloud_api_contexts_receive_packet_fails_as_not_connected);
    RUN_TEST(test_provizio_radar_point_cloud_api_close_fails_as_not_connected);
    RUN_TEST(test_provizio_set_radar_mode_ok);
    RUN_TEST(test_provizio_set_radar_mode_broadcasting_ok);
    RUN_TEST(test_provizio_set_radar_mode_invalid_mode);
    RUN_TEST(test_provizio_set_radar_mode_timeout);
    RUN_TEST(test_provizio_set_radar_mode_invalid_ack_packet_type);
    RUN_TEST(test_provizio_set_radar_mode_invalid_ack_protocol_version);
    RUN_TEST(test_provizio_set_radar_mode_timeout_due_to_incorrect_position);
    RUN_TEST(test_provizio_set_radar_mode_timeout_due_to_incorrect_mode);
    RUN_TEST(test_provizio_set_radar_mode_unsupported_mode);

    return UNITY_END();
}
