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
#include <string.h>

#include "unity/unity.h"

#include "provizio/radar_api/radar_point_cloud.h"
#include "provizio/socket.h"
#include "provizio/util.h"

#define PROVIZIO__TEST_MAX_WARNING_LENGTH 1024
char provizio_test_radar_point_cloud_warning[PROVIZIO__TEST_MAX_WARNING_LENGTH];

void test_provizio_radar_point_cloud_on_warning(const char *warning)
{
    strcpy(provizio_test_radar_point_cloud_warning, warning);
}

typedef int (*provizio_radar_point_cloud_packet_callback)(const provizio_radar_point_cloud_packet *packet,
                                                          void *user_data);

int make_test_pointcloud(uint16_t frame_index, uint64_t timestamp, uint16_t radar_position_id, uint16_t num_points,
                         provizio_radar_point_cloud_packet_callback callback, void *user_data)
{
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

    float x = (x_min + x_max) / 2.0F;
    float y = (y_min + y_max) / 2.0F;
    float z = (z_min + z_max) / 2.0F;
    float velocity = (velocity_min + velocity_max) / 2.0F;
    float signal_to_noise_ratio = (signal_to_noise_ratio_min + signal_to_noise_ratio_max) / 2.0F;

    while (num_points > 0)
    {
        const uint16_t points_in_packet = num_points < PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET
                                              ? num_points
                                              : PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET;

        provizio_radar_point_cloud_packet packet;
        provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                             PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
        provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                             PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
        provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);
        provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
        provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_id);
        provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
        provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, points_in_packet);
        provizio_set_protocol_field_uint16_t(&packet.header.reserved, 0);

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

        const int result = callback(&packet, user_data);
        if (result != 0)
        {
            return result;
        }

        num_points -= points_in_packet;
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

int send_point_cloud_packet(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    send_point_cloud_packet_data *data = (send_point_cloud_packet_data *)user_data;

    const int sent = sendto(data->sock, packet, provizio_radar_point_cloud_packet_size(&packet->header), 0,
                            data->target_address, sizeof(struct sockaddr_in));

    if (sent < 0)
    {
        provizio_error("send_test_pointcloud: Failed to send provizio_radar_point_cloud_packet");
        return sent;
    }

    if (data->further_callback)
    {
        data->further_callback(packet, data->user_data);
    }

    return 0;
}

int send_test_point_cloud(uint16_t port, uint16_t frame_index, uint64_t timestamp, uint16_t radar_position_id,
                          uint16_t num_points, provizio_radar_point_cloud_packet_callback on_packet_sent,
                          void *user_data)
{
    PROVIZIO__SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (!provizio_socket_valid(sock))
    {
        provizio_error("send_test_pointcloud: Failed to create a UDP socket!");
        return 1;
    }

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    my_address.sin_port = 0;                        // Any port
    my_address.sin_addr.s_addr = htonl(INADDR_ANY); // Any address

    if (bind(sock, (struct sockaddr *)&my_address, sizeof(my_address)) != 0)
    {
        provizio_error("send_test_pointcloud: Failed to bind socket");
        provizio_socket_close(sock);
        return 1;
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

    if (make_test_pointcloud(frame_index, timestamp, radar_position_id, num_points, &send_point_cloud_packet,
                             &send_data) != 0)
    {
        provizio_socket_close(sock);
        return 1;
    }

    if (provizio_socket_close(sock) != 0)
    {
        provizio_error("send_test_pointcloud: Failed to close the socket");
        return 1;
    }

    return 0;
}

typedef struct test_provizio_radar_point_cloud_callback_data
{
    int called_times;
    provizio_radar_point_cloud last_point_cloud;
} test_provizio_radar_point_cloud_callback_data;

void test_provizio_radar_point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
                                              provizio_radar_point_cloud_api_context *context)
{
    test_provizio_radar_point_cloud_callback_data *data =
        (test_provizio_radar_point_cloud_callback_data *)context->user_data;

    ++data->called_times;
    data->last_point_cloud = *point_cloud;
}

int test_receive_packet_on_packet_sent(const provizio_radar_point_cloud_packet *packet, void *user_data)
{
    (void)packet;

    const int receive_result =
        provizio_radar_point_cloud_api_receive_packet((provizio_radar_point_cloud_api_context *)user_data);
    TEST_ASSERT_EQUAL_INT32(0, receive_result);

    return 0;
}

void test_provizio_radar_point_cloud_packet_size(void)
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
    TEST_ASSERT_EQUAL_STRING("num_points_in_packet exceeds PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET!",
                             provizio_test_radar_point_cloud_warning);
    provizio_set_on_warning(NULL);
}

void test_receives_correct_pointclouds(void)
{
    const uint16_t port_number = 10000 + PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT;
    const uint32_t frame_index = 17;
    const uint64_t timestamp = 0x0123456789abcdef;
    const uint16_t radar_position_id = provizio_radar_position_rear_left;
    const uint16_t num_points = 32768;

    test_provizio_radar_point_cloud_callback_data callback_data;
    memset(&callback_data, 0, sizeof(callback_data));

    provizio_radar_point_cloud_api_context api_context;
    provizio_radar_point_cloud_api_context_create(&test_provizio_radar_point_cloud_callback, &callback_data,
                                                  &api_context);
    int status = provizio_radar_point_cloud_api_context_open(&api_context, port_number, 0);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = send_test_point_cloud(port_number, frame_index, timestamp, radar_position_id, num_points,
                                   &test_receive_packet_on_packet_sent, &api_context);
    TEST_ASSERT_EQUAL_INT32(0, status);

    status = provizio_radar_point_cloud_api_context_close(&api_context);
    TEST_ASSERT_EQUAL_INT32(0, status);

    TEST_ASSERT_EQUAL_INT32(1, callback_data.called_times);
    TEST_ASSERT_EQUAL_UINT32(frame_index, callback_data.last_point_cloud.frame_index);
    TEST_ASSERT_EQUAL_UINT64(timestamp, callback_data.last_point_cloud.timestamp);
    TEST_ASSERT_EQUAL_UINT16(radar_position_id, callback_data.last_point_cloud.radar_position_id);
    TEST_ASSERT_EQUAL_UINT16(num_points, callback_data.last_point_cloud.num_points_expected);
    TEST_ASSERT_EQUAL_UINT16(num_points, callback_data.last_point_cloud.num_points_received);
}

void provizio_run_test_radar_point_cloud(void)
{
    memset(provizio_test_radar_point_cloud_warning, 0, sizeof(provizio_test_radar_point_cloud_warning));

    test_provizio_radar_point_cloud_packet_size();
    test_receives_correct_pointclouds();
}
