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

#include "provizio/radar_api/core.h"

#include <string.h>

int32_t provizio_open_radar_connection(uint16_t udp_port, uint64_t receive_timeout_ns, uint8_t check_connection,
                                       provizio_radar_point_cloud_api_context *radar_point_cloud_api_context,
                                       provizio_radar_api_connection *out_connection)
{
    return provizio_open_radars_connection(udp_port, receive_timeout_ns, check_connection,
                                           radar_point_cloud_api_context, radar_point_cloud_api_context != NULL ? 1 : 0,
                                           out_connection);
}

int32_t provizio_open_radars_connection(uint16_t udp_port, uint64_t receive_timeout_ns, uint8_t check_connection,
                                        provizio_radar_point_cloud_api_context *radar_point_cloud_api_contexts,
                                        size_t num_radar_point_cloud_api_contexts,
                                        provizio_radar_api_connection *out_connection)
{
    memset(out_connection, 0, sizeof(provizio_radar_api_connection));
    out_connection->sock = PROVIZIO__INVALID_SOCKET;

    PROVIZIO__SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (!provizio_socket_valid(sock))
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        const int32_t status = errno;
        provizio_error("provizio_open_radars_connection: Failed to create a UDP socket!"
#ifdef _WIN32
                       " Have you called provizio_sockets_initialize or WSAStartup?"
#endif
        );
        return status != 0 ? status : -1;
        // LCOV_EXCL_STOP
    }

    int32_t status = 0;
    if (receive_timeout_ns)
    {
        // Despite clang-tidy suggestion, (int32_t) may be required as it's platform dependent
        status = (int32_t)provizio_socket_set_recv_timeout(sock, receive_timeout_ns); // NOLINT
        if (status != 0)
        {
            // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
            provizio_error("provizio_open_radars_connection: Setting timeout failed!");
            provizio_socket_close(sock);
            return status;
            // LCOV_EXCL_STOP
        }
    }

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    // linting disabled in next line as htons implementation is up to a platform (it uses asm instructions in some
    // platforms, which clang-tidy hates)
    my_address.sin_port = htons(udp_port);   // NOLINT
    my_address.sin_addr.s_addr = INADDR_ANY; // Any address

    status = (int32_t)bind(sock, (struct sockaddr *)&my_address, sizeof(my_address));
    if (status != 0)
    {
        provizio_error("provizio_open_radars_connection: Failed to bind a UDP socket!");
        provizio_socket_close(sock);
        return status;
    }

    if (check_connection)
    {
        provizio_radar_point_cloud_packet packet;
        int32_t received = (int32_t)recv(sock, (char *)&packet, sizeof(packet), 0);
        if (received == (int32_t)-1)
        {
            const int32_t error_code = (int32_t)errno;

            provizio_socket_close(sock);

            if (error_code == 0 || error_code == (int32_t)EWOULDBLOCK)
            {
                return PROVIZIO_E_TIMEOUT;
            }

            return error_code; // LCOV_EXCL_LINE: Can't be unit-tested as it depends on the state of the OS
        }
    }

    out_connection->sock = sock;
    out_connection->radar_point_cloud_api_contexts = radar_point_cloud_api_contexts;
    out_connection->num_radar_point_cloud_api_contexts = num_radar_point_cloud_api_contexts;

    return 0;
}

int32_t provizio_radar_api_receive_packet(provizio_radar_api_connection *connection)
{
    if (!provizio_socket_valid(connection->sock))
    {
        provizio_error("provizio_radar_api_receive_packet: Not connected");
        return PROVIZIO_E_ARGUMENT;
    }

    uint8_t packet[PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES];
    int32_t received = (int32_t)recv(connection->sock, (char *)&packet, PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES, 0);
    if (received == (int32_t)-1)
    {
        if (errno != 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
            const int32_t status_code = errno;
            provizio_error("provizio_radar_api_receive_packet: Failed to receive");
            return (int32_t)status_code; // NOLINT: Type cast is platform specific
            // LCOV_EXCL_STOP
        }

        return (int32_t)PROVIZIO_E_TIMEOUT;
    }

    int32_t status_code = PROVIZIO_E_SKIPPED;

    // Let's try to handle it as a point cloud packet
    if (status_code == PROVIZIO_E_SKIPPED && connection->num_radar_point_cloud_api_contexts > 0 &&
        connection->radar_point_cloud_api_contexts != NULL)
    {
        status_code = provizio_handle_possible_radars_point_cloud_packet(connection->radar_point_cloud_api_contexts,
                                                                         connection->num_radar_point_cloud_api_contexts,
                                                                         packet, received);
    }

    return status_code;
}

int32_t provizio_close_radars_connection(provizio_radar_api_connection *connection)
{
    if (!provizio_socket_valid(connection->sock))
    {
        provizio_error("provizio_close_radars_connection: Not connected");
        return PROVIZIO_E_ARGUMENT;
    }

    int32_t status = provizio_socket_close(connection->sock);
    if (status != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("provizio_close_radars_connection: provizio_socket_close failed!");
        return status;
        // LCOV_EXCL_STOP
    }

    connection->sock = PROVIZIO__INVALID_SOCKET;
    return 0;
}
