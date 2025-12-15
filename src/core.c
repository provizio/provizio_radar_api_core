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
#include "provizio/util.h"

#include <stdio.h>
#include <string.h>

int32_t provizio_open_radar_connection(uint16_t udp_port, uint64_t receive_timeout_ns, uint8_t check_connection,
                                       provizio_radar_point_cloud_api_context *radar_point_cloud_api_context,
                                       provizio_radar_entities_api_context *radar_entities_api_context,
                                       provizio_radar_api_connection *out_connection)
{
    return provizio_open_radars_connection(udp_port, receive_timeout_ns, check_connection,
                                           radar_point_cloud_api_context, radar_point_cloud_api_context != NULL ? 1 : 0,
                                           radar_entities_api_context, radar_entities_api_context != NULL ? 1 : 0,
                                           out_connection);
}

int32_t provizio_open_radars_connection(uint16_t udp_port, uint64_t receive_timeout_ns, uint8_t check_connection,
                                        provizio_radar_point_cloud_api_context *radar_point_cloud_api_contexts,
                                        size_t num_radar_point_cloud_api_contexts,
                                        provizio_radar_entities_api_context *radar_entities_api_contexts,
                                        size_t num_radar_entities_api_contexts,
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

    // Enable broadcasting support
    const int broadcast = 1;
    status = (int32_t)setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char *)&broadcast, sizeof(broadcast));
    if (status != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_warning("provizio_open_radars_connection: Enabling broadcasting failed!");
        // LCOV_EXCL_STOP
    }

    // Enable address and port reuse, so multiple processes can receive same packets
    status = provizio_socket_enable_address_and_port_reuse(sock);
    if (status != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_warning("provizio_open_radars_connection: Enabling address & port reuse failed!");
        // LCOV_EXCL_STOP
    }

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    // linting disabled in next line as htons implementation is up to a platform (it uses asm instructions in some
    // platforms, which clang-tidy hates)
    my_address.sin_port = htons(udp_port != 0 ? udp_port : PROVIZIO__RADAR_API_DEFAULT_PORT); // NOLINT
    my_address.sin_addr.s_addr = INADDR_ANY;                                                  // Any address

    status = (int32_t)bind(sock, (struct sockaddr *)&my_address, sizeof(my_address));
    if (status != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("provizio_open_radars_connection: Failed to bind a UDP socket!");
        provizio_socket_close(sock);
        return status;
        // LCOV_EXCL_STOP
    }

    if (check_connection)
    {
        provizio_verbose("provizio_open_radars_connection: Checking connection...");

        provizio_radar_point_cloud_packet packet;
        int32_t received = (int32_t)recv(sock, (char *)&packet, sizeof(packet), 0);
        if (received == (int32_t)-1)
        {
            const int32_t error_code = (int32_t)errno;

            provizio_socket_close(sock);

            if (error_code == 0 || error_code == (int32_t)EWOULDBLOCK)
            {
                provizio_verbose("provizio_open_radars_connection: Timed out on connection check");
                return PROVIZIO_E_TIMEOUT;
            }

            // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
            provizio_verbose("provizio_open_radars_connection: Failure on connection check");
            return error_code;
            // LCOV_EXCL_STOP
        }
    }

    out_connection->sock = sock;
    out_connection->radar_point_cloud_api_contexts = radar_point_cloud_api_contexts;
    out_connection->num_radar_point_cloud_api_contexts = num_radar_point_cloud_api_contexts;
    out_connection->radar_entities_api_contexts = radar_entities_api_contexts;
    out_connection->num_radar_entities_api_contexts = num_radar_entities_api_contexts;

    provizio_verbose("provizio_open_radars_connection: Connected");

    return 0;
}

int32_t provizio_radar_api_receive_packet(provizio_radar_api_connection *connection)
{
    provizio_verbose("provizio_radar_api_receive_packet: Receiving next packet...");

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

        provizio_verbose("provizio_radar_api_receive_packet: Timed out");
        return (int32_t)PROVIZIO_E_TIMEOUT;
    }

    provizio_verbose("provizio_radar_api_receive_packet: Received a packet");

    int32_t status_code = PROVIZIO_E_SKIPPED;

    // Try handling it as a point cloud packet
    if (status_code == PROVIZIO_E_SKIPPED && connection->num_radar_point_cloud_api_contexts > 0 &&
        connection->radar_point_cloud_api_contexts != NULL)
    {
        status_code = provizio_handle_possible_radars_point_cloud_packet(connection->radar_point_cloud_api_contexts,
                                                                         connection->num_radar_point_cloud_api_contexts,
                                                                         packet, received);
    }

    // Try handling it as an entities packet
    if (status_code == PROVIZIO_E_SKIPPED && connection->num_radar_entities_api_contexts > 0 &&
        connection->radar_entities_api_contexts != NULL)
    {
        status_code = provizio_handle_possible_radars_entities_packet(
            connection->radar_entities_api_contexts, connection->num_radar_entities_api_contexts, packet, received);
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

    provizio_verbose("provizio_close_radars_connection: Connection closed");

    return 0;
}

int32_t provizio_set_radar_range(provizio_radar_position radar_position_id, provizio_radar_range range,
                                 uint16_t udp_port, const char *ipv4_address,
                                 provizio_radar_range *out_actual_radar_range)
{
    return provizio_set_radar_range_with_timeout(radar_position_id, range, udp_port, ipv4_address,
                                                 out_actual_radar_range, PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_TIMEOUT);
}

int32_t provizio_set_radar_range_with_timeout(provizio_radar_position radar_position_id, provizio_radar_range range,
                                              uint16_t udp_port, const char *ipv4_address,
                                              provizio_radar_range *out_actual_radar_range, uint64_t timeout_ns)
{
    const char *broadcast_ipv4_address = "255.255.255.255";
    const uint64_t recv_timeout_ns = 250000000; // 0.25s
    const int max_ack_recv_tries = 6;

    provizio_verbose("provizio_set_radar_range_with_timeout: Setting a radar range of %d to %d at %s:%d...",
                     (int)radar_position_id, (int)range, (ipv4_address != NULL ? ipv4_address : broadcast_ipv4_address),
                     (int)udp_port);

    if (out_actual_radar_range != NULL)
    {
        *out_actual_radar_range = provizio_radar_range_unknown;
    }

    if ((timeout_ns / recv_timeout_ns) + max_ack_recv_tries > INT32_MAX)
    {
        provizio_error("provizio_set_radar_range_with_timeout: timeout_ns is too large");
        return PROVIZIO_E_ARGUMENT;
    }

    PROVIZIO__SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    int32_t status = -1;

    if (!provizio_socket_valid(sock))
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        status = (int32_t)errno;
        provizio_error("provizio_set_radar_range_with_timeout: Failed to create a UDP socket!");
        return status != 0 ? status : (int32_t)-1;
        // LCOV_EXCL_STOP
    }

    status = provizio_socket_set_recv_timeout(sock, recv_timeout_ns);
    if (status != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("provizio_set_radar_range_with_timeout: Failed to set recv timeout!");
        provizio_socket_close(sock);
        return status;
        // LCOV_EXCL_STOP
    }

    // Enable broadcasting if required
    if (ipv4_address == NULL || strstr(ipv4_address, "255") != NULL)
    {
        const int broadcast = 1;
        status = (int32_t)setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char *)&broadcast, sizeof(broadcast));
        if (status != 0)
        {
            // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
            provizio_error("provizio_set_radar_range_with_timeout: Failed to enable broadcasting!");
            provizio_socket_close(sock);
            return status;
            // LCOV_EXCL_STOP
        }
#if defined(IPPROTO_IP) && defined(IP_ONESBCAST)
        if (ipv4_address == NULL || strcmp(ipv4_address, broadcast_ipv4_address) == 0)
        {
            const int ones_broadcast = 1;
            status = (int32_t)setsockopt(sock, IPPROTO_IP, IP_ONESBCAST, (const char *)&ones_broadcast,
                                         sizeof(ones_broadcast));
            if (status != 0)
            {
                // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
                provizio_error("provizio_set_radar_range_with_timeout: Failed to enable limited broadcasting support!");
                provizio_socket_close(sock);
                return status;
                // LCOV_EXCL_STOP
            }
        }
#endif
    }

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    my_address.sin_port = 0;                 // Any port
    my_address.sin_addr.s_addr = INADDR_ANY; // Any address

    status = bind(sock, (struct sockaddr *)&my_address, sizeof(my_address));
    if (status != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("provizio_set_radar_range_with_timeout: Failed to bind socket");
        provizio_socket_close(sock);
        return status;
        // LCOV_EXCL_STOP
    }

    struct sockaddr_in target_address;
    memset(&target_address, 0, sizeof(target_address));
    target_address.sin_family = AF_INET;
    // linting disabled in next line as htons implementation is up to a platform (it uses asm instructions in some
    // platforms, which clang-tidy hates)
    target_address.sin_port = htons(udp_port != 0 ? udp_port : PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_PORT); // NOLINT
    target_address.sin_addr.s_addr = inet_addr(ipv4_address != NULL ? ipv4_address : broadcast_ipv4_address);

    provizio_set_radar_range_packet set_range_packet;
    memset(&set_range_packet, 0, sizeof(set_range_packet));
    provizio_set_protocol_field_uint16_t(&set_range_packet.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_SET_RANGE_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&set_range_packet.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_RANGE_PROTOCOL_VERSION);
    provizio_set_protocol_field_uint16_t(&set_range_packet.radar_position_id, radar_position_id);
    provizio_set_protocol_field_uint16_t(&set_range_packet.radar_range, range);

    provizio_set_radar_range_response_packet response_packet;
    memset(&response_packet, 0, sizeof(response_packet));

    // Send the request and wait for an acknowledgement packet
    int8_t acknowledgement_received = 0;
    int recv_tries = max_ack_recv_tries;
    do
    {
        status = (int32_t)sendto(sock, (const char *)&set_range_packet, // NOLINT
                                 sizeof(set_range_packet), 0, (const struct sockaddr *)&target_address,
                                 sizeof(struct sockaddr_in));

        if (status < 0)
        {
            // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
#define PROVIZIO__ERROR_MESSAGE_BUFFER_SIZE (1024)
            const int32_t send_error = (int32_t)errno;
            char error_message_buffer[PROVIZIO__ERROR_MESSAGE_BUFFER_SIZE];
            (void)snprintf(error_message_buffer, PROVIZIO__ERROR_MESSAGE_BUFFER_SIZE,
                           "provizio_set_radar_range_with_timeout: Failed to send provizio_set_radar_range_packet - "
                           "errno %d",
                           (int)send_error);
            provizio_error(error_message_buffer);
            provizio_socket_close(sock);
            return send_error != 0 ? send_error : (int32_t)-1;
#undef PROVIZIO__ERROR_MESSAGE_BUFFER_SIZE
            // LCOV_EXCL_STOP
        }

        // Receive an acknowledgement
        status = (int32_t)recv(sock, (char *)&response_packet, sizeof(response_packet), 0);
        uint16_t packet_type = 0;
        if (status >= (int32_t)sizeof(provizio_radar_api_protocol_header))
        {
            packet_type = provizio_get_protocol_field_uint16_t(&response_packet.protocol_header.packet_type);
            if (packet_type != PROVIZIO__RADAR_API_SET_RANGE_ACKNOWLEDGEMENT_PACKET_TYPE &&
                packet_type != PROVIZIO__RADAR_API_SET_RANGE_RESPONSE_PACKET_TYPE)
            {
                provizio_error("provizio_set_radar_range_with_timeout: Invalid acknowledgement packet type received");
                provizio_socket_close(sock);
                return PROVIZIO_E_PROTOCOL;
            }

            if (provizio_get_protocol_field_uint16_t(&response_packet.protocol_header.protocol_version) !=
                PROVIZIO__RADAR_API_RANGE_PROTOCOL_VERSION)
            {
                provizio_error("provizio_set_radar_range_with_timeout: Incompatible protocol version");
                provizio_socket_close(sock);
                return PROVIZIO_E_PROTOCOL;
            }
        }

        if (status == sizeof(response_packet))
        {
            if (packet_type != PROVIZIO__RADAR_API_SET_RANGE_ACKNOWLEDGEMENT_PACKET_TYPE ||
                (radar_position_id != provizio_radar_position_any &&
                 provizio_get_protocol_field_uint16_t(&response_packet.radar_position_id) != radar_position_id) ||
                provizio_get_protocol_field_uint16_t(&response_packet.requested_radar_range) != range)
            {
                // Correct packet format, but from a previous provizio_set_radar_range request: keep
                // waiting for the correct one
                provizio_verbose("provizio_set_radar_range_with_timeout: Attempt timeout");
                status = PROVIZIO_E_TIMEOUT;
            }
            else
            {
                // Correct acknowledgement
                acknowledgement_received = 1;

                if (out_actual_radar_range != NULL)
                {
                    *out_actual_radar_range =
                        provizio_get_protocol_field_uint16_t(&response_packet.current_radar_range);
                }

                status = (int32_t)provizio_get_protocol_field_uint32_t((uint32_t *)&response_packet.error_code);
                if (status == 0)
                {
                    provizio_verbose("provizio_set_radar_range_with_timeout: Received acknowledgement - Success");
                }
                else
                {
                    provizio_verbose("provizio_set_radar_range_with_timeout: Received acknowledgement - Failure");
                }
            }
        }
        else
        {
            status = errno;
            if (status == 0 || status == (int32_t)EWOULDBLOCK)
            {
                provizio_verbose("provizio_set_radar_range_with_timeout: Attempt timeout");
                status = PROVIZIO_E_TIMEOUT;
            }
        }
    } while (status == PROVIZIO_E_TIMEOUT && --recv_tries > 0);

    // Waiting for the response on operation completion now, unless acknowledgment phase failed or the range is already
    // as requested
    if (status == 0 && provizio_get_protocol_field_uint16_t(&response_packet.current_radar_range) != range)
    {
        // Using timeout_ns and knowing how many timed out receive tries has already been done on the acknowledgement
        // step (i.e. time already spent), calculate how many tries we still have left for receiving the response.
        // Even if resulting value is <= 0, at least a single receive attempt will be done by design thanks to do..while
        recv_tries = (int)(timeout_ns / recv_timeout_ns) - (max_ack_recv_tries - recv_tries);

        do
        {
            // Receive a response
            status = (int32_t)recv(sock, (char *)&response_packet, sizeof(response_packet), 0);
            uint16_t packet_type = 0;
            if (status >= (int32_t)sizeof(provizio_radar_api_protocol_header))
            {
                packet_type = provizio_get_protocol_field_uint16_t(&response_packet.protocol_header.packet_type);
                if (packet_type != PROVIZIO__RADAR_API_SET_RANGE_ACKNOWLEDGEMENT_PACKET_TYPE &&
                    packet_type != PROVIZIO__RADAR_API_SET_RANGE_RESPONSE_PACKET_TYPE)
                {
                    provizio_error("provizio_set_radar_range_with_timeout: Invalid response packet type received");
                    provizio_socket_close(sock);
                    return PROVIZIO_E_PROTOCOL;
                }

                if (provizio_get_protocol_field_uint16_t(&response_packet.protocol_header.protocol_version) !=
                    PROVIZIO__RADAR_API_RANGE_PROTOCOL_VERSION)
                {
                    provizio_error("provizio_set_radar_range_with_timeout: Incompatible protocol version");
                    provizio_socket_close(sock);
                    return PROVIZIO_E_PROTOCOL;
                }
            }

            if (status == sizeof(response_packet))
            {
                if (packet_type != PROVIZIO__RADAR_API_SET_RANGE_RESPONSE_PACKET_TYPE ||
                    (radar_position_id != provizio_radar_position_any &&
                     provizio_get_protocol_field_uint16_t(&response_packet.radar_position_id) != radar_position_id) ||
                    provizio_get_protocol_field_uint16_t(&response_packet.requested_radar_range) != range)
                {
                    // Correct packet format, but from a previous provizio_set_radar_range request: keep
                    // waiting for the correct one
                    provizio_verbose("provizio_set_radar_range_with_timeout: Attempt timeout");
                    status = PROVIZIO_E_TIMEOUT;
                }
                else
                {
                    // Correct response
                    if (out_actual_radar_range != NULL)
                    {
                        *out_actual_radar_range =
                            provizio_get_protocol_field_uint16_t(&response_packet.current_radar_range);
                    }

                    status = (int32_t)provizio_get_protocol_field_uint32_t((uint32_t *)&response_packet.error_code);
                    if (status == 0)
                    {
                        provizio_verbose("provizio_set_radar_range_with_timeout: Received acknowledgement - Success");
                    }
                    else
                    {
                        provizio_verbose("provizio_set_radar_range_with_timeout: Received acknowledgement - Failure");
                    }
                }
            }
            else
            {
                status = errno;
                if (status == 0 || status == (int32_t)EWOULDBLOCK)
                {
                    provizio_verbose("provizio_set_radar_range_with_timeout: Attempt timeout");
                    status = PROVIZIO_E_TIMEOUT;
                }
            }
        } while (status == PROVIZIO_E_TIMEOUT && --recv_tries > 0);
    }

    if (status != 0)
    {
        if (status == PROVIZIO_E_TIMEOUT)
        {
            if (!acknowledgement_received)
            {
                provizio_error("provizio_set_radar_range_with_timeout: No acknowledgement received, likely due to a "
                               "connection issue");
            }
            else
            {
                provizio_error(
                    "provizio_set_radar_range_with_timeout: Received the acknowledgement but not the response");
            }
        }
        else if (range != provizio_radar_range_unknown) // Unknown is used for requesting current range, not an error
        {
            provizio_error("provizio_set_radar_range_with_timeout: Failed to set the requested range");
        }

        provizio_socket_close(sock);
        return status;
    }

    status = provizio_socket_close(sock);
    if (status != 0)
    {
        // LCOV_EXCL_START: Can't be unit-tested as it depends on the state of the OS
        provizio_error("provizio_set_radar_range_with_timeout: Failed to close the socket");
        return status;
        // LCOV_EXCL_STOP
    }

    provizio_verbose("provizio_set_radar_range_with_timeout: Success");

    return 0;
}

int32_t provizio_request_current_range(provizio_radar_position radar_position_id, uint16_t udp_port,
                                       const char *ipv4_address, provizio_radar_range *out_current_radar_range)
{
    if (out_current_radar_range == NULL)
    {
        provizio_error("provizio_request_current_range: out_current_radar_range argument can't be NULL");
        return PROVIZIO_E_ARGUMENT;
    }

    // "Set range to provizio_radar_range_unknown" is used to detect the current range
    *out_current_radar_range = provizio_radar_range_unknown;
    int32_t status = provizio_set_radar_range(radar_position_id, provizio_radar_range_unknown, udp_port, ipv4_address,
                                              out_current_radar_range);
    return *out_current_radar_range != provizio_radar_range_unknown ? 0 : status;
}
