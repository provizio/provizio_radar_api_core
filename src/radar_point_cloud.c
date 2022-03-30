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

#include "provizio/radar_api/radar_point_cloud.h"

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "provizio/util.h"

void provizio_return_point_cloud(provizio_radar_point_cloud_api_context *context,
                                 provizio_radar_point_cloud *point_cloud)
{
    // First of all, make sure all older but incomplete point clouds have been already returned
    for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
    {
        provizio_radar_point_cloud *other_point_cloud = &context->impl.point_clouds_being_received[i];
        if (other_point_cloud != point_cloud && other_point_cloud->num_points_expected > 0 &&
            other_point_cloud->frame_index < point_cloud->frame_index)
        {
            // Make sure to return the obsolete point cloud first
            provizio_return_point_cloud(context, other_point_cloud);
        }
    }

    context->callback(point_cloud, context);
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));
}

provizio_radar_point_cloud *provizio_get_point_cloud_being_received(
    provizio_radar_point_cloud_api_context *context, provizio_radar_point_cloud_packet_header *packet_header)
{
    provizio_radar_point_cloud *point_cloud = NULL;

    const uint32_t frame_index = provizio_get_protocol_field_uint32_t(&packet_header->frame_index);
    const uint16_t total_points_in_frame = provizio_get_protocol_field_uint16_t(&packet_header->total_points_in_frame);

    if (frame_index < (uint32_t)0x0000ffff && context->impl.latest_frame > (uint32_t)0xffff0000)
    {
        // A very special case: frame indices seem to have exceeded the 0xffffffff and have been reset. Let's drop all
        // we have so far, resetting the state of the API.
        memset(&context->impl.point_clouds_being_received, 0,
               sizeof(provizio_radar_point_cloud) *
                   PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT);
        context->impl.latest_frame = 0;
    }

    if (context->impl.latest_frame < frame_index)
    {
        context->impl.latest_frame = frame_index;
    }

    // Look for a point cloud already being received
    for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
    {
        point_cloud = &context->impl.point_clouds_being_received[i];
        if (point_cloud->frame_index == frame_index && point_cloud->num_points_expected > 0)
        {
            if (point_cloud->num_points_expected != total_points_in_frame)
            {
                provizio_warning("provizio_get_point_cloud_being_received: num_points_expected mismatch across "
                                 "different packets of the same frame");
            }

            return point_cloud;
        }
    }

    provizio_radar_point_cloud *result = NULL;

    // Look for an empty point cloud to be used
    for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
    {
        point_cloud = &context->impl.point_clouds_being_received[i];
        if (point_cloud->num_points_expected == 0)
        {
            result = point_cloud;
        }
    }

    // We have to drop the oldest incomplete point cloud unless it's newer than packet_header
    if (!result)
    {
        for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
        {
            point_cloud = &context->impl.point_clouds_being_received[i];
            if (point_cloud->frame_index < frame_index && (!result || point_cloud->frame_index < result->frame_index))
            {
                result = point_cloud;
            }
        }

        if (result != NULL)
        {
            // Return the obsolete incomplete point cloud
            provizio_return_point_cloud(context, result);
        }
    }

    if (result != NULL)
    {
        // Initialize the point cloud
        result->frame_index = frame_index;
        result->timestamp = provizio_get_protocol_field_uint64_t(&packet_header->timestamp);
        result->radar_position_id = provizio_get_protocol_field_uint16_t(&packet_header->radar_position_id);
        result->num_points_expected = total_points_in_frame;
        assert(result->num_points_received == 0);
    }

    return result;
}

size_t provizio_radar_point_cloud_packet_size(const provizio_radar_point_cloud_packet_header *header)
{
    const uint16_t num_points = provizio_get_protocol_field_uint16_t(&header->num_points_in_packet);

    if (num_points > PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET)
    {
        provizio_warning("num_points_in_packet exceeds PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET!");
        return 0;
    }

    return sizeof(provizio_radar_point_cloud_packet_header) + sizeof(provizio_radar_point) * num_points;
}

void provizio_radar_point_cloud_api_context_create(provizio_radar_point_cloud_callback callback, void *user_data,
                                                   provizio_radar_point_cloud_api_context *out_context)
{
    memset(out_context, 0, sizeof(provizio_radar_point_cloud_api_context));

    out_context->callback = callback;
    out_context->user_data = user_data;
}

int32_t provizio_handle_radar_point_cloud_packet(provizio_radar_point_cloud_api_context *context,
                                                 provizio_radar_point_cloud_packet *packet, size_t packet_size)
{
    if (packet_size < sizeof(provizio_radar_point_cloud_packet_protocol_header))
    {
        provizio_error("provizio_handle_radar_point_cloud_packet: insufficient packet_size");
        return EPROTO;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE)
    {
        provizio_error("provizio_handle_radar_point_cloud_packet: unexpected packet_type");
        return EPROTO;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION)
    {
        provizio_error("provizio_handle_radar_point_cloud_packet: Incompatible protocol version");
        return EPROTO;
    }

    if (packet_size < sizeof(provizio_radar_point_cloud_packet_header))
    {
        provizio_error("provizio_handle_radar_point_cloud_packet: insufficient packet_size");
        return EPROTO;
    }

    if (packet_size != provizio_radar_point_cloud_packet_size(&packet->header))
    {
        provizio_error("provizio_handle_radar_point_cloud_packet: incorrect packet_size");
        return EPROTO;
    }

    provizio_radar_point_cloud *cloud = provizio_get_point_cloud_being_received(context, &packet->header);
    if (!cloud)
    {
        // Skip the packet (warning has already been published in get_point_cloud_being_received)
        return EAGAIN;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.total_points_in_frame) == 0)
    {
        // No points in the frame - just skip it
        return EAGAIN;
    }

    const uint16_t num_points_in_packet = provizio_get_protocol_field_uint16_t(&packet->header.num_points_in_packet);
    // Use uint32_t to avoid overflowing uint16_t
    if ((uint32_t)cloud->num_points_received + (uint32_t)num_points_in_packet > (uint32_t)cloud->num_points_expected)
    {
        provizio_error("provizio_handle_radar_point_cloud_packet: Too many points received");
        return EPROTO;
    }

    memcpy(&cloud->radar_points[cloud->num_points_received], &packet->radar_points,
           sizeof(provizio_radar_point) * num_points_in_packet);
    cloud->num_points_received += num_points_in_packet;

    if (cloud->num_points_received == cloud->num_points_expected)
    {
        provizio_return_point_cloud(context, cloud);
    }

    return 0;
}

int32_t provizio_handle_possible_radar_point_cloud_packet(provizio_radar_point_cloud_api_context *context,
                                                          const void *payload, size_t payload_size)
{
    if (payload_size < sizeof(provizio_radar_point_cloud_packet_header))
    {
        // Not enough data
        return EAGAIN;
    }

    provizio_radar_point_cloud_packet_header *packet_header = (provizio_radar_point_cloud_packet_header *)payload;
    if (provizio_get_protocol_field_uint16_t(&packet_header->protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE)
    {
        // Non-point cloud packet
        return EAGAIN;
    }

    if (provizio_get_protocol_field_uint16_t(&packet_header->protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION)
    {
        provizio_error("provizio_handle_possible_radar_point_cloud_packet: Incompatible protocol version");
        return EPROTO;
    }

    return provizio_handle_radar_point_cloud_packet(context, (provizio_radar_point_cloud_packet *)payload,
                                                    payload_size);
}

int32_t provizio_radar_point_cloud_api_context_open(provizio_radar_point_cloud_api_context *context, uint16_t udp_port,
                                                    uint64_t receive_timeout)
{
    if (context->impl.sock != 0)
    {
        provizio_error("provizio_radar_point_cloud_api_context_open: Already opened!");
        return EBUSY;
    }

    PROVIZIO__SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (!provizio_socket_valid(sock))
    {
        provizio_error("provizio_radar_point_cloud_api_context_open: Failed to create a UDP socket!");
        return sock;
    }

    int32_t status;
    if (receive_timeout)
    {
        status = (int32_t)provizio_socket_set_recv_timeout(sock, receive_timeout);
        if (status != 0)
        {
            provizio_error("provizio_radar_point_cloud_api_context_open: Setting receive_timeout failed!");
            provizio_socket_close(sock);
            return status;
        }
    }

    struct sockaddr_in my_address;
    memset(&my_address, 0, sizeof(my_address));
    my_address.sin_family = AF_INET;
    my_address.sin_port = htons(udp_port);
    my_address.sin_addr.s_addr = INADDR_ANY; // Any address

    status = (int32_t)bind(sock, (struct sockaddr *)&my_address, sizeof(my_address));
    if (status != 0)
    {
        provizio_error("provizio_radar_point_cloud_api_context_open: Failed to bind a UDP socket!");
        provizio_socket_close(sock);
        return status;
    }

    context->impl.sock = sock;
    return 0;
}

int32_t provizio_radar_point_cloud_api_receive_packet(provizio_radar_point_cloud_api_context *context)
{
    provizio_radar_point_cloud_packet packet;

    int received = recv(context->impl.sock, &packet, sizeof(packet), 0);
    if (received == -1)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            provizio_error("provizio_radar_point_cloud_api_receive_packet: Failed to receive");
            return (int32_t)errno;
        }

        return (int32_t)EAGAIN;
    }

    return provizio_handle_radar_point_cloud_packet(context, &packet, (size_t)received);
}

int32_t provizio_radar_point_cloud_api_context_close(provizio_radar_point_cloud_api_context *context)
{
    int32_t status = provizio_socket_close(context->impl.sock);
    if (status != 0)
    {
        provizio_error("provizio_radar_point_cloud_api_context_close: provizio_socket_close failed!");
        return status;
    }

    return 0;
}
