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

#include <string.h>

#include "provizio/util.h"

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

    // TODO: provizio_radar_point_cloud_api_context_create
    (void)callback;
    (void)user_data;
}

void provizio_handle_radar_point_cloud_packet(provizio_radar_point_cloud_api_context *context,
                                              provizio_radar_point_cloud_packet *packet)
{
    // TODO: provizio_handle_radar_point_cloud_packet
    (void)context;
    (void)packet;
}

bool provizio_handle_possible_radar_point_cloud_packet(provizio_radar_point_cloud_api_context *context,
                                                       const uint8_t *payload, size_t payload_size)
{
    // TODO: provizio_handle_possible_radar_point_cloud_packet
    (void)context;
    (void)payload;
    (void)payload_size;
    return false;
}

int provizio_radar_point_cloud_api_context_open(provizio_radar_point_cloud_api_context *context, uint16_t udp_port,
                                                uint64_t receive_timeout)
{
    // TODO: provizio_radar_point_cloud_api_context_open
    (void)context;
    (void)udp_port;
    (void)receive_timeout;
    return 1;
}

int provizio_radar_point_cloud_api_receive_packet(provizio_radar_point_cloud_api_context *context)
{
    // TODO: provizio_radar_point_cloud_api_receive_packet
    (void)context;
    return 1;
}

int provizio_radar_point_cloud_api_context_close(provizio_radar_point_cloud_api_context *context)
{
    // TODO: provizio_radar_point_cloud_api_context_close
    (void)context;
    return 1;
}
