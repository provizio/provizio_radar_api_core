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

#include "provizio/radar_api/radar_api_context.h"
#include "provizio/radar_api/radar_point_cloud.h"

#include <assert.h>
#include <string.h>
#include <math.h>

#include "provizio/radar_api/errno.h"
#include "provizio/util.h"

static int provizio_network_floats_reversed(void)
{
    const float test_value = 1.0F;
    return ((const char *)(&test_value))[3] != 0;
}

void provizio_return_point_cloud(provizio_radar_api_context *context,
                                 provizio_radar_point_cloud *point_cloud)
{
    // First of all, make sure all older but incomplete point clouds have been already returned
#pragma unroll
    for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
    {
        provizio_radar_point_cloud *other_point_cloud = &context->impl.point_clouds_being_received[i];
        if (other_point_cloud != point_cloud && other_point_cloud->num_points_expected > 0 &&
            other_point_cloud->frame_index < point_cloud->frame_index)
        {
            // Make sure to return the obsolete point cloud first
            provizio_return_point_cloud(context, other_point_cloud);
        }
    }

    context->point_cloud_callback(point_cloud, context);
    memset(point_cloud, 0, sizeof(provizio_radar_point_cloud));
}

provizio_radar_point_cloud *provizio_get_point_cloud_being_received(
    provizio_radar_api_context *context, provizio_radar_point_cloud_packet_header *packet_header)
{
    const uint32_t small_frame_index_cap = 0x0000ffff;
    const uint32_t large_frame_index_threashold = 0xffff0000;

    provizio_radar_point_cloud *point_cloud = NULL;

    const uint16_t radar_position_id = provizio_get_protocol_field_uint16_t(&packet_header->radar_position_id);
    const uint32_t frame_index = provizio_get_protocol_field_uint32_t(&packet_header->frame_index);
    const uint16_t total_points_in_frame = provizio_get_protocol_field_uint16_t(&packet_header->total_points_in_frame);
    const uint16_t radar_mode = provizio_get_protocol_field_uint16_t(&packet_header->radar_mode);

    if (frame_index < small_frame_index_cap && context->impl.latest_frame > large_frame_index_threashold)
    {
        // A very special case: frame indices seem to have exceeded the 0xffffffff and have been reset. Let's reset the
        // state of the API to avoid complicated state-related issues.
        provizio_warning(
            "provizio_get_point_cloud_being_received: frame indices overflow detected - resetting API state");
        provizio_radar_api_context_init(context->point_cloud_callback, context->point_cloud_user_data, context->ego_motion_callback, context->ego_motion_user_data, context);
    }

    if (context->radar_position_id == provizio_radar_position_unknown)
    {
        provizio_radar_api_context_assign(context, radar_position_id);
    }
    else if (context->radar_position_id != radar_position_id)
    {
        return NULL;
    }

    if (context->impl.latest_frame < frame_index)
    {
        context->impl.latest_frame = frame_index;
    }

    // Look for a point cloud already being received
#pragma unroll
    for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
    {
        point_cloud = &context->impl.point_clouds_being_received[i];
        if (point_cloud->frame_index == frame_index && point_cloud->num_points_expected > 0)
        {
            if (point_cloud->num_points_expected != total_points_in_frame)
            {
                provizio_warning("provizio_get_point_cloud_being_received: num_points_expected mismatch across "
                                 "different packets of the same frame");
            }

            if (point_cloud->radar_mode != radar_mode)
            {
                provizio_warning("provizio_get_point_cloud_being_received: radar_mode mismatch across different "
                                 "packets of the same frame");
            }

            return point_cloud;
        }
    }

    provizio_radar_point_cloud *result = NULL;

    // Look for an empty point cloud to be used
#pragma unroll
    for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
    {
        point_cloud = &context->impl.point_clouds_being_received[i];
        if (point_cloud->num_points_expected == 0)
        {
            result = point_cloud;
        }
    }

    // We have to drop (well, return incomplete) the oldest incomplete point cloud unless it's newer than packet_header
    if (!result)
    {
#pragma unroll
        for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
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
        result->radar_position_id = radar_position_id;
        result->num_points_expected = total_points_in_frame;
        result->radar_mode = radar_mode;
        result->vs_x = NAN;
        result->vs_y = NAN;
        assert(result->num_points_received == 0);
    }

    return result;
}

size_t provizio_radar_point_cloud_packet_size(const provizio_radar_point_cloud_packet_header *header)
{
    const uint16_t num_points = provizio_get_protocol_field_uint16_t(&header->num_points_in_packet);

    if (num_points > PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET)
    {
        provizio_warning("provizio_radar_point_cloud_packet_size: num_points_in_packet exceeds "
                         "PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET!");
        return 0;
    }

    return sizeof(provizio_radar_point_cloud_packet_header) + sizeof(provizio_radar_point) * num_points;
}

int32_t provizio_check_radar_point_cloud_packet(provizio_radar_point_cloud_packet *packet, size_t packet_size)
{
    if (packet_size < sizeof(provizio_radar_api_protocol_header))
    {
        provizio_error("provizio_check_radar_point_cloud_packet: insufficient packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE)
    {
        provizio_error("provizio_check_radar_point_cloud_packet: unexpected packet_type");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION)
    {
        provizio_error("provizio_check_radar_point_cloud_packet: Incompatible protocol version");
        return PROVIZIO_E_PROTOCOL;
    }

    if (packet_size < sizeof(provizio_radar_point_cloud_packet_header))
    {
        provizio_error("provizio_check_radar_point_cloud_packet: insufficient packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    if (packet_size != provizio_radar_point_cloud_packet_size(&packet->header))
    {
        provizio_error("provizio_check_radar_point_cloud_packet: incorrect packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.radar_position_id) == provizio_radar_position_unknown)
    {
        provizio_error("provizio_check_radar_point_cloud_packet: the value of radar_position_id can't be "
                       "provizio_radar_position_unknown");
        return PROVIZIO_E_PROTOCOL;
    }

    return 0;
}

int32_t provizio_handle_radar_point_cloud_packet_checked(provizio_radar_api_context *context,
                                                         provizio_radar_point_cloud_packet *packet)
{
    provizio_radar_point_cloud *cloud = provizio_get_point_cloud_being_received(context, &packet->header);
    if (!cloud)
    {
        // Skip the packet (warning has already been published in get_point_cloud_being_received)
        return PROVIZIO_E_SKIPPED;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.total_points_in_frame) == 0)
    {
        // No points in the frame - just skip it
        return PROVIZIO_E_SKIPPED;
    }

    const uint16_t num_points_in_packet = provizio_get_protocol_field_uint16_t(&packet->header.num_points_in_packet);
    // Use uint32_t to avoid overflowing uint16_t
    if ((uint32_t)cloud->num_points_received + (uint32_t)num_points_in_packet > (uint32_t)cloud->num_points_expected)
    {
        provizio_error("provizio_handle_radar_point_cloud_packet_checked: Too many points received");
        return PROVIZIO_E_PROTOCOL;
    }

    // Append new points to the point cloud being received
    if (!provizio_network_floats_reversed())
    {
        // Optimized version: host machine uses network byte order for floats, no need to convert them
        // LCOV_EXCL_START: host CPU arch dependent
        memcpy(&cloud->radar_points[cloud->num_points_received], &packet->radar_points,
               sizeof(provizio_radar_point) * num_points_in_packet);
        // LCOV_EXCL_STOP
    }
    else
    {
        // Every value has to be converted to the host byte order
        // LCOV_EXCL_START: host CPU arch dependent
        for (uint16_t i = 0; i < num_points_in_packet; ++i)
        {
            provizio_radar_point *out_point = &cloud->radar_points[cloud->num_points_received + i];
            provizio_radar_point *in_point = &packet->radar_points[i];
            out_point->x_meters = provizio_get_protocol_field_float(&in_point->x_meters);
            out_point->y_meters = provizio_get_protocol_field_float(&in_point->y_meters);
            out_point->z_meters = provizio_get_protocol_field_float(&in_point->z_meters);
            out_point->radar_relative_radial_velocity_m_s = provizio_get_protocol_field_float(&in_point->radar_relative_radial_velocity_m_s);
            out_point->signal_to_noise_ratio = provizio_get_protocol_field_float(&in_point->signal_to_noise_ratio);
            out_point->ground_relative_radial_velocity_m_s = NAN;
        }
        // LCOV_EXCL_STOP
    }

    cloud->num_points_received += num_points_in_packet;

    if (cloud->num_points_received == cloud->num_points_expected)
    {
        provizio_return_point_cloud(context, cloud);
    }

    return 0;
}

int32_t provizio_handle_radar_point_cloud_packet(provizio_radar_api_context *context,
                                                 provizio_radar_point_cloud_packet *packet, size_t packet_size)
{
    const int32_t check_status = provizio_check_radar_point_cloud_packet(packet, packet_size);
    if (check_status != 0)
    {
        return check_status;
    }

    return provizio_handle_radar_point_cloud_packet_checked(context, packet);
}

int32_t provizio_handle_radars_point_cloud_packet(provizio_radar_api_context *contexts, size_t num_contexts,
                                                  provizio_radar_point_cloud_packet *packet, size_t packet_size)
{
    const int32_t check_status = provizio_check_radar_point_cloud_packet(packet, packet_size);
    if (check_status != 0)
    {
        return check_status;
    }

    const uint16_t radar_position_id = provizio_get_protocol_field_uint16_t(&packet->header.radar_position_id);

    provizio_radar_api_context *context =
        provizio_get_radar_api_context_by_position_id(contexts, num_contexts, radar_position_id);

    if (!context)
    {
        // Error message has been already posted by provizio_get_radar_api_context_by_position_id
        return PROVIZIO_E_OUT_OF_CONTEXTS;
    }

    return provizio_handle_radar_point_cloud_packet_checked(context, packet);
}

int32_t provizio_handle_possible_radar_point_cloud_packet(provizio_radar_api_context *context,
                                                          const void *payload, size_t payload_size)
{
    return provizio_handle_possible_radars_point_cloud_packet(context, 1, payload, payload_size);
}

int32_t provizio_handle_possible_radars_point_cloud_packet(provizio_radar_api_context *contexts,
                                                           size_t num_contexts, const void *payload,
                                                           size_t payload_size)
{
    if (payload_size < sizeof(provizio_radar_point_cloud_packet_header))
    {
        // Not enough data
        return PROVIZIO_E_SKIPPED;
    }

    provizio_radar_point_cloud_packet_header *packet_header = (provizio_radar_point_cloud_packet_header *)payload;
    if (provizio_get_protocol_field_uint16_t(&packet_header->protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE)
    {
        // Non-point cloud packet
        return PROVIZIO_E_SKIPPED;
    }

    if (provizio_get_protocol_field_uint16_t(&packet_header->protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION)
    {
        provizio_error("provizio_handle_possible_radar_point_cloud_packet: Incompatible protocol version");
        return PROVIZIO_E_PROTOCOL;
    }

    return num_contexts != 1
               ? provizio_handle_radars_point_cloud_packet(contexts, num_contexts,
                                                           (provizio_radar_point_cloud_packet *)payload, payload_size)
               : provizio_handle_radar_point_cloud_packet(contexts, (provizio_radar_point_cloud_packet *)payload,
                                                          payload_size);
}
