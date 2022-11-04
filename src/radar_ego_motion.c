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
#include "provizio/radar_api/radar_ego_motion.h"
#include "provizio/radar_api/radar_point_cloud.h"

#include <assert.h>
#include <string.h>
#include <math.h>

#include "provizio/radar_api/errno.h"
#include "provizio/util.h"


void provizio_radar_ego_motion_basic_callback(const provizio_radar_ego_motion *ego_motion,
                                              struct provizio_radar_api_context *context)
{
#pragma unroll
    for(size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; i++) {
        if(ego_motion->frame_index == context->impl.point_clouds_being_received[i].frame_index) {
            context->impl.point_clouds_being_received[i].radar_velocity_x_m_s = ego_motion->radar_velocity_x_m_s;
            context->impl.point_clouds_being_received[i].radar_velocity_y_m_s = ego_motion->radar_velocity_y_m_s;
            break;
        }
    }
}

int provizio_radar_ego_motion_calculate_ground_relative_radial_velocity(const provizio_radar_ego_motion *ego_motion,
                                                                        provizio_radar_point_cloud *point_cloud)
{
    float radar_velocity_x_m_s = ego_motion->radar_velocity_x_m_s;
    float radar_velocity_y_m_s = ego_motion->radar_velocity_y_m_s;

    if(isnan(radar_velocity_x_m_s) || isnan(radar_velocity_y_m_s)) {
        return PROVIZIO_E_ARGUMENT;
    }

    for(size_t i = 0; i < point_cloud->num_points_received; i++) {
        provizio_radar_point *point = &point_cloud->radar_points[i];
        point->ground_relative_radial_velocity_m_s =
            cos(atan2(point->y_meters, point->x_meters) * -1) * radar_velocity_x_m_s - sin(atan2(point->y_meters, point->x_meters) * -1) * radar_velocity_y_m_s;
    }

    return 0;
}

provizio_radar_ego_motion *provizio_get_ego_motion(
    provizio_radar_api_context *context, provizio_radar_ego_motion_packet *packet)
{
    provizio_radar_ego_motion *result = NULL;
    const uint32_t small_frame_index_cap = 0x0000ffff;
    const uint32_t large_frame_index_threashold = 0xffff0000;
    const uint16_t radar_position_id = provizio_get_protocol_field_uint16_t(&packet->radar_position_id);
    const uint32_t frame_index = provizio_get_protocol_field_uint32_t(&packet->frame_index);

    if (frame_index < small_frame_index_cap && context->motion.frame_index > large_frame_index_threashold)
    {
        // A very special case: frame indices seem to have exceeded the 0xffffffff and have been reset. Let's reset the
        // state of the API to avoid complicated state-related issues.
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

    // only update index if newer frame received
    if (context->motion.frame_index < frame_index)
    {
        context->motion.frame_index = frame_index;
    }

    result = &context->motion;

    // save motion data
    result->frame_index = frame_index;
    result->timestamp = provizio_get_protocol_field_uint64_t(&packet->timestamp);
    result->radar_position_id = provizio_get_protocol_field_uint16_t(&packet->radar_position_id);
    result->radar_velocity_x_m_s = provizio_get_protocol_field_float(&packet->radar_velocity_x_m_s);
    result->radar_velocity_y_m_s = provizio_get_protocol_field_float(&packet->radar_velocity_y_m_s);

    return result;
}


int32_t provizio_check_radar_ego_motion_packet(provizio_radar_ego_motion_packet *packet, size_t packet_size)
{
    if (packet_size < sizeof(provizio_radar_api_protocol_header))
    {
        provizio_error("provizio_check_radar_ego_motion_packet: insufficient packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_EGO_MOTION_PACKET_TYPE)
    {
        provizio_error("provizio_check_radar_ego_motion_packet: unexpected packet_type");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_EGO_MOTION_PROTOCOL_VERSION)
    {
        provizio_error("provizio_check_radar_ego_motion_packet: incompatible protocol version");
        return PROVIZIO_E_PROTOCOL;
    }

    if (packet_size < sizeof(provizio_radar_ego_motion_packet))
    {
        provizio_error("provizio_check_radar_ego_motion_packet: insufficient packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->radar_position_id) == provizio_radar_position_unknown)
    {
        provizio_error("provizio_check_radar_ego_motion_packet: the value of radar_position_id can't be "
                       "provizio_radar_position_unknown");
        return PROVIZIO_E_PROTOCOL;
    }

    return 0;
}

int32_t provizio_handle_radar_ego_motion_packet_checked(provizio_radar_api_context *context,
                                                        provizio_radar_ego_motion_packet *packet)
{
    provizio_radar_ego_motion *motion = provizio_get_ego_motion(context, packet);
    if (!motion)
    {
        // Skip the packet (warning has already been published in get_ego_motion)
        return PROVIZIO_E_SKIPPED;
    }

    context->ego_motion_callback(motion, context);

    return 0;
}

int32_t provizio_handle_radar_ego_motion_packet(provizio_radar_api_context *context,
                                                provizio_radar_ego_motion_packet *packet,
                                                size_t packet_size)
{
    const int32_t check_status = provizio_check_radar_ego_motion_packet(packet, packet_size);
    if (check_status != 0)
    {
        return check_status;
    }

    return provizio_handle_radar_ego_motion_packet_checked(context, packet);
}


int32_t provizio_handle_radars_ego_motion_packet(provizio_radar_api_context *contexts,
                                                 size_t num_contexts,
                                                 provizio_radar_ego_motion_packet *packet,
                                                 size_t packet_size)
{
    const int32_t check_status = provizio_check_radar_ego_motion_packet(packet, packet_size);
    if (check_status != 0)
    {
        return check_status;
    }

    const uint16_t radar_position_id = provizio_get_protocol_field_uint16_t(&packet->radar_position_id);

    provizio_radar_api_context *context =
        provizio_get_radar_api_context_by_position_id(contexts, num_contexts, radar_position_id);

    if (!context)
    {
        // Error message has been already posted by provizio_get_radar_api_context_by_position_id
        return PROVIZIO_E_OUT_OF_CONTEXTS;
    }

    return provizio_handle_radar_ego_motion_packet_checked(context, packet);
}

int32_t provizio_handle_possible_radar_ego_motion_packet(provizio_radar_api_context *context,
                                                         const void *payload, size_t payload_size)
{
    return provizio_handle_possible_radars_ego_motion_packet(context, 1, payload, payload_size);
}

int32_t provizio_handle_possible_radars_ego_motion_packet(provizio_radar_api_context *contexts,
                                                          size_t num_contexts, const void *payload,
                                                          size_t payload_size)
{
    if (payload_size < sizeof(provizio_radar_ego_motion_packet))
    {
        // Not enough data
        return PROVIZIO_E_SKIPPED;
    }

    provizio_radar_ego_motion_packet *packet = (provizio_radar_ego_motion_packet *)payload;
    if (provizio_get_protocol_field_uint16_t(&packet->protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_EGO_MOTION_PACKET_TYPE)
    {
        // Non-point cloud packet
        return PROVIZIO_E_SKIPPED;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_EGO_MOTION_PROTOCOL_VERSION)
    {
        provizio_error("provizio_handle_possible_radar_ego_motion_packet: incompatible protocol version");
        return PROVIZIO_E_PROTOCOL;
    }

    return num_contexts != 1
               ? provizio_handle_radars_ego_motion_packet(contexts, num_contexts,
                                                          (provizio_radar_ego_motion_packet *)payload, payload_size)
               : provizio_handle_radar_ego_motion_packet(contexts, (provizio_radar_ego_motion_packet *)payload,
                                                         payload_size);
}
