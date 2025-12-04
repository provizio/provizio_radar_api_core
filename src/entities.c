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

#include "provizio/radar_api/entities.h"

#include <assert.h>
#include <string.h>

#include "provizio/radar_api/errno.h"
#include "provizio/util.h"

void provizio_return_entities_frame(provizio_radar_entities_api_context *context, provizio_radar_entities_frame *frame)
{
    // Make sure all older but incomplete frames are returned first
#pragma unroll
    for (size_t i = 0; i < PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT; ++i)
    {
        provizio_radar_entities_frame *other_frame = &context->impl.entities_frames_being_received[i];
        if (other_frame != frame && other_frame->num_entities_expected > 0 &&
            other_frame->frame_index < frame->frame_index)
        {
            provizio_return_entities_frame(context, other_frame);
        }
    }

    context->callback(frame, context);
    memset(frame, 0, sizeof(provizio_radar_entities_frame));
}

provizio_radar_entities_frame *provizio_get_entities_frame_being_received(
    provizio_radar_entities_api_context *context, provizio_radar_entities_packet_header *packet_header)
{
    const uint32_t small_frame_index_cap = 0x0000ffff;
    const uint32_t large_frame_index_threshold = 0xffff0000;

    provizio_radar_entities_frame *frame = NULL;

    const uint16_t radar_position_id = provizio_get_protocol_field_uint16_t(&packet_header->radar_position_id);
    const uint32_t frame_index = provizio_get_protocol_field_uint32_t(&packet_header->frame_index);
    const uint16_t total_entities_in_frame =
        provizio_get_protocol_field_uint16_t(&packet_header->total_entities_in_frame);
    const uint16_t radar_range = provizio_get_protocol_field_uint16_t(&packet_header->radar_range);

    if (frame_index < small_frame_index_cap && context->impl.latest_frame > large_frame_index_threshold)
    {
        provizio_warning("provizio_get_entities_frame_being_received: frame indices overflow detected - resetting API "
                         "state");
        provizio_radar_entities_api_context_init(context->callback, context->user_data, context);
    }

    if (context->radar_position_id == provizio_radar_position_unknown)
    {
        provizio_radar_entities_api_context_assign(context, radar_position_id);
    }
    else if (context->radar_position_id != radar_position_id)
    {
        return NULL;
    }

    if (context->impl.latest_frame < frame_index)
    {
        context->impl.latest_frame = frame_index;
    }

    // Look for a frame already being received
#pragma unroll
    for (size_t i = 0; i < PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT; ++i)
    {
        frame = &context->impl.entities_frames_being_received[i];
        if (frame->frame_index == frame_index && frame->num_entities_expected > 0)
        {
            if (frame->num_entities_expected != total_entities_in_frame)
            {
                provizio_warning("provizio_get_entities_frame_being_received: num_entities_expected mismatch across "
                                 "different packets of the same frame");
            }

            if (frame->radar_range != radar_range)
            {
                provizio_warning("provizio_get_entities_frame_being_received: radar_range mismatch across different "
                                 "packets of the same frame");
            }

            return frame;
        }
    }

    provizio_radar_entities_frame *result = NULL;

    // Look for an empty frame slot
#pragma unroll
    for (size_t i = 0; i < PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT; ++i)
    {
        frame = &context->impl.entities_frames_being_received[i];
        if (frame->num_entities_expected == 0)
        {
            result = frame;
        }
    }

    if (!result)
    {
        // Drop (return incomplete) the oldest incomplete frame unless it's newer than the incoming packet
#pragma unroll
        for (size_t i = 0; i < PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT; ++i)
        {
            frame = &context->impl.entities_frames_being_received[i];
            if (frame->frame_index < frame_index && (!result || frame->frame_index < result->frame_index))
            {
                result = frame;
            }
        }

        if (result != NULL)
        {
            provizio_return_entities_frame(context, result);
        }
    }

    if (result != NULL)
    {
        result->frame_index = frame_index;
        result->timestamp = provizio_get_protocol_field_uint64_t(&packet_header->timestamp);
        result->radar_position_id = radar_position_id;
        result->num_entities_expected = total_entities_in_frame;
        result->radar_range = radar_range;
        assert(result->num_entities_received == 0);
    }

    return result;
}

size_t provizio_radar_entities_packet_size(const provizio_radar_entities_packet_header *header)
{
    size_t result = 0;
    const uint16_t num_entities = provizio_get_protocol_field_uint16_t(&header->num_entities_in_packet);

    if (num_entities > PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET)
    {
        provizio_warning("provizio_radar_entities_packet_size: num_entities_in_packet exceeds "
                         "PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET!");
    }
    else
    {
        result = sizeof(provizio_radar_entities_packet_header) + (sizeof(provizio_radar_entity) * num_entities);
    }

    return result;
}

void provizio_radar_entities_api_context_init(provizio_radar_entities_callback callback, void *user_data,
                                              provizio_radar_entities_api_context *context)
{
    memset(context, 0, sizeof(provizio_radar_entities_api_context));

    context->callback = callback;
    context->user_data = user_data;
    context->radar_position_id = provizio_radar_position_unknown;
}

void provizio_radar_entities_api_contexts_init(provizio_radar_entities_callback callback, void *user_data,
                                               provizio_radar_entities_api_context *contexts, size_t num_contexts)
{
#pragma unroll(5)
    for (size_t i = 0; i < num_contexts; ++i)
    {
        provizio_radar_entities_api_context_init(callback, user_data, &contexts[i]);
    }
}

int32_t provizio_radar_entities_api_context_assign(provizio_radar_entities_api_context *context,
                                                   provizio_radar_position radar_position_id)
{
    if (radar_position_id == provizio_radar_position_unknown)
    {
        provizio_error("provizio_radar_entities_api_context_assign: can't assign to provizio_radar_position_unknown");
        return PROVIZIO_E_ARGUMENT;
    }

    if (context->radar_position_id == radar_position_id)
    {
        return 0;
    }

    if (context->radar_position_id == provizio_radar_position_unknown)
    {
        context->radar_position_id = radar_position_id;
        return 0;
    }

    provizio_error("provizio_radar_entities_api_context_assign: already assigned");
    return PROVIZIO_E_NOT_SUPPORTED;
}

int32_t provizio_check_radar_entities_packet(provizio_radar_entities_packet *packet, size_t packet_size)
{
    if (packet_size < sizeof(provizio_radar_api_protocol_header))
    {
        provizio_error("provizio_check_radar_entities_packet: insufficient packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_ENTITIES_PACKET_TYPE)
    {
        provizio_error("provizio_check_radar_entities_packet: unexpected packet_type");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_ENTITY_PROTOCOL_VERSION)
    {
        provizio_error("provizio_check_radar_entities_packet: Incompatible protocol version");
        return PROVIZIO_E_PROTOCOL;
    }

    if (packet_size < sizeof(provizio_radar_entities_packet_header))
    {
        provizio_error("provizio_check_radar_entities_packet: insufficient packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    const size_t expected_packet_size = provizio_radar_entities_packet_size(&packet->header);
    if (expected_packet_size == 0)
    {
        provizio_error("provizio_check_radar_entities_packet: incorrect num_entities_in_packet");
        return PROVIZIO_E_PROTOCOL;
    }

    if (packet_size != expected_packet_size)
    {
        provizio_error("provizio_check_radar_entities_packet: incorrect packet_size");
        return PROVIZIO_E_PROTOCOL;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.radar_position_id) == provizio_radar_position_unknown)
    {
        provizio_error("provizio_check_radar_entities_packet: the value of radar_position_id can't be "
                       "provizio_radar_position_unknown");
        return PROVIZIO_E_PROTOCOL;
    }

    const uint16_t total_entities_in_frame =
        provizio_get_protocol_field_uint16_t(&packet->header.total_entities_in_frame);
    if (total_entities_in_frame > PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME)
    {
        provizio_error("provizio_check_radar_entities_packet: total_entities_in_frame exceeds "
                       "PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME");
        return PROVIZIO_E_PROTOCOL;
    }

    const uint16_t num_entities_in_packet =
        provizio_get_protocol_field_uint16_t(&packet->header.num_entities_in_packet);
    if (num_entities_in_packet > total_entities_in_frame)
    {
        provizio_error("provizio_check_radar_entities_packet: num_entities_in_packet exceeds total_entities_in_frame");
        return PROVIZIO_E_PROTOCOL;
    }

    return 0;
}

int32_t provizio_check_for_too_many_entities(provizio_radar_entities_frame *frame,
                                             const uint16_t num_entities_in_packet)
{
    if ((uint32_t)frame->num_entities_received + (uint32_t)num_entities_in_packet >
        (uint32_t)frame->num_entities_expected)
    {
        provizio_error("provizio_check_for_too_many_entities: Too many entities received");
        return PROVIZIO_E_PROTOCOL;
    }

    provizio_verbose("provizio_check_for_too_many_entities: OK");
    return 0;
}

int32_t provizio_handle_entities_packet_checked(provizio_radar_entities_api_context *context,
                                                provizio_radar_entities_packet *packet)
{
    provizio_radar_entities_frame *frame = provizio_get_entities_frame_being_received(context, &packet->header);
    if (!frame)
    {
        return PROVIZIO_E_SKIPPED;
    }

    if (provizio_get_protocol_field_uint16_t(&packet->header.total_entities_in_frame) == 0)
    {
        provizio_verbose("provizio_handle_entities_packet_checked: Empty frame");
        return PROVIZIO_E_SKIPPED;
    }

    const uint16_t num_entities_in_packet =
        provizio_get_protocol_field_uint16_t(&packet->header.num_entities_in_packet);

    const int32_t status = provizio_check_for_too_many_entities(frame, num_entities_in_packet);
    if (status != 0)
    {
        return status;
    }

    provizio_radar_entity *output_to = &frame->radar_entities[frame->num_entities_received];

    for (uint16_t i = 0; i < num_entities_in_packet; ++i)
    {
        provizio_radar_entity *out_entity = output_to + i;
        const provizio_radar_entity *in_entity = &packet->radar_entities[i];

        out_entity->entity_id = provizio_get_protocol_field_uint32_t(&in_entity->entity_id);
        out_entity->x_meters = provizio_get_protocol_field_float(&in_entity->x_meters);
        out_entity->y_meters = provizio_get_protocol_field_float(&in_entity->y_meters);
        out_entity->z_meters = provizio_get_protocol_field_float(&in_entity->z_meters);
        out_entity->radar_relative_radial_velocity_m_s =
            provizio_get_protocol_field_float(&in_entity->radar_relative_radial_velocity_m_s);
        out_entity->ground_relative_radial_velocity_m_s =
            provizio_get_protocol_field_float(&in_entity->ground_relative_radial_velocity_m_s);
        out_entity->orientation.w = provizio_get_protocol_field_float(&in_entity->orientation.w);
        out_entity->orientation.x = provizio_get_protocol_field_float(&in_entity->orientation.x);
        out_entity->orientation.y = provizio_get_protocol_field_float(&in_entity->orientation.y);
        out_entity->orientation.z = provizio_get_protocol_field_float(&in_entity->orientation.z);
        out_entity->entity_class = provizio_get_protocol_field_uint8_t(&in_entity->entity_class);
        out_entity->entity_confidence = provizio_get_protocol_field_uint8_t(&in_entity->entity_confidence);
        out_entity->entity_class_confidence = provizio_get_protocol_field_uint8_t(&in_entity->entity_class_confidence);
        out_entity->reserved = provizio_get_protocol_field_uint8_t(&in_entity->reserved);
    }

    frame->num_entities_received += num_entities_in_packet;

    if (frame->num_entities_received == frame->num_entities_expected)
    {
        provizio_verbose("provizio_handle_entities_packet_checked: Return complete entities frame");
        provizio_return_entities_frame(context, frame);
    }
    else
    {
        provizio_verbose("provizio_handle_entities_packet_checked: Success, but incomplete yet");
    }

    return 0;
}

int32_t provizio_handle_entities_packet(provizio_radar_entities_api_context *context,
                                        provizio_radar_entities_packet *packet, size_t packet_size)
{
    const int32_t check_status = provizio_check_radar_entities_packet(packet, packet_size);
    if (check_status != 0)
    {
        provizio_verbose("provizio_handle_entities_packet: Packet check failed");
        return check_status;
    }

    return provizio_handle_entities_packet_checked(context, packet);
}

provizio_radar_entities_api_context *provizio_get_radar_entities_api_context_by_position_id(
    provizio_radar_entities_api_context *contexts, size_t num_contexts, provizio_radar_entities_packet *packet)
{
    const uint16_t radar_position_id = provizio_get_protocol_field_uint16_t(&packet->header.radar_position_id);
    assert(radar_position_id != provizio_radar_position_unknown);

#pragma unroll(5)
    for (size_t i = 0; i < num_contexts; ++i)
    {
        if (contexts[i].radar_position_id == radar_position_id)
        {
            return &contexts[i];
        }
    }

#pragma unroll(5)
    for (size_t i = 0; i < num_contexts; ++i)
    {
        if (contexts[i].radar_position_id == provizio_radar_position_unknown)
        {
            return &contexts[i];
        }
    }

    provizio_error("provizio_get_radar_entities_api_context_by_position_id: Out of available contexts");
    return NULL;
}

int32_t provizio_handle_radars_entities_packet(provizio_radar_entities_api_context *contexts, size_t num_contexts,
                                               provizio_radar_entities_packet *packet, size_t packet_size)
{
    const int32_t check_status = provizio_check_radar_entities_packet(packet, packet_size);
    if (check_status != 0)
    {
        provizio_verbose("provizio_handle_radars_entities_packet: Packet check failed");
        return check_status;
    }

    provizio_radar_entities_api_context *context =
        provizio_get_radar_entities_api_context_by_position_id(contexts, num_contexts, packet);

    if (!context)
    {
        return PROVIZIO_E_OUT_OF_CONTEXTS;
    }

    return provizio_handle_entities_packet_checked(context, packet);
}

int32_t provizio_handle_possible_radar_entities_packet(provizio_radar_entities_api_context *context,
                                                       const void *payload, size_t payload_size)
{
    return provizio_handle_possible_radars_entities_packet(context, 1, payload, payload_size);
}

int32_t provizio_handle_possible_radars_entities_packet(provizio_radar_entities_api_context *contexts,
                                                        size_t num_contexts, const void *payload, size_t payload_size)
{
    if (payload_size < sizeof(provizio_radar_entities_packet_header))
    {
        provizio_verbose("provizio_handle_possible_radars_entities_packet: Not enough data");
        return PROVIZIO_E_SKIPPED;
    }

    provizio_radar_entities_packet_header *packet_header = (provizio_radar_entities_packet_header *)payload;
    if (provizio_get_protocol_field_uint16_t(&packet_header->protocol_header.packet_type) !=
        PROVIZIO__RADAR_API_ENTITIES_PACKET_TYPE)
    {
        provizio_verbose("provizio_handle_possible_radars_entities_packet: Non-entities packet");
        return PROVIZIO_E_SKIPPED;
    }

    if (provizio_get_protocol_field_uint16_t(&packet_header->protocol_header.protocol_version) >
        PROVIZIO__RADAR_API_ENTITY_PROTOCOL_VERSION)
    {
        provizio_error("provizio_handle_possible_radars_entities_packet: Incompatible protocol version");
        return PROVIZIO_E_PROTOCOL;
    }

    return num_contexts != 1
               ? provizio_handle_radars_entities_packet(contexts, num_contexts,
                                                        (provizio_radar_entities_packet *)payload, payload_size)
               : provizio_handle_entities_packet(contexts, (provizio_radar_entities_packet *)payload, payload_size);
}
