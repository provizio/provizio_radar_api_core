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

#ifndef INCLUDE_TEST_ENTITIES_HELPERS
#define INCLUDE_TEST_ENTITIES_HELPERS

#include <math.h>
#include <string.h>

#include "provizio/radar_api/entities.h"
#include "provizio/radar_api/radar_ranges.h"
#include "provizio/util.h"

static inline float provizio_test_next_value(float current, float min, float max, float step)
{
    const float range = max - min;
    if (range <= 0.0F)
    {
        return min;
    }

    float next = current + step;
    while (next >= max)
    {
        next = min + fmodf(next - min, range);
    }
    return next;
}

static inline int32_t provizio_test_create_entities_packet(provizio_radar_entities_packet *packet, uint32_t frame_index,
                                                           uint64_t timestamp, uint16_t radar_position_id,
                                                           uint16_t radar_range, uint16_t total_entities_in_frame,
                                                           uint16_t num_entities_in_packet, float x_offset)
{
    const float x_meters_min = -50.0F;
    const float x_meters_max = 50.0F;
    const float x_meters_step = 3.5F;
    const float y_meters_min = -20.0F;
    const float y_meters_max = 20.0F;
    const float y_meters_step = 1.3F;
    const float z_meters_min = -5.0F;
    const float z_meters_max = 15.0F;
    const float z_meters_step = 0.73F;
    const float radar_velocity_min = -15.0F;
    const float radar_velocity_max = 25.0F;
    const float radar_velocity_step = 4.0F;
    const float ground_velocity_min = -10.0F;
    const float ground_velocity_max = 12.0F;
    const float ground_velocity_step = 2.25F;
    const float orientation_min = -1.0F;
    const float orientation_max = 1.0F;
    const float orientation_step = 0.19F;
    const float half = 0.5F;

    float x_meters = (x_meters_min + x_meters_max) * half + x_offset;
    float y_meters = (y_meters_min + y_meters_max) * half;
    float z_meters = (z_meters_min + z_meters_max) * half;
    float radar_velocity = (radar_velocity_min + radar_velocity_max) * half;
    float ground_velocity = (ground_velocity_min + ground_velocity_max) * half;
    float orientation_w = (orientation_min + orientation_max) * half;
    float orientation_x = orientation_w * half;
    float orientation_y = -orientation_w * half;
    float orientation_z = orientation_w;

    memset(packet, 0, sizeof(provizio_radar_entities_packet));

    provizio_set_protocol_field_uint16_t(&packet->header.protocol_header.packet_type,
                                         PROVIZIO__RADAR_API_ENTITIES_PACKET_TYPE);
    provizio_set_protocol_field_uint16_t(&packet->header.protocol_header.protocol_version,
                                         PROVIZIO__RADAR_API_ENTITY_PROTOCOL_VERSION);
    provizio_set_protocol_field_uint32_t(&packet->header.frame_index, frame_index);
    provizio_set_protocol_field_uint64_t(&packet->header.timestamp, timestamp);
    provizio_set_protocol_field_uint16_t(&packet->header.radar_position_id, radar_position_id);
    provizio_set_protocol_field_uint16_t(&packet->header.total_entities_in_frame, total_entities_in_frame);
    provizio_set_protocol_field_uint16_t(&packet->header.num_entities_in_packet, num_entities_in_packet);
    provizio_set_protocol_field_uint16_t(&packet->header.radar_range, radar_range == provizio_radar_range_unknown
                                                                          ? provizio_radar_range_medium
                                                                          : radar_range);

    for (uint16_t j = 0; j < num_entities_in_packet; ++j)
    {
        provizio_radar_entity *entity = &packet->radar_entities[j];

        x_meters = provizio_test_next_value(x_meters, x_meters_min + x_offset, x_meters_max + x_offset, x_meters_step);
        y_meters = provizio_test_next_value(y_meters, y_meters_min, y_meters_max, y_meters_step);
        z_meters = provizio_test_next_value(z_meters, z_meters_min, z_meters_max, z_meters_step);
        radar_velocity =
            provizio_test_next_value(radar_velocity, radar_velocity_min, radar_velocity_max, radar_velocity_step);
        ground_velocity =
            provizio_test_next_value(ground_velocity, ground_velocity_min, ground_velocity_max, ground_velocity_step);
        orientation_w = provizio_test_next_value(orientation_w, orientation_min, orientation_max, orientation_step);
        orientation_x = provizio_test_next_value(orientation_x, orientation_min, orientation_max, orientation_step);
        orientation_y = provizio_test_next_value(orientation_y, orientation_min, orientation_max, orientation_step);
        orientation_z = provizio_test_next_value(orientation_z, orientation_min, orientation_max, orientation_step);

        provizio_set_protocol_field_uint32_t(&entity->entity_id, (uint32_t)j + 1U);
        provizio_set_protocol_field_float(&entity->x_meters, x_meters);
        provizio_set_protocol_field_float(&entity->y_meters, y_meters);
        provizio_set_protocol_field_float(&entity->z_meters, z_meters);
        provizio_set_protocol_field_float(&entity->radar_relative_radial_velocity_m_s, radar_velocity);
        provizio_set_protocol_field_float(&entity->ground_relative_radial_velocity_m_s, ground_velocity);
        provizio_set_protocol_field_float(&entity->orientation.w, orientation_w);
        provizio_set_protocol_field_float(&entity->orientation.x, orientation_x);
        provizio_set_protocol_field_float(&entity->orientation.y, orientation_y);
        provizio_set_protocol_field_float(&entity->orientation.z, orientation_z);
        provizio_set_protocol_field_uint8_t(&entity->entity_class,
                                            (uint8_t)((j % (provizio_entity_class_obstacle + 1)) + 1));
        provizio_set_protocol_field_uint8_t(&entity->entity_confidence, (uint8_t)(50 + j));
        provizio_set_protocol_field_uint8_t(&entity->entity_class_confidence, (uint8_t)(60 + j));
        provizio_set_protocol_field_uint8_t(&entity->reserved, 0);
    }

    return 0;
}

#endif // INCLUDE_TEST_ENTITIES_HELPERS
