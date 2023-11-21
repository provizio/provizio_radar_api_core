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

#ifndef PROVIZIO_RADAR_API_RADAR_MODES
#define PROVIZIO_RADAR_API_RADAR_MODES

#include "provizio/radar_api/common.h"
#include "provizio/radar_api/radar_position.h"

// To be incremented on any breaking protocol changes (used for backward compatibility)
#define PROVIZIO__RADAR_API_MODE_PROTOCOL_VERSION ((uint16_t)1)

// Use packed structs intended to be sent for binary compatibility across all CPUs
#pragma pack(push, 1)

typedef enum provizio_radar_mode
{
    provizio_radar_mode_short_range = 0,
    provizio_radar_mode_medium_range = 1,
    provizio_radar_mode_long_range = 2,
    provizio_radar_mode_ultra_long_range = 3,
    provizio_radar_mode_hyper_long_range = 4,

    provizio_radar_mode_unknown = ((uint16_t)0xffff)
} provizio_radar_mode;

/**
 * @brief Packet structure for a radar mode change request
 *
 * @note All fields are sent using network bytes order.
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when
 * accessing fields on non-x86/x64 systems.
 * @see provizio_set_protocol_field_uint16_t
 * @see provizio_get_protocol_field_uint16_t
 */
typedef struct provizio_set_radar_mode_packet
{
    provizio_radar_api_protocol_header protocol_header;

    uint16_t radar_position_id; // Either one of provizio_radar_position enum values or a custom position id
    uint16_t radar_mode;        // One of provizio_radar_mode enum values
} provizio_set_radar_mode_packet;

/**
 * @brief Packet structure for a radar mode change acknowledgement
 *
 * @note All fields are sent using network bytes order.
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when
 * accessing fields on non-x86/x64 systems.
 * @see provizio_set_protocol_field_uint16_t
 * @see provizio_get_protocol_field_uint16_t
 * @see provizio_set_protocol_field_uint32_t
 * @see provizio_get_protocol_field_uint32_t
 */
typedef struct provizio_set_radar_mode_acknowledgement_packet
{
    provizio_radar_api_protocol_header protocol_header;

    uint16_t radar_position_id;    // Either one of provizio_radar_position enum values or a custom position id
    uint16_t requested_radar_mode; // One of provizio_radar_mode enum values
    int32_t error_code;            // 0 for success, PROVIZIO_E_NOT_PERMITTED if the mode is not supported
} provizio_set_radar_mode_acknowledgement_packet;

// Reset alignment settings
#pragma pack(pop)

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(offsetof(provizio_set_radar_mode_packet, protocol_header) == 0,
              "Unexpected position of protocol_header in provizio_set_radar_mode_packet");
static_assert(offsetof(provizio_set_radar_mode_packet, radar_position_id) == 4,
              "Unexpected position of radar_position_id in provizio_set_radar_mode_packet");
static_assert(offsetof(provizio_set_radar_mode_packet, radar_mode) == 6,
              "Unexpected position of radar_mode in provizio_set_radar_mode_packet");
static_assert(sizeof(provizio_set_radar_mode_packet) == 8, "Unexpected size of provizio_set_radar_mode_packet");
static_assert(offsetof(provizio_set_radar_mode_acknowledgement_packet, protocol_header) == 0,
              "Unexpected position of protocol_header in provizio_set_radar_mode_acknowledgement_packet");
static_assert(offsetof(provizio_set_radar_mode_acknowledgement_packet, radar_position_id) == 4,
              "Unexpected position of radar_position_id in provizio_set_radar_mode_acknowledgement_packet");
static_assert(offsetof(provizio_set_radar_mode_acknowledgement_packet, requested_radar_mode) == 6,
              "Unexpected position of requested_radar_mode in provizio_set_radar_mode_acknowledgement_packet");
static_assert(offsetof(provizio_set_radar_mode_acknowledgement_packet, error_code) == 8,
              "Unexpected position of error_code in provizio_set_radar_mode_acknowledgement_packet");
static_assert(sizeof(provizio_set_radar_mode_acknowledgement_packet) == 12,
              "Unexpected size of provizio_set_radar_mode_acknowledgement_packet");
#endif // defined(__cplusplus) && __cplusplus >= 201103L

#endif // PROVIZIO_RADAR_API_RADAR_MODES
