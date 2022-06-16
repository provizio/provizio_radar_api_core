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

#ifndef PROVIZIO_RADAR_API_COMMON
#define PROVIZIO_RADAR_API_COMMON

#include "provizio/common.h"

// Values are used to identify packet types, can't change even on protocol updates
#define PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE ((uint16_t)1)
#define PROVIZIO__RADAR_API_SET_MODE_PACKET_TYPE ((uint16_t)2)

// Use packed structs intended to be sent for binary compatibility across all CPUs
#pragma pack(push, 1)

/**
 * @brief 4-bytes header prefix used to identify a packet type and the protocol version
 *
 * @note This struct should never change, even on protocol updates
 * @see provizio_set_protocol_field_uint16_t
 * @see provizio_get_protocol_field_uint16_t
 */
typedef struct provizio_radar_api_protocol_header
{
    uint16_t packet_type;      // Identifies a protocol packet, one of PROVIZIO__*_PACKET_TYPE
    uint16_t protocol_version; // Packet type - specific protocol version, used for backward compatibility
} provizio_radar_api_protocol_header;

// Reset alignment settings
#pragma pack(pop)

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(offsetof(provizio_radar_api_protocol_header, packet_type) == 0,
              "Unexpected position of protocol_header in provizio_radar_api_protocol_header");
static_assert(offsetof(provizio_radar_api_protocol_header, protocol_version) == 2,
              "Unexpected position of frame_index in provizio_radar_api_protocol_header");
static_assert(sizeof(provizio_radar_api_protocol_header) == 4, "Unexpected size of provizio_radar_api_protocol_header");
#endif // defined(__cplusplus) && __cplusplus >= 201103L

#endif // PROVIZIO_RADAR_API_COMMON
