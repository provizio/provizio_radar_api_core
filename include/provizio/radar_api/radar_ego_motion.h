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

#ifndef PROVIZIO_RADAR_API_RADAR_EGO_MOTION
#define PROVIZIO_RADAR_API_RADAR_EGO_MOTION

#include "provizio/common.h"
#include "provizio/socket.h"

// Used to identify provizio_radar_ego_motion_packet, can't change even on protocol updates
#define PROVIZIO__RADAR_API_EGO_MOTION_PACKET_TYPE ((uint16_t)2)

// To be incremented on any protocol changes (used for backward compatibility)
#define PROVIZIO__RADAR_API_EGO_MOTION_PROTOCOL_VERSION ((uint16_t)1)

// Use packed structs intended to be sent for binary compatibility across all CPUs
#pragma pack(push, 1)

/**
 * @brief 4-bytes header prefix used to identify a packet type and the protocol version
 *
 * @note This struct should never change, even on protocol updates
 * @see provizio_set_protocol_field_uint16_t
 * @see provizio_get_protocol_field_uint16_t
 */
typedef struct provizio_radar_ego_motion_packet_protocol_header
{
    uint16_t packet_type;      // = PROVIZIO__RADAR_API_EGO_MOTION_PACKET_TYPE
    uint16_t protocol_version; // = PROVIZIO__RADAR_API_EGO_MOTION_PROTOCOL_VERSION (in the "current" protocol version)
} provizio_radar_ego_motion_packet_protocol_header;

/**
 * @brief Structure to hold EGO motion calculations (up to one per each radar frame).
 *
 * @note All fields are sent using network bytes order.
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when accessing accessing fields on non-x86/x64 systems.
 * @see provizio_set_protocol_field_uint32_t
 * @see provizio_get_protocol_field_uint32_t
 * @see provizio_set_protocol_field_uint64_t
 * @see provizio_get_protocol_field_uint64_t
 * @see provizio_set_protocol_field_float
 * @see provizio_get_protocol_field_float
 */
typedef struct provizio_radar_ego_motion_packet
{
    provizio_radar_ego_motion_packet_protocol_header protocol_header;

    uint32_t frame_index;   // 0-based
    uint64_t timestamp;     // Time of the frame capture measured in absolute number of nanoseconds since the start of the
                            // GPS Epoch (midnight on Jan 6, 1980)
    float vs_x;             // sensor ego motion x
    float vs_y;             // sensor ego motion y
    float vs_xy_mag;        // sensor ego velocity magnitude
    // need radar direction or radar mount angle to determine vs_alpha
    float vs_alpha;
    // need radar mount angle, car axle to sensor x, and car axle to sensor y for these:
    float car_velocity;
    float car_yaw_rate;
    float car_turn_radius;
    uint32_t reserved;      // Not used currently, kept for better alignment and potential future use
} provizio_radar_ego_motion_packet;


// Reset alignment settings
#pragma pack(pop)


/**
 * @brief EGO motion information for a particular radar frame
 */
typedef struct provizio_radar_ego_motion
{
    uint32_t frame_index;   // 0-based
    uint64_t timestamp;     // Time of the frame capture measured in absolute number of nanoseconds since the start of the
                            // GPS Epoch (midnight on Jan 6, 1980)
    float vs_x;             // sensor ego motion x
    float vs_y;             // sensor ego motion y
    float vs_xy_mag;        // sensor ego velocity magnitude
    // need radar direction or radar mount angle to determine vs_alpha
    float vs_alpha;
    // need radar mount angle, car axle to sensor x, and car axle to sensor y for car motion
    float car_velocity;
    float car_yaw_rate;
    float car_turn_radius;
} provizio_radar_ego_motion;


struct provizio_radar_ego_motion_api_context;
typedef void (*provizio_radar_ego_motion_callback)(const provizio_radar_ego_motion *ego_motion,
                                                   struct provizio_radar_ego_motion_api_context *context);

/**
 * @brief Keeps all data required for functioning of radar ego motion API
 */
typedef struct provizio_radar_ego_motion_api_context
{
    provizio_radar_ego_motion_callback callback;
    void *user_data;
    uint16_t radar_position_id;
} provizio_radar_ego_motion_api_context;

/**
 * @brief Handles a single radar ego motion UDP packet from a single radar
 *
 * @param context Previously initialized provizio_radar_ego_motion_api_context
 * @param packet Valid provizio_radar_ego_motion_packet
 * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
 * other error code in case of another error
 *
 * @warning radar_position_id of all packets handled by this context must be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_radar_ego_motion_packet(provizio_radar_ego_motion_api_context *context,
                                                                   provizio_radar_ego_motion_packet *packet);

/**
 * @brief Handles a single radar ego motion UDP packet from one of multiple radars
 *
 * @param contexts Previously initialized array of num_contexts of provizio_radar_ego_motion_api_context objects
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
 * @param packet Valid provizio_radar_ego_motion_packet
 * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
 * PROVIZIO_E_OUT_OF_CONTEXTS in case num_contexts is not enough, other error code in case of another error
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_radars_ego_motion_packet(provizio_radar_ego_motion_api_context *contexts,
                                                                    size_t num_contexts,
                                                                    provizio_radar_ego_motion_packet *packet);


#endif // PROVIZIO_RADAR_API_RADAR_EGO_MOTION

