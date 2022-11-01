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

#include "provizio/radar_api/common.h"
#include "provizio/radar_api/radar_position.h"
#include "provizio/socket.h"

// Used to identify provizio_radar_ego_motion_packet, can't change even on protocol updates
#define PROVIZIO__RADAR_API_EGO_MOTION_PACKET_TYPE ((uint16_t)2)

// To be incremented on any protocol changes (used for backward compatibility)
#define PROVIZIO__RADAR_API_EGO_MOTION_PROTOCOL_VERSION ((uint16_t)1)

// Use packed structs intended to be sent for binary compatibility across all CPUs
#pragma pack(push, 1)

// declared in a radar_api_context.h which is not included to avoid circular dependencies
struct provizio_radar_api_context;

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
 * @see provizio_set_protocol_field_uint16_t
 * @see provizio_get_protocol_field_uint16_t
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
    uint16_t radar_position_id;   // Either one of provizio_radar_position enum values or a custom position id
    uint16_t reserved;      // Not used currently, kept for better alignment and potential future use
    float vs_x;             // sensor ego motion x
    float vs_y;             // sensor ego motion y
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
    uint16_t radar_position_id;   // Either one of provizio_radar_position enum values or a custom position id
} provizio_radar_ego_motion;


typedef void (*provizio_radar_ego_motion_callback)(const provizio_radar_ego_motion *ego_motion,
                                                   struct provizio_radar_api_context *context);

#if 0
/**
 * @brief Keeps all data required for functioning of radar ego motion API
 */
typedef struct provizio_radar_ego_motion_api_context
{
    provizio_radar_ego_motion_callback callback;
    void *user_data;
    uint16_t radar_position_id;
    provizio_radar_ego_motion motion;
} provizio_radar_ego_motion_api_context;

/**
 * @brief Initializes a provizio_ego_motion_cloud_api_context object to handle a single radar
 *
 * @param callback Function to be called on receiving a radar ego motion
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param context The provizio_radar_ego_motion_api_context object to initialize
 *
 * @warning radar_position_id of all packets handled by this context must be same
 */
PROVIZIO__EXTERN_C void provizio_radar_ego_motion_api_context_init(provizio_radar_ego_motion_callback callback,
                                                                   void *user_data,
                                                                   provizio_radar_ego_motion_api_context *context);

/**
 * @brief Initializes multiple provizio_radar_ego_motion_api_context objects to handle packets from multiple radars
 *
 * @param callback Function to be called on receiving a radar ego motion
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param contexts Array of num_contexts of provizio_radar_ego_motion_api_context objects to initialize
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle) to initialize
 */
PROVIZIO__EXTERN_C void provizio_radar_ego_motion_api_contexts_init(provizio_radar_ego_motion_callback callback,
                                                                    void *user_data,
                                                                    provizio_radar_ego_motion_api_context *contexts,
                                                                    size_t num_contexts);

/**
 * @brief Makes provizio_radar_ego_motion_api_context object handle a specific radar, which makes it skip packets
 * intended for other radars
 *
 * @param context provizio_radar_ego_motion_api_context to be assigned
 * @param radar_position_id radar to assign
 * @return 0 in case it was successfully assigned, an error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_radar_ego_motion_api_context_assign(
    provizio_radar_ego_motion_api_context *context, provizio_radar_position radar_position_id);
#endif

/**
 * @brief Handles a single radar ego motion UDP packet from a single radar
 *
 * @param context Previously initialized provizio_radar_api_context
 * @param packet Valid provizio_radar_ego_motion_packet
 * @param packet_size The size of the packet, to check data is valid and avoid out-of-bounds access
 * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
 * other error code in case of another error
 *
 * @warning radar_position_id of all packets handled by this context must be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_radar_ego_motion_packet(struct provizio_radar_api_context *context,
                                                                   provizio_radar_ego_motion_packet *packet,
                                                                   size_t packet_size);

/**
 * @brief Handles a single radar ego motion UDP packet from one of multiple radars
 *
 * @param contexts Previously initialized array of num_contexts of provizio_radar_api_context objects
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
 * @param packet Valid provizio_radar_ego_motion_packet
 * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
 * PROVIZIO_E_OUT_OF_CONTEXTS in case num_contexts is not enough, other error code in case of another error
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_radars_ego_motion_packet(struct provizio_radar_api_context *contexts,
                                                                    size_t num_contexts,
                                                                    provizio_radar_ego_motion_packet *packet,
                                                                    size_t packet_size);

/**
 * @brief Handles a single Provizio Radar API UDP packet from a single radar, that can be a correct
 * provizio_radar_ego_motion_packet or something else
 *
 * @param context Previously initialized provizio_radar_api_context
 * @param payload The payload of the UDP packet
 * @param payload_size The size of the payload in bytes
 * @return 0 if it's a provizio_radar_ego_motion_packet and it was handled successfully, PROVIZIO_E_SKIPPED if it's not
 * a provizio_radar_ego_motion_packet, other error code if it's a provizio_radar_ego_motion_packet but its handling
 * failed for another reason
 *
 * @warning if it's a provizio_radar_ego_motion_packet, radar_position_id of all packets handled by this context must
 * be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_possible_radar_ego_motion_packet(
    struct provizio_radar_api_context *context, const void *payload, size_t payload_size);

/**
 * @brief Handles a single Provizio Radar API UDP packet from one of multiple radars, that can be a correct
 * provizio_radar_ego_motion_packet or something else
 *
 * @param contexts Previously initialized array of num_contexts of provizio_radar_api_context objects
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
 * @param payload The payload of the UDP packet
 * @param payload_size The size of the payload in bytes
 * @return 0 if it's a provizio_radar_ego_motion_packet and it was handled successfully, PROVIZIO_E_SKIPPED if it's not
 * a provizio_radar_ego_motion_packet, PROVIZIO_E_OUT_OF_CONTEXTS in case num_contexts is not enough, other error code
 * if it's a provizio_radar_ego_motion_packet but its handling failed for another reason
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_possible_radars_ego_motion_packet(
    struct provizio_radar_api_context *contexts, size_t num_contexts, const void *payload, size_t payload_size);

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(offsetof(provizio_radar_ego_motion_packet, protocol_header) == 0,
              "Unexpected position of protocol_header in provizio_radar_ego_motion_packet");
static_assert(offsetof(provizio_radar_ego_motion_packet, frame_index) == 4,
              "Unexpected position of frame_index in provizio_radar_ego_motion_packet");
static_assert(offsetof(provizio_radar_ego_motion_packet, timestamp) == 8,
              "Unexpected position of timestamp in provizio_radar_ego_motion_packet");
static_assert(offsetof(provizio_radar_ego_motion_packet, radar_position_id) == 16,
              "Unexpected position of radar_position_id in provizio_radar_ego_motion_packet");
static_assert(offsetof(provizio_radar_ego_motion_packet, vs_x) == 20,
              "Unexpected position of vs_x in provizio_radar_ego_motion_packet");
static_assert(offsetof(provizio_radar_ego_motion_packet, vs_y) == 24,
              "Unexpected position of vs_y in provizio_radar_ego_motion_packet");
static_assert(sizeof(provizio_radar_ego_motion_packet) == 28,
              "Unexpected size of provizio_radar_ego_motion_packet");
#endif // defined(__cplusplus) && __cplusplus >= 201103L

#endif // PROVIZIO_RADAR_API_RADAR_EGO_MOTION
