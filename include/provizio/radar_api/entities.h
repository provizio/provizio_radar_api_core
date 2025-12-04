// Copyright 2025 Provizio Ltd.
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

#ifndef PROVIZIO_RADAR_API_ENTITIES
#define PROVIZIO_RADAR_API_ENTITIES

#include "provizio/quaternion.h"
#include "provizio/radar_api/common.h"
#include "provizio/radar_api/radar_position.h"
#include "provizio/radar_api/radar_ranges.h"
#include "provizio/socket.h"

// To be incremented on any breaking protocol changes (used for backward compatibility)
#define PROVIZIO__RADAR_API_ENTITY_PROTOCOL_VERSION ((uint16_t)1)

// Radar entities structures of the binary UDP protocol are defined here, see README.md for more details

typedef enum provizio_entity_class
{
    provizio_entity_class_unknown = 0,
    provizio_entity_class_pedestrian = 1,
    provizio_entity_class_cyclist = 2,
    provizio_entity_class_motorbike = 3,
    provizio_entity_class_car = 4,
    provizio_entity_class_truck = 5,
    provizio_entity_class_bus = 6,
    provizio_entity_class_obstacle = 7
} provizio_entity_class;

// Use packed structs intended to be sent for binary compatibility across all CPUs
#pragma pack(push, 1)

typedef struct
{
    float x_meters; // Forward, radar relative
    float y_meters; // Left, radar relative
    float z_meters; // Up, radar relative
} provizio_size;

/**
 * @brief Represents a single entity.
 *
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when accessing
 * fields on non-x86/x64 systems.
 * @see provizio_set_protocol_field_float
 * @see provizio_get_protocol_field_float
 * @see provizio_get_protocol_field_uint8_t
 * @see provizio_set_protocol_field_uint8_t
 * @see provizio_get_protocol_field_uint32_t
 * @see provizio_set_protocol_field_uint32_t
 */
typedef struct
{
    uint32_t entity_id;                        // Unique identifier of an entity persistent across frames
    float x_meters;                            // Forward, radar relative
    float y_meters;                            // Left, radar relative
    float z_meters;                            // Up, radar relative
    float radar_relative_radial_velocity_m_s;  // Forward, radar relative
    float ground_relative_radial_velocity_m_s; // Ground relative projection to the radar forward axis (NaN if
                                               // unavailable)
    provizio_quaternion orientation;           // Orientation quaternion: (w, x, y, z)
    uint8_t entity_class;                      // One of provizio_entity_class values
    uint8_t entity_confidence;                 // Confidence the entity actually exists, 0..255
    uint8_t entity_class_confidence;           // Confidence the entity class is correct, 0..255
    uint8_t reserved;                          // Unused, reserved for better memory alignment
} provizio_radar_entity;

/**
 * @brief Header placed in the beginning of each radar entities packet.
 *
 * @note All fields are sent using network bytes order.
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when
 * accessing fields on non-x86/x64 systems.
 * @see provizio_set_protocol_field_uint16_t
 * @see provizio_get_protocol_field_uint16_t
 * @see provizio_set_protocol_field_uint32_t
 * @see provizio_get_protocol_field_uint32_t
 * @see provizio_set_protocol_field_uint64_t
 * @see provizio_get_protocol_field_uint64_t
 */
typedef struct provizio_radar_entities_packet_header
{
    provizio_radar_api_protocol_header protocol_header;

    uint32_t frame_index; // 0-based
    uint64_t timestamp; // Time of the latest radar frame capture measured in number of nanoseconds since the Unix Epoch
    uint16_t radar_position_id;       // Either one of provizio_radar_position enum values or a custom position id
    uint16_t total_entities_in_frame; // Number of entities in the entire frame
    uint16_t num_entities_in_packet;  // Number of entities in this single packet
    uint16_t radar_range;             // One of provizio_radar_range enum values, used
} provizio_radar_entities_packet_header;

// Max number of radar entities in a single UDP packet
#define PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET                                                                    \
    ((uint16_t)((PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES - sizeof(provizio_radar_entities_packet_header)) /         \
                sizeof(provizio_radar_entity)))

// Max number of radar entities in a single frame
#define PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME ((uint16_t)0x400)

/**
 * @brief Structure to hold a single radar entities packet (one of some number of such packets per each frame).
 *
 * @note Not all of PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME entities may be present, see
 * header.num_entities_in_packet for the actual number stored in this packet.
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when
 * accessing fields on non-x86/x64 systems.
 */
PROVIZIO__EXTERN_C typedef struct provizio_radar_entities_packet
{
    provizio_radar_entities_packet_header header;
    provizio_radar_entity radar_entities[PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET];
} provizio_radar_entities_packet;

/**
 * @brief Returns the total size (in bytes) of the radar entities packet payload, based on its header.
 *
 * @param header Header of the packet
 * @return size_t Size (in bytes)
 */
PROVIZIO__EXTERN_C size_t provizio_radar_entities_packet_size(const provizio_radar_entities_packet_header *header);

// Reset alignment settings
#pragma pack(pop)

/**
 * @brief A complete or partial radar entities frame
 *
 * @note Complete entities frames always have num_entities_received == num_entities_expected
 */
typedef struct provizio_radar_entities_frame
{
    uint32_t frame_index; // 0-based
    uint64_t timestamp; // Time of the latest radar frame capture measured in number of nanoseconds since the Unix Epoch
    uint16_t radar_position_id;     // Either one of provizio_radar_position enum values or a custom position id
    uint16_t num_entities_expected; // Number of entities in the entire frame
    uint16_t num_entities_received; // Number of entities in the frame received so far
    uint16_t radar_range;           // One of provizio_radar_range enum values
    provizio_radar_entity radar_entities[PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME];
} provizio_radar_entities_frame;

struct provizio_radar_entities_api_context;
typedef void (*provizio_radar_entities_callback)(const provizio_radar_entities_frame *entities_frame,
                                                 struct provizio_radar_entities_api_context *context);

#ifndef PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT
#define PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT 2
#endif // PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT
typedef struct provizio_radar_entities_api_context_impl
{
    uint32_t latest_frame;
    provizio_radar_entities_frame
        entities_frames_being_received[PROVIZIO__RADAR_ENTITIES_API_CONTEXT_IMPL_FRAMES_BEING_RECEIVED_COUNT];
} provizio_radar_entities_api_context_impl;

/**
 * @brief Keeps all data required for functioning of entities API
 */
typedef struct provizio_radar_entities_api_context
{
    provizio_radar_entities_callback callback;
    void *user_data;
    uint16_t radar_position_id;

    provizio_radar_entities_api_context_impl impl;
} provizio_radar_entities_api_context;

/**
 * @brief Initializes a provizio_radar_entities_api_context object to handle a single radar
 *
 * @param callback Function to be called on receiving a complete or partial entities frame
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param context The provizio_radar_entities_api_context object to initialize
 *
 * @note radar_position_id of all packets handled by this context must be same
 */
PROVIZIO__EXTERN_C void provizio_radar_entities_api_context_init(provizio_radar_entities_callback callback,
                                                                 void *user_data,
                                                                 provizio_radar_entities_api_context *context);

/**
 * @brief Initializes multiple provizio_radar_entities_api_context objects to handle packets from multiple radars
 *
 * @param callback Function to be called on receiving a complete or partial entities frame
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param contexts Array of num_contexts of provizio_radar_entities_api_context objects to initialize
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle) to initialize
 */
PROVIZIO__EXTERN_C void provizio_radar_entities_api_contexts_init(provizio_radar_entities_callback callback,
                                                                  void *user_data,
                                                                  provizio_radar_entities_api_context *contexts,
                                                                  size_t num_contexts);

/**
 * @brief Makes provizio_radar_entities_callback object handle a specific radar, which makes it skip packets
 * intended for other radars
 *
 * @param context provizio_radar_entities_api_context to be assigned
 * @param radar_position_id radar to assign
 * @return 0 in case it was successfully assigned, an error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_radar_entities_api_context_assign(provizio_radar_entities_api_context *context,
                                                                      provizio_radar_position radar_position_id);

/**
 * @brief Handles a single radar entities UDP packet from a single radar
 *
 * @param context Previously initialized provizio_radar_entities_api_context
 * @param packet Valid provizio_radar_entities_packet
 * @param packet_size The size of the packet, to check data is valid and avoid out-of-bounds access
 * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
 * other error code in case of another error
 *
 * @note radar_position_id of all packets handled by this context must be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_entities_packet(provizio_radar_entities_api_context *context,
                                                           provizio_radar_entities_packet *packet, size_t packet_size);

/**
 * @brief Handles a single radar entities UDP packet from one of multiple radars
 *
 * @param contexts Previously initialized array of num_contexts of provizio_radar_entities_api_context objects
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
 * @param packet Valid provizio_radar_entities_packet
 * @param packet_size The size of the packet, to check data is valid and avoid out-of-bounds access
 * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
 * PROVIZIO_E_OUT_OF_CONTEXTS in case num_contexts is not enough, other error code in case of another error
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_radars_entities_packet(provizio_radar_entities_api_context *contexts,
                                                                  size_t num_contexts,
                                                                  provizio_radar_entities_packet *packet,
                                                                  size_t packet_size);

/**
 * @brief Handles a single Provizio Radar API UDP packet from a single radar, that can be a correct
 * provizio_radar_entities_packet or something else
 *
 * @param context Previously initialized provizio_radar_entities_api_context
 * @param payload The payload of the UDP packet
 * @param payload_size The size of the payload in bytes
 * @return 0 if it's a provizio_radar_entities_packet and it was handled successfully, PROVIZIO_E_SKIPPED if it's not
 * a provizio_radar_entities_packet, other error code if it's a provizio_radar_entities_packet but its handling
 * failed for another reason
 *
 * @note if it's a provizio_radar_entities_packet, radar_position_id of all packets handled by this context must
 * be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_possible_radar_entities_packet(provizio_radar_entities_api_context *context,
                                                                          const void *payload, size_t payload_size);

/**
 * @brief Handles a single Provizio Radar API UDP packet from one of multiple radars, that can be a correct
 * provizio_radar_entities_packet or something else
 *
 * @param contexts Previously initialized array of num_contexts of provizio_radar_entities_api_context objects
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
 * @param payload The payload of the UDP packet
 * @param payload_size The size of the payload in bytes
 * @return 0 if it's a provizio_radar_entities_packet and it was handled successfully, PROVIZIO_E_SKIPPED if it's not
 * a provizio_radar_entities_packet, PROVIZIO_E_OUT_OF_CONTEXTS in case num_contexts is not enough, other error code
 * if it's a provizio_radar_entities_packet but its handling failed for another reason
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_possible_radars_entities_packet(
    provizio_radar_entities_api_context *contexts, size_t num_contexts, const void *payload, size_t payload_size);

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(offsetof(provizio_radar_entities_packet_header, protocol_header) == 0,
              "Unexpected position of protocol_header in provizio_radar_entities_packet_header");
static_assert(offsetof(provizio_radar_entities_packet_header, frame_index) == 4,
              "Unexpected position of frame_index in provizio_radar_entities_packet_header");
static_assert(offsetof(provizio_radar_entities_packet_header, timestamp) == 8,
              "Unexpected position of timestamp in provizio_radar_entities_packet_header");
static_assert(offsetof(provizio_radar_entities_packet_header, radar_position_id) == 16,
              "Unexpected position of radar_position_id in provizio_radar_entities_packet_header");
static_assert(offsetof(provizio_radar_entities_packet_header, total_entities_in_frame) == 18,
              "Unexpected position of total_entities_in_frame in provizio_radar_entities_packet_header");
static_assert(offsetof(provizio_radar_entities_packet_header, num_entities_in_packet) == 20,
              "Unexpected position of num_entities_in_packet in provizio_radar_entities_packet_header");
static_assert(sizeof(provizio_radar_entities_packet_header) == 24,
              "Unexpected size of provizio_radar_entities_packet_header");
static_assert(offsetof(provizio_radar_entities_packet, header) == 0,
              "Unexpected position of header in provizio_radar_entities_packet");
static_assert(offsetof(provizio_radar_entities_packet, radar_entities) == sizeof(provizio_radar_entities_packet_header),
              "Unexpected position of radar_entities in provizio_radar_entities_packet");
static_assert(sizeof(provizio_radar_entities_packet) ==
                  sizeof(provizio_radar_entities_packet_header) +
                      sizeof(provizio_radar_entity) * PROVIZIO__MAX_RADAR_ENTITIES_PER_UDP_PACKET,
              "Unexpected size of provizio_radar_entities_packet");
static_assert(offsetof(provizio_radar_entity, entity_id) == 0,
              "Unexpected position of entity_id in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, x_meters) == 4,
              "Unexpected position of x_meters in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, y_meters) == 8,
              "Unexpected position of y_meters in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, z_meters) == 12,
              "Unexpected position of z_meters in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, radar_relative_radial_velocity_m_s) == 16,
              "Unexpected position of radar_relative_radial_velocity_m_s in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, ground_relative_radial_velocity_m_s) == 20,
              "Unexpected position of ground_relative_radial_velocity_m_s in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, orientation) == 24,
              "Unexpected position of orientation in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, entity_class) == 40,
              "Unexpected position of entity_class in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, entity_confidence) == 41,
              "Unexpected position of entity_confidence in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, entity_class_confidence) == 42,
              "Unexpected position of entity_class_confidence in provizio_radar_entity");
static_assert(offsetof(provizio_radar_entity, reserved) == 43,
              "Unexpected position of reserved in provizio_radar_entity");
static_assert(sizeof(provizio_radar_entity) == 44, "Unexpected size of provizio_radar_entity");
static_assert(offsetof(provizio_size, x_meters) == 0, "Unexpected position of x_meters in provizio_size");
static_assert(offsetof(provizio_size, y_meters) == 4, "Unexpected position of y_meters in provizio_size");
static_assert(offsetof(provizio_size, z_meters) == 8, "Unexpected position of z_meters in provizio_size");
static_assert(sizeof(provizio_size) == 12, "Unexpected size of provizio_size");
#endif // defined(__cplusplus) && __cplusplus >= 201103L

#endif // PROVIZIO_RADAR_API_ENTITIES
