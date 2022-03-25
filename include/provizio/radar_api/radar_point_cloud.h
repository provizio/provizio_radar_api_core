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

#ifndef PROVIZIO_RADAR_API_RADAR_POINT_CLOUD
#define PROVIZIO_RADAR_API_RADAR_POINT_CLOUD

#include "provizio/common.h"
#include "provizio/socket.h"

// Used to identify provizio_radar_point_cloud_packet, can't change even on protocol updates
#define PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE ((uint16_t)1)

// To be incremented on any protocol changes (used for backward compatibility)
#define PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION ((uint16_t)1)

// Radar point cloud structures of the binary UDP protocol are defined here, see README.md for more details

typedef enum provizio_radar_position
{
    provizio_radar_position_front_center = 0,
    provizio_radar_position_front_left = 1,
    provizio_radar_position_front_right = 2,
    provizio_radar_position_rear_left = 3,
    provizio_radar_position_rear_right = 4,

    provizio_radar_position_custom = ((uint16_t)0x1000),

    provizio_radar_position_unknown = ((uint16_t)0xffff),
    provizio_radar_position_max = provizio_radar_position_unknown
} provizio_radar_position;

// Use packed structs intended to be sent for binary compatibility across all CPUs
#pragma pack(push, 1)

/**
 * @brief Represents a single radar point.
 *
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when accessing
 * fields on non-x86/x64 systems.
 * @see provizio_set_protocol_field_float
 * @see provizio_get_protocol_field_float
 */
typedef struct provizio_radar_point
{
    float x_meters;     // Forward, radar relative
    float y_meters;     // Left, radar relative
    float z_meters;     // Up, radar relative
    float velocity_m_s; // Forward, radar relative
    float signal_to_noise_ratio;
} provizio_radar_point;

/**
 * @brief 4-bytes header prefix used to identify a packet type and the protocol version
 *
 * @note This struct should never change, even on protocol updates
 * @see provizio_set_protocol_field_uint16_t
 * @see provizio_get_protocol_field_uint16_t
 */
typedef struct provizio_radar_point_cloud_packet_protocol_header
{
    uint16_t packet_type;      // = PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE
    uint16_t protocol_version; // = PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION (in the "current" protocol version)
} provizio_radar_point_cloud_packet_protocol_header;

/**
 * @brief Header placed in the beginning of each radar point cloud packet.
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
typedef struct provizio_radar_point_cloud_packet_header
{
    provizio_radar_point_cloud_packet_protocol_header protocol_header;

    uint32_t frame_index; // 0-based
    uint64_t timestamp;   // Time of the frame capture measured in absolute number of nanoseconds since the start of the
                          // GPS Epoch (midnight on Jan 6, 1980)
    uint16_t radar_position_id;     // Either one of provizio_radar_position enum values or a custom position id
    uint16_t total_points_in_frame; // Number of points in the entire frame
    uint16_t num_points_in_packet;  // Number of points in this single packet
    uint16_t reserved;              // Not used currently, kept for better alignment and potential future use
} provizio_radar_point_cloud_packet_header;

// Max number of radar points in a single UDP packet
#define PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET                                                                      \
    ((size_t)((PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES - sizeof(provizio_radar_point_cloud_packet_header)) /        \
              sizeof(provizio_radar_point)))

// Max number of radar points in a single point cloud
#define PROVIZIO__MAX_RADAR_POINTS_IN_POINT_CLOUD ((uint16_t)0xffff)

/**
 * @brief Structure to hold a single point cloud packet (one of some number of such packets per each radar frame).
 *
 * @note Not all of PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET radar_points may be present, see
 * header.num_points_in_packet for the actual number stored in this packet.
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when
 * accessing fields on non-x86/x64 systems.
 */
PROVIZIO__EXTERN_C typedef struct provizio_radar_point_cloud_packet
{
    provizio_radar_point_cloud_packet_header header;
    provizio_radar_point radar_points[PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET];
} provizio_radar_point_cloud_packet;

/**
 * @brief Returns the total size (in bytes) of the point cloud packet payload, based on its header.
 *
 * @param header Header of the packet
 * @return size_t Size (in bytes)
 */
PROVIZIO__EXTERN_C size_t
provizio_radar_point_cloud_packet_size(const provizio_radar_point_cloud_packet_header *header);

// Reset alignment settings
#pragma pack(pop)

/**
 * @brief A complete or partial radar point cloud
 */
typedef struct provizio_radar_point_cloud
{
    uint32_t frame_index;
    uint64_t timestamp;
    uint16_t radar_position_id;
    uint16_t num_points_expected;
    uint16_t num_points_received;
    provizio_radar_point points[PROVIZIO__MAX_RADAR_POINTS_IN_POINT_CLOUD];
} provizio_radar_point_cloud;

struct provizio_radar_point_cloud_api_context;
typedef void (*provizio_radar_point_cloud_callback)(const provizio_radar_point_cloud *point_cloud,
                                                    struct provizio_radar_point_cloud_api_context *context);

typedef struct provizio_radar_point_cloud_api_context_impl
{
    PROVIZIO__SOCKET socket;
} provizio_radar_point_cloud_api_context_impl;

/**
 * @brief Keeps all data required for functioning of radar point clouds API
 */
typedef struct provizio_radar_point_cloud_api_context
{
    provizio_radar_point_cloud_callback callback;
    void *user_data;

    provizio_radar_point_cloud_api_context_impl impl;
} provizio_radar_point_cloud_api_context;

/**
 * @brief Initializes a provizio_radar_point_cloud_api_context object
 *
 * @param callback Function to be called on receiving a complete or partial radar point cloud
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param out_context The provizio_radar_point_cloud_api_context object to be initialized
 */
PROVIZIO__EXTERN_C void provizio_radar_point_cloud_api_context_create(
    provizio_radar_point_cloud_callback callback, void *user_data, provizio_radar_point_cloud_api_context *out_context);

/**
 * @brief Handles a single radar point cloud UDP packet
 *
 * @param context Previously created provizio_radar_point_cloud_api_context (doesn't have to be opened)
 * @param packet Valid provizio_radar_point_cloud_packet
 */
PROVIZIO__EXTERN_C void provizio_handle_radar_point_cloud_packet(provizio_radar_point_cloud_api_context *context,
                                                                 provizio_radar_point_cloud_packet *packet);

/**
 * @brief Handles a single Provizio Radar API UDP packet, that can be a provizio_radar_point_cloud_packet or something
 * else
 *
 * @param context Previously created provizio_radar_point_cloud_api_context (doesn't have to be opened)
 * @param payload The payload of the UDP packet
 * @param payload_size The size of the payload in bytes
 * @return true if it's a provizio_radar_point_cloud_packet, false otherwise
 */
PROVIZIO__EXTERN_C bool provizio_handle_possible_radar_point_cloud_packet(
    provizio_radar_point_cloud_api_context *context, const uint8_t *payload, size_t payload_size);

#define PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT ((uint16_t)7769)

/**
 * @brief Opens a provizio_radar_point_cloud_api_context to start receiving packets by UDP
 *
 * @param context Previously created provizio_radar_point_cloud_api_context, shouldn't be opened yet
 * @param udp_port UDP port to receive from, by default = PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT
 * @param receive_timeout Max number of nanoseconds provizio_radar_point_cloud_api_receive_packet should wait for a
 * packet, or 0 to wait as long as required
 * @return 0 if successfull, error code otherwise
 */
PROVIZIO__EXTERN_C int provizio_radar_point_cloud_api_context_open(provizio_radar_point_cloud_api_context *context,
                                                                   uint16_t udp_port, uint64_t receive_timeout);

/**
 * @brief Receive and handle the next UDP packet using a previously opened provizio_radar_point_cloud_api_context
 *
 * @param context Previously created and opened provizio_radar_point_cloud_api_context
 * @return 0 if received successfully, EAGAIN if timed out, other error value if failed
 */
PROVIZIO__EXTERN_C int provizio_radar_point_cloud_api_receive_packet(provizio_radar_point_cloud_api_context *context);

/**
 * @brief Closes a previously opened provizio_radar_point_cloud_api_context
 *
 * @param context Previously created and opened provizio_radar_point_cloud_api_context
 * @return 0 if successfull, error code otherwise
 */
PROVIZIO__EXTERN_C int provizio_radar_point_cloud_api_context_close(provizio_radar_point_cloud_api_context *context);

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(offsetof(provizio_radar_point_cloud_packet_protocol_header, packet_type) == 0,
              "Unexpected position of protocol_header in provizio_radar_point_cloud_packet_protocol_header");
static_assert(offsetof(provizio_radar_point_cloud_packet_protocol_header, protocol_version) == 2,
              "Unexpected position of frame_index in provizio_radar_point_cloud_packet_protocol_header");
static_assert(sizeof(provizio_radar_point_cloud_packet_protocol_header) == 4,
              "Unexpected size of provizio_radar_point_cloud_packet_protocol_header");
static_assert(offsetof(provizio_radar_point_cloud_packet_header, protocol_header) == 0,
              "Unexpected position of protocol_header in provizio_radar_point_cloud_packet_header");
static_assert(offsetof(provizio_radar_point_cloud_packet_header, frame_index) == 4,
              "Unexpected position of frame_index in provizio_radar_point_cloud_packet_header");
static_assert(offsetof(provizio_radar_point_cloud_packet_header, timestamp) == 8,
              "Unexpected position of timestamp in provizio_radar_point_cloud_packet_header");
static_assert(offsetof(provizio_radar_point_cloud_packet_header, radar_position_id) == 16,
              "Unexpected position of radar_position_id in provizio_radar_point_cloud_packet_header");
static_assert(offsetof(provizio_radar_point_cloud_packet_header, total_points_in_frame) == 18,
              "Unexpected position of total_points_in_frame in provizio_radar_point_cloud_packet_header");
static_assert(offsetof(provizio_radar_point_cloud_packet_header, num_points_in_packet) == 20,
              "Unexpected position of num_points_in_packet in provizio_radar_point_cloud_packet_header");
static_assert(sizeof(provizio_radar_point_cloud_packet_header) == 24,
              "Unexpected size of provizio_radar_point_cloud_packet_header");
static_assert(offsetof(provizio_radar_point_cloud_packet, header) == 0,
              "Unexpected position of header in provizio_radar_point_cloud_packet");
static_assert(offsetof(provizio_radar_point_cloud_packet, radar_points) ==
                  sizeof(provizio_radar_point_cloud_packet_header),
              "Unexpected position of radar_points in provizio_radar_point_cloud_packet");
static_assert(sizeof(provizio_radar_point_cloud_packet) ==
                  sizeof(provizio_radar_point_cloud_packet_header) +
                      sizeof(provizio_radar_point) * PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET,
              "Unexpected size of provizio_radar_point_cloud_packet");
#endif // defined(__cplusplus) && __cplusplus >= 201103L

#endif // PROVIZIO_RADAR_API_RADAR_POINT_CLOUD
