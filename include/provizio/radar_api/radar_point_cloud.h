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
typedef struct
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
    ((uint16_t)((PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES - sizeof(provizio_radar_point_cloud_packet_header)) /      \
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
 *
 * @note Complete point clouds always have num_points_received == num_points_expected
 */
typedef struct provizio_radar_point_cloud
{
    uint32_t frame_index; // 0-based
    uint64_t timestamp;   // Time of the frame capture measured in absolute number of nanoseconds since the start of the
                          // GPS Epoch (midnight on Jan 6, 1980)
    uint16_t radar_position_id;   // Either one of provizio_radar_position enum values or a custom position id
    uint16_t num_points_expected; // Number of points in the entire frame
    uint16_t num_points_received; // Number of points in the frame received so far
    provizio_radar_point radar_points[PROVIZIO__MAX_RADAR_POINTS_IN_POINT_CLOUD];
} provizio_radar_point_cloud;

struct provizio_radar_point_cloud_api_context;
typedef void (*provizio_radar_point_cloud_callback)(const provizio_radar_point_cloud *point_cloud,
                                                    struct provizio_radar_point_cloud_api_context *context);

#define PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT 2
typedef struct provizio_radar_point_cloud_api_context_impl
{
    uint32_t latest_frame;
    provizio_radar_point_cloud
        point_clouds_being_received[PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT];
} provizio_radar_point_cloud_api_context_impl;

/**
 * @brief Keeps all data required for functioning of radar point clouds API
 */
typedef struct provizio_radar_point_cloud_api_context
{
    provizio_radar_point_cloud_callback callback;
    void *user_data;
    uint16_t radar_position_id;

    provizio_radar_point_cloud_api_context_impl impl;
} provizio_radar_point_cloud_api_context;

typedef struct provizio_radar_point_cloud_api_connection
{
    PROVIZIO__SOCKET sock;
} provizio_radar_point_cloud_api_connection;

/**
 * @brief Initializes a provizio_radar_point_cloud_api_context object to handle a single radar
 *
 * @param callback Function to be called on receiving a complete or partial radar point cloud
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param context The provizio_radar_point_cloud_api_context object to initialize
 *
 * @warning radar_position_id of all packets handled by this context must be same
 */
PROVIZIO__EXTERN_C void provizio_radar_point_cloud_api_context_init(provizio_radar_point_cloud_callback callback,
                                                                    void *user_data,
                                                                    provizio_radar_point_cloud_api_context *context);

/**
 * @brief Initializes multiple provizio_radar_point_cloud_api_context objects to handle packets from multiple radars
 *
 * @param callback Function to be called on receiving a complete or partial radar point cloud
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param contexts Array of num_contexts of provizio_radar_point_cloud_api_context objects to initialize
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle) to initialize
 */
PROVIZIO__EXTERN_C void provizio_radar_point_cloud_api_contexts_init(provizio_radar_point_cloud_callback callback,
                                                                     void *user_data,
                                                                     provizio_radar_point_cloud_api_context *contexts,
                                                                     size_t num_contexts);

/**
 * @brief Makes provizio_radar_point_cloud_api_context object handle a specific radar, which makes it skip packets
 * intended for other radars
 *
 * @param context provizio_radar_point_cloud_api_context to be assigned
 * @param radar_position_id radar to assign
 * @return 0 in case it was successfully assigned, an error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_radar_point_cloud_api_context_assign(
    provizio_radar_point_cloud_api_context *context, provizio_radar_position radar_position_id);

/**
 * @brief Handles a single radar point cloud UDP packet from a single radar
 *
 * @param context Previously initialized provizio_radar_point_cloud_api_context
 * @param packet Valid provizio_radar_point_cloud_packet
 * @param packet_size The size of the packet, to check data is valid and avoid out-of-bounds access
 * @return 0 in case the packet was handled successfully, EAGAIN in case the packet was skipped as obsolete, other error
 * code in case of another error
 *
 * @warning radar_position_id of all packets handled by this context must be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_radar_point_cloud_packet(provizio_radar_point_cloud_api_context *context,
                                                                    provizio_radar_point_cloud_packet *packet,
                                                                    size_t packet_size);

/**
 * @brief Handles a single radar point cloud UDP packet from one of multiple radars
 *
 * @param contexts Previously initialized array of num_contexts of provizio_radar_point_cloud_api_context objects
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
 * @param packet Valid provizio_radar_point_cloud_packet
 * @param packet_size The size of the packet, to check data is valid and avoid out-of-bounds access
 * @return 0 in case the packet was handled successfully, EAGAIN in case the packet was skipped as obsolete, EBUSY in
 * case num_contexts is not enough, other error code in case of another error
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_radars_point_cloud_packet(provizio_radar_point_cloud_api_context *contexts,
                                                                     size_t num_contexts,
                                                                     provizio_radar_point_cloud_packet *packet,
                                                                     size_t packet_size);

/**
 * @brief Handles a single Provizio Radar API UDP packet from a single radar, that can be a correct
 * provizio_radar_point_cloud_packet or something else
 *
 * @param context Previously initialized provizio_radar_point_cloud_api_context
 * @param payload The payload of the UDP packet
 * @param payload_size The size of the payload in bytes
 * @return 0 if it's a provizio_radar_point_cloud_packet and it was handled successfully, EAGAIN if it's not a
 * provizio_radar_point_cloud_packet, other error code if it's a provizio_radar_point_cloud_packet but its handling
 * failed for another reason
 *
 * @warning if it's a provizio_radar_point_cloud_packet, radar_position_id of all packets handled by this context must
 * be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_possible_radar_point_cloud_packet(
    provizio_radar_point_cloud_api_context *context, const void *payload, size_t payload_size);

/**
 * @brief Handles a single Provizio Radar API UDP packet from one of multiple radars, that can be a correct
 * provizio_radar_point_cloud_packet or something else
 *
 * @param contexts Previously initialized array of num_contexts of provizio_radar_point_cloud_api_context objects
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
 * @param payload The payload of the UDP packet
 * @param payload_size The size of the payload in bytes
 * @return 0 if it's a provizio_radar_point_cloud_packet and it was handled successfully, EAGAIN if it's not a
 * provizio_radar_point_cloud_packet, EBUSY in case num_contexts is not enough, other error code if it's a
 * provizio_radar_point_cloud_packet but its handling failed for another reason
 *
 * @warning if it's a provizio_radar_point_cloud_packet, radar_position_id of all packets handled by this context must
 * be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_handle_possible_radars_point_cloud_packet(
    provizio_radar_point_cloud_api_context *contexts, size_t num_contexts, const void *payload, size_t payload_size);

#define PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT ((uint16_t)7769)

/**
 * @brief Connect to the radar point clouds API to start receiving packets by UDP
 *
 * @param udp_port UDP port to receive from, by default = PROVIZIO__RADAR_API_POINT_CLOUD_DEFAULT_PORT
 * @param receive_timeout_ns Max number of nanoseconds provizio_radar_point_cloud_api_receive_packet should wait for a
 * packet, or 0 to wait as long as required
 * @param check_connection Use any non-zero value if the connection is to be checked to be receiving anything prior to
 * returning a successful result
 * @param out_connection A provizio_radar_point_cloud_api_connection to store the connection handle
 * @return 0 if received successfully, EAGAIN if timed out, other error value if failed for another reason
 */
PROVIZIO__EXTERN_C int32_t
provizio_radar_point_cloud_api_open_connection(uint16_t udp_port, uint64_t receive_timeout_ns, uint8_t check_connection,
                                               provizio_radar_point_cloud_api_connection *out_connection);

/**
 * @brief Receive and handle the next UDP packet from a single radar using a previously connected API and a previously
 * initialized provizio_radar_point_cloud_api_context
 *
 * @param context provizio_radar_point_cloud_api_context previously initialized with
 * provizio_radar_point_cloud_api_context_init
 * @param connection A provizio_radar_point_cloud_api_connection previously connected with
 * provizio_radar_point_cloud_api_open_connection
 * @return 0 if received successfully, EAGAIN if timed out, other error value if failed for another reason
 *
 * @warning radar_position_id of all packets handled by this context must be same (returns an error otherwise)
 */
PROVIZIO__EXTERN_C int32_t provizio_radar_point_cloud_api_context_receive_packet(
    provizio_radar_point_cloud_api_context *context, provizio_radar_point_cloud_api_connection *connection);

/**
 * @brief Receive and handle the next UDP packet from one of multiple radars using a previously connected API and a
 * previously initialized array of provizio_radar_point_cloud_api_context objects
 *
 * @param contexts Array of provizio_radar_point_cloud_api_context objects previously initialized with
 * provizio_radar_point_cloud_api_contexts_init
 * @param num_contexts Number of objects in contexts
 * @param connection A provizio_radar_point_cloud_api_connection previously connected with
 * provizio_radar_point_cloud_api_open_connection
 * @return 0 if received successfully, EAGAIN if timed out or the received packet was obsolete, EBUSY in case
 * num_contexts is not enough, other error value if failed for another reason
 */
PROVIZIO__EXTERN_C int32_t provizio_radar_point_cloud_api_contexts_receive_packet(
    provizio_radar_point_cloud_api_context *contexts, size_t num_contexts,
    provizio_radar_point_cloud_api_connection *connection);

/**
 * @brief Closes a previously connected radar point clouds API
 *
 * @param connection A provizio_radar_point_cloud_api_connection previously connected with
 * provizio_radar_point_cloud_api_open_connection
 * @return 0 if successfull, error code otherwise
 */
PROVIZIO__EXTERN_C int32_t
provizio_radar_point_cloud_api_close_connection(provizio_radar_point_cloud_api_connection *connection);

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
