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

#include "provizio/radar_api/common.h"

// Used to identify provizio_radar_point_cloud_packet, can't change even on protocol updates
#define PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE ((uint16_t)1)

// To be incremented on any protocol changes (used for backward compatibility)
#define PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION ((uint16_t)1)

// Radar point cloud structures of the binary UDP protocol are defined here, see README.md for more details

// Use packed structs for binary compatibility across all CPUs
#pragma pack(push, 1)

/**
 * @brief Represents a single radar point.
 *
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when accessing
 * fields on non-x86/x64 systems.
 */
PROVIZIO__EXTERN_C typedef struct provizio_radar_point
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
 */
PROVIZIO__EXTERN_C typedef struct provizio_radar_point_cloud_packet_protocol_header
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
 */
PROVIZIO__EXTERN_C typedef struct provizio_radar_point_cloud_packet_header
{
    provizio_radar_point_cloud_packet_protocol_header protocol_header;

    uint32_t frame_index;           // 0-based
    uint64_t timestamp;             // Time of the frame capture measured in milliseconds since the UNIX epoch
    uint16_t radar_position_id;     // Either one of provizio_radar_position enum values or a custom position id
    uint16_t total_points_in_frame; // Number of points in the entire frame
    uint16_t num_points_in_packet;  // Number of points in this single packet
    uint16_t reserved;              // Not used currently, kept for better alignment and potential future use
} provizio_radar_point_cloud_packet_header;

#define PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET                                                                      \
    ((size_t)((PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES - sizeof(provizio_radar_point_cloud_packet_header)) /        \
              sizeof(provizio_radar_point)))

/**
 * @brief Structure to hold a single point cloud packet (one of some number of such packets per each radar frame).
 *
 * @note Not all of PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET radar_points may be present, see
 * header.num_points_in_packet for the actual number stored in this packet.
 * @warning Given packed structures are used, fields alignment is not guaranteed and caution is needed when accessing
 * fields on non-x86/x64 systems.
 */
PROVIZIO__EXTERN_C typedef struct provizio_radar_point_cloud_packet
{
    provizio_radar_point_cloud_packet_header header;
    provizio_radar_point radar_points[PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET];
} provizio_radar_point_cloud_packet;

// Reset alignment settings
#pragma pack(pop)

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
