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

PROVIZIO__NAMESPACE_OPEN

// Use packed structs for binary compatibility across all CPUs
#pragma pack(push, 1)

typedef struct PROVIZIO__NAMESPACE(radar_point)
{
    float x_meters;     // Forward, radar relative
    float y_meters;     // Left, radar relative
    float z_meters;     // Up, radar relative
    float velocity_m_s; // Forward, radar relative
    float signal_to_noise_ratio;
} PROVIZIO__NAMESPACE(radar_point);

typedef struct PROVIZIO__NAMESPACE(radar_point_cloud_subset_header)
{
    int32_t frame_index;            // Network bytes order
    int16_t subset_index;           // Network bytes order
    int16_t total_subsets_in_frame; // Network bytes order
    int16_t num_points;             // Network bytes order
} PROVIZIO__NAMESPACE(radar_point_cloud_subset_header);

#define PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET                                                                      \
    ((size_t)((PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES -                                                            \
               sizeof(PROVIZIO__NAMESPACE_GLOBAL(radar_point_cloud_subset_header))) /                                  \
              sizeof(PROVIZIO__NAMESPACE_GLOBAL(radar_point))))

typedef struct PROVIZIO__NAMESPACE(radar_point_cloud_subset)
{
    PROVIZIO__NAMESPACE(radar_point_cloud_subset_header) header;
    PROVIZIO__NAMESPACE(radar_point) radar_points[PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET];
} PROVIZIO__NAMESPACE(radar_point_cloud_subset);

// Reset alignment settings
#pragma pack(pop)

#if __cplusplus >= 201103L
static_assert(offsetof(PROVIZIO__NAMESPACE(radar_point_cloud_subset_header), frame_index) == 0,
              "Unexpected position of frame_index in radar_point_cloud_subset_header");
static_assert(offsetof(PROVIZIO__NAMESPACE(radar_point_cloud_subset_header), subset_index) == 4,
              "Unexpected position of subset_index in radar_point_cloud_subset_header");
static_assert(offsetof(PROVIZIO__NAMESPACE(radar_point_cloud_subset_header), total_subsets_in_frame) == 6,
              "Unexpected position of total_subsets_in_frame in radar_point_cloud_subset_header");
static_assert(offsetof(PROVIZIO__NAMESPACE(radar_point_cloud_subset_header), num_points) == 8,
              "Unexpected position of num_points in radar_point_cloud_subset_header");
static_assert(sizeof(PROVIZIO__NAMESPACE(radar_point_cloud_subset_header)) == 10,
              "Unexpected size of radar_point_cloud_subset_header");
static_assert(offsetof(PROVIZIO__NAMESPACE(radar_point_cloud_subset), header) == 0,
              "Unexpected position of header in radar_point_cloud_subset");
static_assert(offsetof(PROVIZIO__NAMESPACE(radar_point_cloud_subset), radar_points) ==
                  sizeof(radar_point_cloud_subset_header),
              "Unexpected position of radar_points in radar_point_cloud_subset");
static_assert(sizeof(PROVIZIO__NAMESPACE(radar_point_cloud_subset)) ==
                  sizeof(PROVIZIO__NAMESPACE(radar_point_cloud_subset_header)) +
                      sizeof(PROVIZIO__NAMESPACE(radar_point)) * PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET,
              "Unexpected size of radar_point_cloud_subset");
#endif // __cplusplus

PROVIZIO__NAMESPACE_CLOSE

#endif // PROVIZIO_RADAR_API_RADAR_POINT_CLOUD
