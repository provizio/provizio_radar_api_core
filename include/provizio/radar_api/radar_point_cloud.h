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

// Use packed structs for binary compatibility across all CPUs
#pragma pack(push, 1)

PROVIZIO__EXTERN_C typedef struct provizio_radar_point
{
    float x_meters;     // Forward, radar relative
    float y_meters;     // Left, radar relative
    float z_meters;     // Up, radar relative
    float velocity_m_s; // Forward, radar relative
    float signal_to_noise_ratio;
} provizio_radar_point;

PROVIZIO__EXTERN_C typedef struct provizio_radar_point_cloud_subset_header
{
    int32_t frame_index;            // Network bytes order
    int16_t subset_index;           // Network bytes order
    int16_t total_subsets_in_frame; // Network bytes order
    int16_t num_points;             // Network bytes order
} provizio_radar_point_cloud_subset_header;

#define PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET                                                                      \
    ((size_t)((PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES - sizeof(provizio_radar_point_cloud_subset_header)) /        \
              sizeof(provizio_radar_point)))

PROVIZIO__EXTERN_C typedef struct provizio_radar_point_cloud_subset
{
    provizio_radar_point_cloud_subset_header header;
    provizio_radar_point radar_points[PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET];
} provizio_radar_point_cloud_subset;

// Reset alignment settings
#pragma pack(pop)

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(offsetof(provizio_radar_point_cloud_subset_header, frame_index) == 0,
              "Unexpected position of frame_index in provizio_radar_point_cloud_subset_header");
static_assert(offsetof(provizio_radar_point_cloud_subset_header, subset_index) == 4,
              "Unexpected position of subset_index in provizio_radar_point_cloud_subset_header");
static_assert(offsetof(provizio_radar_point_cloud_subset_header, total_subsets_in_frame) == 6,
              "Unexpected position of total_subsets_in_frame in provizio_radar_point_cloud_subset_header");
static_assert(offsetof(provizio_radar_point_cloud_subset_header, num_points) == 8,
              "Unexpected position of num_points in provizio_radar_point_cloud_subset_header");
static_assert(sizeof(provizio_radar_point_cloud_subset_header) == 10,
              "Unexpected size of provizio_radar_point_cloud_subset_header");
static_assert(offsetof(provizio_radar_point_cloud_subset, header) == 0,
              "Unexpected position of header in provizio_radar_point_cloud_subset");
static_assert(offsetof(provizio_radar_point_cloud_subset, radar_points) ==
                  sizeof(provizio_radar_point_cloud_subset_header),
              "Unexpected position of radar_points in provizio_radar_point_cloud_subset");
static_assert(sizeof(provizio_radar_point_cloud_subset) ==
                  sizeof(provizio_radar_point_cloud_subset_header) +
                      sizeof(provizio_radar_point) * PROVIZIO__MAX_RADAR_POINTS_PER_UDP_PACKET,
              "Unexpected size of provizio_radar_point_cloud_subset");
#endif // defined(__cplusplus) && __cplusplus >= 201103L

#endif // PROVIZIO_RADAR_API_RADAR_POINT_CLOUD
