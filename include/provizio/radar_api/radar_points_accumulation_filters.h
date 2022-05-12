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

#ifndef PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION_FILTERS
#define PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION_FILTERS

#include "provizio/common.h"
#include "provizio/radar_api/radar_points_accumulation_types.h"

/**
 * @brief Function type to be used as a filter for point clouds, i.e. a function that defines which point clouds are
 * to be accumulated and which to be dropped.
 *
 * @param in_points - Input (unfiltered) array of points.
 * @param num_in_points - Number of points in in_points.
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @param new_iterator Iterator to the point cloud being accumulated.
 * @param user_data Custom argument to be passed to the filter (specified when accumulating), may be NULL.
 * @param out_points Output (filtered) array of points, to be assigned by the filter, at least num_in_points large.
 * @param num_out_points Pointer to the output (filtered) number of points, to be set by the filter (can't exceed
 * PROVIZIO__MAX_RADAR_POINTS_IN_POINT_CLOUD).
 * @see provizio_enu_fix
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator
 */
typedef void (*provizio_radar_points_accumulation_filter)(
    const provizio_radar_point *in_points, uint16_t num_in_points,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    const provizio_accumulated_radar_point_cloud_iterator *new_iterator, void *user_data,
    provizio_radar_point *out_points, uint16_t *num_out_points);

/**
 * @brief A provizio_radar_points_accumulation_filter that accumulates all points, i.e. no doesn't filter.
 *
 * @param in_points - Input (unfiltered) array of points.
 * @param num_in_points - Number of points in in_points.
 * @param accumulated_point_clouds - Ignored by this filter.
 * @param num_accumulated_point_clouds - Ignored by this filter.
 * @param new_iterator - Ignored by this filter.
 * @param user_data - Ignored by this filter, use NULL.
 * @param out_points Output (filtered) array of points, to be assigned by the filter, at least num_in_points large.
 * @param num_out_points Pointer to the output (filtered) number of points, to be set by the filter (can't exceed
 * PROVIZIO__MAX_RADAR_POINTS_IN_POINT_CLOUD).
 * @see provizio_enu_fix
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_radar_points_accumulation_filter
 */
PROVIZIO__EXTERN_C void provizio_radar_points_accumulation_filter_copy_all(
    const provizio_radar_point *in_points, uint16_t num_in_points,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    const provizio_accumulated_radar_point_cloud_iterator *new_iterator, void *user_data,
    provizio_radar_point *out_points, uint16_t *num_out_points);

/**
 * @brief A provizio_radar_points_accumulation_filter that accumulates only static (non-moving) points, as defined from
 * their radial velocities relative to the radar and the radar's own movement as detected by GNSS fixes history.
 *
 * @param in_points - Input (unfiltered) array of points.
 * @param num_in_points - Number of points in in_points.
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @param new_iterator Iterator to the point cloud being accumulated.
 * @param user_data Ignored by this filter, use NULL.
 * @param out_points Output (filtered) array of points, to be assigned by the filter, at least num_in_points large.
 * @param num_out_points Pointer to the output (filtered) number of points, to be set by the filter (can't exceed
 * PROVIZIO__MAX_RADAR_POINTS_IN_POINT_CLOUD).
 * @see provizio_enu_fix
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_radar_points_accumulation_filter
 */
PROVIZIO__EXTERN_C void provizio_radar_points_accumulation_filter_static(
    const provizio_radar_point *in_points, uint16_t num_in_points,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    const provizio_accumulated_radar_point_cloud_iterator *new_iterator, void *user_data,
    provizio_radar_point *out_points, uint16_t *num_out_points);

#endif // PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION_FILTERS
