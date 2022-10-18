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

#ifndef PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION
#define PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION

#include "provizio/common.h"
#include "provizio/radar_api/radar_points_accumulation_filters.h"
#include "provizio/radar_api/radar_points_accumulation_types.h"

/**
 * @brief Initializes an array of provizio_accumulated_radar_point_cloud to be later used for point clouds accumulation
 * as a circular buffer.
 *
 * @param accumulated_point_clouds Pointer to the array of provizio_accumulated_radar_point_cloud to initialize.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in the array.
 * @see provizio_accumulate_radar_point_cloud
 */
PROVIZIO__EXTERN_C void provizio_accumulated_radar_point_clouds_init(
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

/**
 * @brief Pushes a new radar point cloud to the array of provizio_accumulated_radar_point_cloud (used as a circular
 * buffer). It enables tracking historical radar points as they would be seen later by the same radar if it could still
 * see them. If accumulated_point_clouds buffer is full then the oldest accumulated point cloud gets dropped.
 *
 * @param point_cloud The new radar point cloud to be accumulated.
 * @param fix_when_received A provizio_enu_fix of the radar at the moment of the point cloud capture. Being the fix of
 * the radar, not the ego vehicle, it has to be pre-transformed to respect the position and orientation of the radar
 * relative to the reference frame of the localization system (extrinsics). All accumulated point clouds in
 * accumulated_point_clouds buffer must use the same ENU reference point. Accumulation can be reset using
 * provizio_accumulated_radar_point_clouds_init in case a new reference point is required (in long journeys).
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @param filter Function that defines which points are to be accumulated and which ones to be dropped. May be NULL to
 * accumulate all, &provizio_radar_points_accumulation_filter_static to accumulate static points, or a custom filter.
 * @param filter_user_data Specifies user_data argument value of the filter (may be NULL).
 * @return provizio_accumulated_radar_point_cloud_iterator pointing to the just pushed point cloud. It can be used to
 * iterate over point clouds accumulated so far - from newest to oldest.
 * @see provizio_accumulate_radar_point_cloud_static
 * @see provizio_radar_point_cloud
 * @see provizio_enu_fix
 * @see provizio_accumulated_radar_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator
 * @see provizio_accumulated_radar_point_clouds_init
 * @see provizio_radar_points_accumulation_filter
 * @see provizio_radar_points_accumulation_filter_static
 */
PROVIZIO__EXTERN_C provizio_accumulated_radar_point_cloud_iterator provizio_accumulate_radar_point_cloud(
    const provizio_radar_point_cloud *point_cloud, const provizio_enu_fix *fix_when_received,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_points_accumulation_filter filter, void *filter_user_data);

/**
 * @brief A shortcut for provizio_accumulate_radar_point_cloud using provizio_radar_points_accumulation_filter_static.
 * @param point_cloud The new radar point cloud to be accumulated.
 * @param fix_when_received A provizio_enu_fix of the radar at the moment of the point cloud capture. Being the fix of
 * the radar, not the ego vehicle, it has to be pre-transformed to respect the position and orientation of the radar
 * relative to the reference frame of the localization system (extrinsics). All accumulated point clouds in
 * accumulated_point_clouds buffer must use the same ENU reference point. Accumulation can be reset using
 * provizio_accumulated_radar_point_clouds_init in case a new reference point is required (in long journeys).
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @return provizio_accumulated_radar_point_cloud_iterator pointing to the just pushed point cloud. It can be used to
 * iterate over point clouds accumulated so far - from newest to oldest.
 *
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_radar_points_accumulation_filter_static
 */
#define provizio_accumulate_radar_point_cloud_static(point_cloud, fix_when_received, accumulated_point_clouds,         \
                                                     num_accumulated_point_clouds)                                     \
    provizio_accumulate_radar_point_cloud((point_cloud), (fix_when_received), (accumulated_point_clouds),              \
                                          (num_accumulated_point_clouds),                                              \
                                          &provizio_radar_points_accumulation_filter_static, NULL)

/**
 * @brief Returns a number of point clouds accumulated so far in accumulated_point_clouds buffer.
 *
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @return Number of point clouds accumulated so far in accumulated_point_clouds buffer.
 */
PROVIZIO__EXTERN_C size_t provizio_accumulated_radar_point_clouds_count(
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

/**
 * @brief Returns a total number of points accumulated so far in all point clouds in accumulated_point_clouds buffer.
 *
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @return Total number of points accumulated so far in all point clouds in accumulated_point_clouds buffer.
 */
PROVIZIO__EXTERN_C size_t provizio_accumulated_radar_points_count(
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

/**
 * @brief Checks if a provizio_accumulated_radar_point_cloud_iterator is an end iterator, i.e. can't iterate anymore.
 *
 * @param iterator Pointer to an provizio_accumulated_radar_point_cloud_iterator to be checked.
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @return A non-zero value if it's an end iterator, 0 otherwise.
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_next_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_next_point
 */
PROVIZIO__EXTERN_C int8_t provizio_accumulated_radar_point_cloud_iterator_is_end(
    const provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

/**
 * @brief Moves the iterator to the next accumulated point cloud. The iterator may become an end iterator if there are
 * no more accumulated point clouds left.
 *
 * @param iterator An iterator to be moved.
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @see provizio_accumulated_radar_point_cloud_iterator_next_point
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_is_end
 * @see provizio_accumulated_radar_point_cloud_iterator_get_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_get_point
 */
PROVIZIO__EXTERN_C void provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

/**
 * @brief Moves the iterator to the next accumulated point (it maybe next point of the same
 * accumulated point cloud or, when it's over, the first point of the next accumulated point cloud). The iterator may
 * become an end iterator if there are no more accumulated points left.
 *
 * @param iterator An iterator to be moved.
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @see provizio_accumulated_radar_point_cloud_iterator_next_point_cloud
 * @see provizio_accumulate_radar_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_is_end
 * @see provizio_accumulated_radar_point_cloud_iterator_get_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator_get_point
 */
PROVIZIO__EXTERN_C void provizio_accumulated_radar_point_cloud_iterator_next_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds);

/**
 * @brief Returns an accumulated point cloud that the specified non-end iterator points to; with an option to either
 * transform its points so their positions are returned relative to the specified provizio_enu_fix of the radar, or
 * generate a 4x4 matrix that can perform such a transformation by multiplying to positions of the accumulated points.
 *
 * @param iterator A provizio_accumulated_radar_point_cloud_iterator.
 * @param current_fix A provizio_enu_fix of the radar to transform relative to, i.e. where the same radar is at now. May
 * be NULL in case both optional_out_transformed_point_cloud and optional_out_transformation_matrix are also NULL.
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @param optional_out_transformed_point_cloud When non-NULL, stores transformed radar points; i.e. the points as they
 * would be "seen" by the same radar in its current position, if it could still "see" them.
 * @param optional_out_transformation_matrix When non-NULL, must point to a float array 16 floats (64 bytes) long to
 * store a 4x4 transformation matrix in column major order. Multiplying this matrix to points positions (as (x, y, z, 1)
 * 4d-vectors) transforms points from where they were relative to the radar at the moment of their capture to where they
 * would be "seen" by the same radar in its current position, if it could still "see" them.
 * @return Untransformed accumulated radar point cloud, or NULL if iterator is an end iterator.
 * @see provizio_accumulated_radar_point_cloud_iterator
 * @see provizio_enu_fix
 *
 * @note When non-NULL optional_out_transformed_point_cloud is specified, the transformation is done without any use of
 * hardware acceleration, so it can be slow. When runtime performance is important and appropriate hw capabilities are
 * present, it's recommended to use optional_out_transformation_matrix and then hw-accelerated transformation instead.
 */
PROVIZIO__EXTERN_C const provizio_accumulated_radar_point_cloud *
provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point_cloud *optional_out_transformed_point_cloud, float *optional_out_transformation_matrix);

/**
 * @brief Returns an accumulated point that the specified non-end iterator points to; with an option to either transform
 * it so its position is returned relative to the specified provizio_enu_fix of the radar, or generate a 4x4 matrix that
 * can perform such a transformation by multiplying to its position.
 *
 * @param iterator A provizio_accumulated_radar_point_cloud_iterator.
 * @param current_fix A provizio_enu_fix of the radar to transform relative to, i.e. where the same radar is at now. May
 * be NULL in case both optional_out_transformed_point and optional_out_transformation_matrix are also NULL.
 * @param accumulated_point_clouds An array of provizio_accumulated_radar_point_cloud previously initialized with
 * provizio_accumulated_radar_point_clouds_init to store accumulated points clouds as a circular buffer.
 * @param num_accumulated_point_clouds Number of provizio_accumulated_radar_point_cloud in accumulated_point_clouds.
 * @param optional_out_transformed_point When non-NULL, stores a transformed radar point; i.e. the point as it would be
 * "seen" by the same radar in its current position, if it could still "see" this point.
 * @param optional_out_transformation_matrix When non-NULL, must point to a float array 16 floats (64 bytes) long to
 * store a 4x4 transformation matrix in column major order. Multiplying this matrix to a point position (as (x, y, z, 1)
 * 4d-vector) transforms the point from where it was relative to the radar at the moment of its capture to where it
 * would be "seen" by the same radar in its current position, if it could still "see" this point.
 * @return Untransformed accumulated radar point, or NULL if iterator is an end iterator.
 * @see provizio_accumulated_radar_point_cloud_iterator_get_point_cloud
 * @see provizio_accumulated_radar_point_cloud_iterator
 * @see provizio_enu_fix
 *
 * @note When non-NULL optional_out_transformed_point is specified, the transformation is done without any use of
 * hardware acceleration, so it can be slow. When runtime performance is important and appropriate hw capabilities are
 * present, it's recommended to use optional_out_transformation_matrix and then hw-accelerated transformation instead.
 * Also, provizio_accumulated_radar_point_cloud_iterator_get_point_cloud is recommended instead, so
 * optional_out_transformation_matrix doesn't have to be generated separately for every point as it's same for the
 * entire accumulated point cloud.
 */
PROVIZIO__EXTERN_C const provizio_radar_point *provizio_accumulated_radar_point_cloud_iterator_get_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point *optional_out_transformed_point, float *optional_out_transformation_matrix);

#endif // PROVIZIO_RADAR_API_RADAR_POINTS_ACCUMULATION
