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

#ifndef PROVIZIO_RADAR_API_CONTEXT
#define PROVIZIO_RADAR_API_CONTEXT

#include "provizio/radar_api/common.h"
#include "provizio/radar_api/radar_position.h"
#include "provizio/radar_api/radar_ego_motion.h"
#include "provizio/radar_api/radar_point_cloud.h"

// To be incremented on any breaking protocol changes (used for backward compatibility)
#define PROVIZIO__RADAR_API_CONTEXT_PROTOCOL_VERSION ((uint16_t)1)

// Use packed structs intended to be sent for binary compatibility across all CPUs
#pragma pack(push, 1)


/**
 * @brief Keeps all data required for functioning of radar point clouds API
 */
typedef struct provizio_radar_api_context
{
    uint16_t radar_position_id;
    provizio_radar_point_cloud_callback point_cloud_callback;
    void *point_cloud_user_data;
    provizio_radar_ego_motion_callback ego_motion_callback;
    void *ego_motion_user_data;

    provizio_radar_ego_motion motion;

    provizio_radar_point_cloud_impl impl;
} provizio_radar_api_context;

/**
 * @brief Initializes a provizio_radar_api_context object to handle a single radar
 *
 * @param callback Function to be called on receiving a complete or partial radar point cloud
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param context The provizio_radar_point_cloud_api_context object to initialize
 *
 * @warning radar_position_id of all packets handled by this context must be same
 */
PROVIZIO__EXTERN_C void provizio_radar_api_context_init(provizio_radar_point_cloud_callback point_cloud_callback,
                                                        void *point_cloud_user_data,
                                                        provizio_radar_ego_motion_callback ego_motion_callback,
                                                        void *ego_motion_user_data,
                                                        provizio_radar_api_context *context);

/**
 * @brief Initializes multiple provizio_radar_api_context objects to handle packets from multiple radars
 *
 * @param callback Function to be called on receiving a complete or partial radar point cloud
 * @param user_data Custom argument to be passed to the callback, may be NULL
 * @param contexts Array of num_contexts of provizio_radar_api_context objects to initialize
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle) to initialize
 */
PROVIZIO__EXTERN_C void provizio_radar_api_contexts_init(provizio_radar_point_cloud_callback point_cloud_callback,
                                                         void *point_cloud_user_data,
                                                         provizio_radar_ego_motion_callback ego_motion_callback,
                                                         void *ego_motion_user_data,
                                                         provizio_radar_api_context *contexts,
                                                         size_t num_contexts);

/**
 * @brief Makes provizio_radar_api_context object handle a specific radar, which makes it skip packets
 * intended for other radars
 *
 * @param context provizio_radar_api_context to be assigned
 * @param radar_position_id radar to assign
 * @return 0 in case it was successfully assigned, an error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_radar_api_context_assign(
    provizio_radar_api_context *context, provizio_radar_position radar_position_id);

/**
 * @brief Returns an api_context associated with specified radar_position_id
 *
 * @param contexts Array of num_contexts of provizio_radar_api_context objects to search
 * @param num_contexts Number of contexts (i.e. max numbers of radars to handle) to search
 * @param radar_position_id radar id to search
 * @return context associated with radar_position_id if found, NULL otherwise
 */
PROVIZIO__EXTERN_C provizio_radar_api_context *provizio_get_radar_api_context_by_position_id(
    provizio_radar_api_context *contexts, size_t num_contexts, const uint16_t radar_position_id);

#endif // PROVIZIO_RADAR_API_CONTEXT
