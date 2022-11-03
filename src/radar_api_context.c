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

#include "provizio/radar_api/radar_api_context.h"

#include <assert.h>
#include <string.h>
#include <math.h>

#include "provizio/radar_api/errno.h"
#include "provizio/util.h"


void provizio_radar_api_context_init(provizio_radar_point_cloud_callback point_cloud_callback,
                                     void *point_cloud_user_data,
                                     provizio_radar_ego_motion_callback ego_motion_callback,
                                     void *ego_motion_user_data,
                                     provizio_radar_api_context *context)
{
    memset(context, 0, sizeof(provizio_radar_api_context));

    context->point_cloud_callback = point_cloud_callback;
    context->point_cloud_user_data = point_cloud_user_data;
    context->ego_motion_callback = ego_motion_callback;
    context->ego_motion_user_data = ego_motion_user_data;
    context->radar_position_id = provizio_radar_position_unknown;
    context->motion.sensor_velocity_x = nanf("");
    context->motion.sensor_velocity_y = nanf("");
    for (size_t i = 0; i < PROVIZIO__RADAR_POINT_CLOUD_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT; ++i)
    {
        context->impl.point_clouds_being_received[i].sensor_velocity_x = nanf("");
        context->impl.point_clouds_being_received[i].sensor_velocity_y = nanf("");
    }
}

void provizio_radar_api_contexts_init(provizio_radar_point_cloud_callback point_cloud_callback,
                                      void *point_cloud_user_data,
                                      provizio_radar_ego_motion_callback ego_motion_callback,
                                      void *ego_motion_user_data,
                                      provizio_radar_api_context *contexts, size_t num_contexts)
{
#pragma unroll(5)
    for (size_t i = 0; i < num_contexts; ++i)
    {
        provizio_radar_api_context_init(point_cloud_callback, point_cloud_user_data, ego_motion_callback, ego_motion_user_data, &contexts[i]);
    }
}

int32_t provizio_radar_api_context_assign(provizio_radar_api_context *context,
                                          provizio_radar_position radar_position_id)
{
    if (radar_position_id == provizio_radar_position_unknown)
    {
        provizio_error(
            "provizio_radar_api_context_assign: can't assign to provizio_radar_position_unknown");
        return PROVIZIO_E_ARGUMENT;
    }

    if (context->radar_position_id == radar_position_id)
    {
        return 0;
    }

    if (context->radar_position_id == provizio_radar_position_unknown)
    {
        context->radar_position_id = radar_position_id;
        return 0;
    }

    provizio_error("provizio_radar_api_context_assign: already assigned");
    return PROVIZIO_E_NOT_PERMITTED;
}

provizio_radar_api_context *provizio_get_radar_api_context_by_position_id(
    provizio_radar_api_context *contexts, size_t num_contexts, const uint16_t radar_position_id)
{
    assert(radar_position_id != provizio_radar_position_unknown);

#pragma unroll(5)
    for (size_t i = 0; i < num_contexts; ++i)
    {
        if (contexts[i].radar_position_id == radar_position_id)
        {
            // Found the correct context
            return &contexts[i];
        }
    }

    // There is no context for this radar_position_id yet, let's look for a yet unused context
#pragma unroll(5)
    for (size_t i = 0; i < num_contexts; ++i)
    {
        provizio_radar_api_context *context = &contexts[i];
        if (context->radar_position_id == provizio_radar_position_unknown)
        {
            // Found!
            return context;
        }
    }

    // Not found
    provizio_error("provizio_get_radar_api_context_by_position_id: Out of available contexts");
    return NULL;
}

