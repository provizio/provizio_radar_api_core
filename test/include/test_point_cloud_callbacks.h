#ifndef INCLUDE_TEST_POINT_CLOUD_CALLBACKS
#define INCLUDE_TEST_POINT_CLOUD_CALLBACKS

#include "provizio/radar_api/radar_point_cloud.h"

#define PROVIZIO__TEST_CALLBACK_DATA_NUM_POINT_CLOUDS 4
typedef struct test_provizio_radar_point_cloud_callback_data // NOLINT: it's aligned exactly as it's supposed to
{
    int32_t called_times;
    provizio_radar_point_cloud last_point_clouds[PROVIZIO__TEST_CALLBACK_DATA_NUM_POINT_CLOUDS];
} test_provizio_radar_point_cloud_callback_data;

void test_provizio_radar_point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
                                              provizio_radar_point_cloud_api_context *context);

#endif // INCLUDE_TEST_POINT_CLOUD_CALLBACKS
