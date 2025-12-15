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

#include "provizio/radar_api/radar_points_accumulation_types.h"

#include "unity/unity.h"

void test_provizio_enu_distance(void)
{
    {
        const provizio_enu_position a = {10.0F, 0.0F, 0.0F};           // NOLINT
        const provizio_enu_position b = {100.0F, 0.0F, 0.0F};          // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(90.0F, provizio_enu_distance(&a, &b)); // NOLINT
    }

    {
        const provizio_enu_position a = {0.0F, 5.0F, 0.0F};           // NOLINT
        const provizio_enu_position b = {0.0F, 2.0F, 0.0F};           // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(3.0F, provizio_enu_distance(&a, &b)); // NOLINT
    }

    {
        const provizio_enu_position a = {0.0F, 0.0F, -10000.0F};          // NOLINT
        const provizio_enu_position b = {0.0F, 0.0F, 0.0F};               // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(10000.0F, provizio_enu_distance(&a, &b)); // NOLINT
    }

    {
        const provizio_enu_position a = {1.0F, 2.0F, 3.0F};                    // NOLINT
        const provizio_enu_position b = {10.0F, 9.0F, 8.0F};                   // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(sqrtf(155.0F), provizio_enu_distance(&a, &b)); // NOLINT
    }

    {
        const provizio_enu_position a = {10.0F, 200.0F, 900.0F};      // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(0.0F, provizio_enu_distance(&a, &a)); // NOLINT
    }
}

int provizio_run_test_radar_points_accumulation_types(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_provizio_enu_distance);

    return UNITY_END();
}
