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

#include <linmath.h>

void test_provizio_quaternion_set_identity(void)
{
    provizio_quaternion quaternion = {2.0F, 3.0F, 4.0F, 5.0F}; // NOLINT Some "garbage" to make sure it gets overwritten
    provizio_quaternion_set_identity(&quaternion);

    TEST_ASSERT_EQUAL_FLOAT(1.0F, quaternion.w); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(0.0F, quaternion.x); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(0.0F, quaternion.y); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(0.0F, quaternion.z); // NOLINT

    vec3 in_vec = {10.0F, 100.0F, 1000.0F}; // NOLINT
    quat in_quat = {quaternion.x, quaternion.y, quaternion.z, quaternion.w};
    vec3 out_vec = {0, 0, 0};
    quat_mul_vec3(out_vec, in_quat, in_vec);
    TEST_ASSERT_EQUAL_FLOAT(in_vec[0], out_vec[0]); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(in_vec[1], out_vec[1]); // NOLINT
    TEST_ASSERT_EQUAL_FLOAT(in_vec[2], out_vec[2]); // NOLINT
}

void test_provizio_quaternion_set_euler_angles(void)
{
    provizio_quaternion quaternion = {2.0F, 3.0F, 4.0F, 5.0F}; // NOLINT Some "garbage" to make sure it gets overwritten

    vec3 in_vec = {5.0F, 10.0F, 30.0F}; // NOLINT

    // Rotate around X
    {
        provizio_quaternion_set_euler_angles((float)M_PI_2, 0.0F, 0.0F, &quaternion);
        quat in_quat = {quaternion.x, quaternion.y, quaternion.z, quaternion.w};
        vec3 out_vec = {0, 0, 0};
        quat_mul_vec3(out_vec, in_quat, in_vec);
        TEST_ASSERT_EQUAL_FLOAT(5.0F, out_vec[0]);   // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-30.0F, out_vec[1]); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(10.0F, out_vec[2]);  // NOLINT
    }

    // Rotate around Y
    {
        provizio_quaternion_set_euler_angles(0.0F, (float)M_PI_2, 0.0F, &quaternion);
        quat in_quat = {quaternion.x, quaternion.y, quaternion.z, quaternion.w};
        vec3 out_vec = {0, 0, 0};
        quat_mul_vec3(out_vec, in_quat, in_vec);
        TEST_ASSERT_EQUAL_FLOAT(30.0F, out_vec[0]); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(10.0F, out_vec[1]); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(-5.0F, out_vec[2]); // NOLINT
    }

    // Rotate around Z
    {
        provizio_quaternion_set_euler_angles(0.0F, 0.0F, (float)M_PI_2, &quaternion);
        quat in_quat = {quaternion.x, quaternion.y, quaternion.z, quaternion.w};
        vec3 out_vec = {0, 0, 0};
        quat_mul_vec3(out_vec, in_quat, in_vec);
        TEST_ASSERT_EQUAL_FLOAT(-10.0F, out_vec[0]); // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(5.0F, out_vec[1]);   // NOLINT
        TEST_ASSERT_EQUAL_FLOAT(30.0F, out_vec[2]);  // NOLINT
    }
}

void test_provizio_quaternion_is_valid_rotation(void)
{
    // Invalid: zero
    {
        const provizio_quaternion quaternion = {0, 0, 0, 0};
        TEST_ASSERT_EQUAL(0, provizio_quaternion_is_valid_rotation(&quaternion));
    }

    // Invalid: garbage
    {
        const provizio_quaternion quaternion = {2.0F, 3.0F, 4.0F, 5.0F};
        TEST_ASSERT_EQUAL(0, provizio_quaternion_is_valid_rotation(&quaternion));
    }

    // Valid: Identity
    {
        provizio_quaternion quaternion = {2.0F, 3.0F, 4.0F, 5.0F}; // NOLINT
        provizio_quaternion_set_identity(&quaternion);
        TEST_ASSERT_NOT_EQUAL(0, provizio_quaternion_is_valid_rotation(&quaternion));
    }

    // Valid: Rotation
    {
        provizio_quaternion quaternion = {2.0F, 3.0F, 4.0F, 5.0F}; // NOLINT
        provizio_quaternion_set_euler_angles((float)M_PI_2, (float)M_PI_4, (float)M_E, &quaternion);
        TEST_ASSERT_NOT_EQUAL(0, provizio_quaternion_is_valid_rotation(&quaternion));
    }
}

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

    RUN_TEST(test_provizio_quaternion_set_identity);
    RUN_TEST(test_provizio_quaternion_set_euler_angles);
    RUN_TEST(test_provizio_quaternion_is_valid_rotation);
    RUN_TEST(test_provizio_enu_distance);

    return UNITY_END();
}
