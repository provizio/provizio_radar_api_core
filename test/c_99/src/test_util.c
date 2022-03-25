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

#include <string.h>

#include "unity/unity.h"

#include "provizio/util.h"

void test_provizio_set_protocol_field_uint8_t(void)
{
    uint8_t test_buffer[3];
    memset(test_buffer, 0, sizeof(test_buffer));

    provizio_set_protocol_field_uint8_t(&test_buffer[0], (uint8_t)0x01);
    TEST_ASSERT_EQUAL_UINT8(0x01, test_buffer[0]);

    provizio_set_protocol_field_uint8_t(&test_buffer[1], (uint8_t)0x23);
    TEST_ASSERT_EQUAL_UINT8(0x23, test_buffer[1]);

    provizio_set_protocol_field_uint8_t(&test_buffer[2], (uint8_t)0xef);
    TEST_ASSERT_EQUAL_UINT8(0xef, test_buffer[2]);
}

void test_provizio_set_protocol_field_uint16_t(void)
{
    uint8_t test_buffer[3];
    memset(test_buffer, 0, sizeof(test_buffer));

    // Aligned write
    provizio_set_protocol_field_uint16_t((uint16_t *)&test_buffer[0], (uint16_t)0x0123);
    TEST_ASSERT_EQUAL_UINT8(0x01, test_buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x23, test_buffer[1]);

    // Unaligned write
    provizio_set_protocol_field_uint16_t((uint16_t *)&test_buffer[1], (uint16_t)0xbcde);
    TEST_ASSERT_EQUAL_UINT8(0xbc, test_buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0xde, test_buffer[2]);
}

void test_provizio_set_protocol_field_uint32_t(void)
{
    uint8_t test_buffer[5];
    memset(test_buffer, 0, sizeof(test_buffer));

    // Aligned write
    provizio_set_protocol_field_uint32_t((uint32_t *)&test_buffer[0], (uint32_t)0x01234567);
    TEST_ASSERT_EQUAL_UINT8(0x01, test_buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x23, test_buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0x45, test_buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(0x67, test_buffer[3]);

    // Unaligned write
    provizio_set_protocol_field_uint32_t((uint32_t *)&test_buffer[1], (uint32_t)0x89abcdef);
    TEST_ASSERT_EQUAL_UINT8(0x89, test_buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0xab, test_buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(0xcd, test_buffer[3]);
    TEST_ASSERT_EQUAL_UINT8(0xef, test_buffer[4]);
}

void test_provizio_set_protocol_field_uint64_t(void)
{
    uint8_t test_buffer[9];
    memset(test_buffer, 0, sizeof(test_buffer));

    // Aligned write
    provizio_set_protocol_field_uint64_t((uint64_t *)&test_buffer[0], (uint64_t)0x0123456789abcdef);
    TEST_ASSERT_EQUAL_UINT8(0x01, test_buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x23, test_buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0x45, test_buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(0x67, test_buffer[3]);
    TEST_ASSERT_EQUAL_UINT8(0x89, test_buffer[4]);
    TEST_ASSERT_EQUAL_UINT8(0xab, test_buffer[5]);
    TEST_ASSERT_EQUAL_UINT8(0xcd, test_buffer[6]);
    TEST_ASSERT_EQUAL_UINT8(0xef, test_buffer[7]);

    // Unaligned write
    provizio_set_protocol_field_uint64_t((uint64_t *)&test_buffer[1], (uint64_t)0x0123456789abcdef);
    TEST_ASSERT_EQUAL_UINT8(0x01, test_buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0x23, test_buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(0x45, test_buffer[3]);
    TEST_ASSERT_EQUAL_UINT8(0x67, test_buffer[4]);
    TEST_ASSERT_EQUAL_UINT8(0x89, test_buffer[5]);
    TEST_ASSERT_EQUAL_UINT8(0xab, test_buffer[6]);
    TEST_ASSERT_EQUAL_UINT8(0xcd, test_buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(0xef, test_buffer[8]);
}

void test_provizio_set_protocol_field_float(void)
{
    uint8_t test_buffer[5];
    memset(test_buffer, 0, sizeof(test_buffer));

    const float test_float = 12.345f;

    // Aligned write
    provizio_set_protocol_field_float((float *)&test_buffer[0], test_float);
    float to_float = 0.0f;
    memcpy(&to_float, &test_buffer[0], sizeof(float));
    TEST_ASSERT_EQUAL_FLOAT(test_float, to_float);

    // Unaligned write
    provizio_set_protocol_field_float((float *)&test_buffer[1], test_float);
    to_float = 0.0f;
    memcpy(&to_float, &test_buffer[1], sizeof(float));
    TEST_ASSERT_EQUAL_FLOAT(test_float, to_float);
}

void test_provizio_get_protocol_field_uint8_t(void)
{
    uint8_t test_buffer[3] = {0x01, 0x23, 0x45};
    TEST_ASSERT_EQUAL_UINT8(0x01, provizio_get_protocol_field_uint8_t(&test_buffer[0]));
    TEST_ASSERT_EQUAL_UINT8(0x23, provizio_get_protocol_field_uint8_t(&test_buffer[1]));
    TEST_ASSERT_EQUAL_UINT8(0x45, provizio_get_protocol_field_uint8_t(&test_buffer[2]));
}

void test_provizio_get_protocol_field_uint16_t(void)
{
    uint8_t test_buffer[3] = {0x01, 0x23, 0x45};

    // Aligned read
    TEST_ASSERT_EQUAL_UINT16((uint16_t)0x0123, provizio_get_protocol_field_uint16_t((uint16_t *)&test_buffer[0]));

    // Unaligned read
    TEST_ASSERT_EQUAL_UINT16((uint16_t)0x2345, provizio_get_protocol_field_uint16_t((uint16_t *)&test_buffer[1]));
}

void test_provizio_get_protocol_field_uint32_t(void)
{
    uint8_t test_buffer[5] = {0x01, 0x23, 0x45, 0x67, 0x89};

    // Aligned read
    TEST_ASSERT_EQUAL_UINT32((uint32_t)0x01234567, provizio_get_protocol_field_uint32_t((uint32_t *)&test_buffer[0]));

    // Unaligned read
    TEST_ASSERT_EQUAL_UINT32((uint32_t)0x23456789, provizio_get_protocol_field_uint32_t((uint32_t *)&test_buffer[1]));
}

void test_provizio_get_protocol_field_uint64_t(void)
{
    uint8_t test_buffer[9] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x1f};

    // Aligned read
    TEST_ASSERT_EQUAL_UINT64((uint64_t)0x0123456789abcdef,
                             provizio_get_protocol_field_uint64_t((uint64_t *)&test_buffer[0]));

    // Unaligned read
    TEST_ASSERT_EQUAL_UINT64((uint64_t)0x23456789abcdef1f,
                             provizio_get_protocol_field_uint64_t((uint64_t *)&test_buffer[1]));
}

void test_provizio_get_protocol_field_float(void)
{
    uint8_t test_buffer[5];
    memset(test_buffer, 0, sizeof(test_buffer));

    const float test_float = 123.456f;

    // Aligned read
    memcpy(&test_buffer[0], &test_float, sizeof(float));
    TEST_ASSERT_EQUAL_FLOAT(test_float, provizio_get_protocol_field_float((float *)&test_buffer[0]));

    // Unaligned read
    memcpy(&test_buffer[1], &test_float, sizeof(float));
    TEST_ASSERT_EQUAL_FLOAT(test_float, provizio_get_protocol_field_float((float *)&test_buffer[1]));
}

void provizio_run_test_util(void)
{
    test_provizio_set_protocol_field_uint8_t();
    test_provizio_set_protocol_field_uint16_t();
    test_provizio_set_protocol_field_uint32_t();
    test_provizio_set_protocol_field_uint64_t();
    test_provizio_set_protocol_field_float();
    test_provizio_get_protocol_field_uint8_t();
    test_provizio_get_protocol_field_uint16_t();
    test_provizio_get_protocol_field_uint32_t();
    test_provizio_get_protocol_field_uint64_t();
    test_provizio_get_protocol_field_float();
}
