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

#include "provizio/common.h"

#define PROVIZIO__TEST_MAX_MESSAGE_LENGTH 1024
char provizio_test_warning[PROVIZIO__TEST_MAX_MESSAGE_LENGTH]; // NOLINT: non-const global by design
char provizio_test_error[PROVIZIO__TEST_MAX_MESSAGE_LENGTH];   // NOLINT: non-const global by design

static void test_provizio_common_on_warning(const char *warning)
{
    strncpy(provizio_test_warning, warning, PROVIZIO__TEST_MAX_MESSAGE_LENGTH - 1);
}

static void test_provizio_common_on_error(const char *error)
{
    strncpy(provizio_test_error, error, PROVIZIO__TEST_MAX_MESSAGE_LENGTH - 1);
}

static void test_provizio_warnings(void)
{
    // Check it doen't crash
    provizio_warning("test_warning");

    // Custom handler
    provizio_set_on_warning(&test_provizio_common_on_warning);
    provizio_warning("test_warning_2");
    TEST_ASSERT_EQUAL_STRING("test_warning_2", provizio_test_warning);

    // Reset to default
    provizio_set_on_warning(NULL);
    provizio_warning("test_warning_3");
    TEST_ASSERT_EQUAL_STRING("test_warning_2", provizio_test_warning); // Check it didn't change
}

static void test_provizio_errors(void)
{
    // Check it doen't crash
    provizio_error("test_error");

    // Custom handler
    provizio_set_on_error(&test_provizio_common_on_error);
    provizio_error("test_error_2");
    TEST_ASSERT_EQUAL_STRING("test_error_2", provizio_test_error);

    // Reset to default
    provizio_set_on_error(NULL);
    provizio_error("test_error_3");
    TEST_ASSERT_EQUAL_STRING("test_error_2", provizio_test_error); // Check it didn't change
}

int provizio_run_test_common(void)
{
    memset(provizio_test_warning, 0, sizeof(provizio_test_warning));
    memset(provizio_test_error, 0, sizeof(provizio_test_error));

    UNITY_BEGIN();

    RUN_TEST(test_provizio_warnings);
    RUN_TEST(test_provizio_errors);

    return UNITY_END();
}
