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
char provizio_test_common_warning[PROVIZIO__TEST_MAX_MESSAGE_LENGTH];
char provizio_test_common_error[PROVIZIO__TEST_MAX_MESSAGE_LENGTH];

void test_provizio_common_on_warning(const char *warning)
{
    strcpy(provizio_test_common_warning, warning);
}

void test_provizio_common_on_error(const char *error)
{
    strcpy(provizio_test_common_error, error);
}

void test_provizio_warnings(void)
{
    // Check it doen't crash
    provizio_warning("test_warning");

    // Custom handler
    provizio_set_on_warning(&test_provizio_common_on_warning);
    provizio_warning("test_warning_2");
    TEST_ASSERT_EQUAL_STRING("test_warning_2", provizio_test_common_warning);

    // Reset to default
    provizio_set_on_warning(NULL);
    provizio_warning("test_warning_3");
    TEST_ASSERT_EQUAL_STRING("test_warning_2", provizio_test_common_warning); // Check it didn't change
}

void test_provizio_errors(void)
{
    // Check it doen't crash
    provizio_error("test_error");

    // Custom handler
    provizio_set_on_error(&test_provizio_common_on_error);
    provizio_error("test_error_2");
    TEST_ASSERT_EQUAL_STRING("test_error_2", provizio_test_common_error);

    // Reset to default
    provizio_set_on_error(NULL);
    provizio_error("test_error_3");
    TEST_ASSERT_EQUAL_STRING("test_error_2", provizio_test_common_error); // Check it didn't change
}

void provizio_run_test_common(void)
{
    memset(provizio_test_common_warning, 0, sizeof(provizio_test_common_warning));
    memset(provizio_test_common_error, 0, sizeof(provizio_test_common_error));

    test_provizio_warnings();
    test_provizio_errors();
}
