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

#include "unity/unity.h"

#include "provizio/socket.h"

void setUp(void)
{
    provizio_sockets_initialize();
}

void tearDown(void)
{
    provizio_sockets_deinitialize();
}

void provizio_run_test_common(void);
void provizio_run_test_util(void);
void provizio_run_test_radar_point_cloud(void);
void provizio_run_test_core(void);

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(provizio_run_test_common);
    RUN_TEST(provizio_run_test_util);
    RUN_TEST(provizio_run_test_radar_point_cloud);

    return UNITY_END();
}
