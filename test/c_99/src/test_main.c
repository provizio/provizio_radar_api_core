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

#include "provizio/socket.h"

void setUp(void)
{
    provizio_sockets_initialize();
}

void tearDown(void)
{
    provizio_sockets_deinitialize();
}

int provizio_run_test_common(void);
int provizio_run_test_util(void);
int provizio_run_test_radar_point_cloud(void);

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    int result = 0;
    result = result ? result : provizio_run_test_common();
    result = result ? result : provizio_run_test_util();
    result = result ? result : provizio_run_test_radar_point_cloud();

    return result;
}
