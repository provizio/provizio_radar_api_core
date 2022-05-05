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

#include "provizio/common.h"

#include <stdio.h>

static void (*provizio_warning_function)(const char *) = NULL; // NOLINT: non-const global by design
static void (*provizio_error_function)(const char *) = NULL;   // NOLINT: non-const global by design

void provizio_set_on_warning(void (*warning_function)(const char *))
{
    provizio_warning_function = warning_function;
}

void provizio_set_on_error(void (*error_function)(const char *))
{
    provizio_error_function = error_function;
}

void provizio_print_message(FILE *stream, const char *message_type, const char *message,
                            void (*handler_function)(const char *))
{
    if (handler_function)
    {
        handler_function(message);
    }
    else
    {
        (void)fprintf(stream, "[provizio_radar_api_core %s] %s\n", message_type, message);
    }
}

void provizio_warning(const char *message)
{
    provizio_print_message(stderr, "warning", message, provizio_warning_function);
}

void provizio_error(const char *message)
{
    provizio_print_message(stderr, "error", message, provizio_error_function);
}
