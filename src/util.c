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

#include "provizio/util.h"
#include "provizio/socket.h"

#include <string.h>

#define PROVIZIO__IS_ALIGNED(P) (((uintptr_t)(const void *)(P)) % sizeof(*(P)) == 0)

#define PROVIZIO__SET_FIELD(FIELD, VALUE)                                                                              \
    do                                                                                                                 \
    {                                                                                                                  \
        if (PROVIZIO__IS_ALIGNED(FIELD))                                                                               \
        {                                                                                                              \
            *FIELD = VALUE;                                                                                            \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            memcpy(FIELD, &VALUE, sizeof(*FIELD));                                                                     \
        }                                                                                                              \
    } while (0)

#define PROVIZIO__GET_FIELD(FIELD, VALUE)                                                                              \
    do                                                                                                                 \
    {                                                                                                                  \
        if (PROVIZIO__IS_ALIGNED(FIELD))                                                                               \
        {                                                                                                              \
            VALUE = *FIELD;                                                                                            \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            memcpy(&VALUE, FIELD, sizeof(VALUE));                                                                      \
        }                                                                                                              \
    } while (0)

uint64_t provizio_ntohll(uint64_t v)
{
    const int test_value = 1;
    if (*(const char *)(&test_value) == test_value)
    {
        // Byte order needs to be reversed
        return ((uint64_t)ntohl(v & 0xFFFFFFFF) << 32) | ntohl(v >> 32); // LCOV_EXCL_LINE: host CPU arch dependent
    }
    else
    {
        return v; // LCOV_EXCL_LINE: host CPU arch dependent
    }
}

uint64_t provizio_htonll(uint64_t v)
{
    // As reversing, when required, is a symmetrical operation
    return provizio_ntohll(v);
}

void provizio_set_protocol_field_uint8_t(uint8_t *field, uint8_t value)
{
    *field = value;
}

void provizio_set_protocol_field_uint16_t(uint16_t *field, uint16_t value)
{
    value = htons(value);
    PROVIZIO__SET_FIELD(field, value);
}

void provizio_set_protocol_field_uint32_t(uint32_t *field, uint32_t value)
{
    value = htonl(value);
    PROVIZIO__SET_FIELD(field, value);
}

void provizio_set_protocol_field_uint64_t(uint64_t *field, uint64_t value)
{
    value = provizio_htonll(value);
    PROVIZIO__SET_FIELD(field, value);
}

void provizio_set_protocol_field_float(float *field, float value)
{
    PROVIZIO__SET_FIELD(field, value);
}

uint8_t provizio_get_protocol_field_uint8_t(const uint8_t *field)
{
    return *field;
}

uint16_t provizio_get_protocol_field_uint16_t(const uint16_t *field)
{
    uint16_t value;
    PROVIZIO__GET_FIELD(field, value);
    return ntohs(value);
}

uint32_t provizio_get_protocol_field_uint32_t(const uint32_t *field)
{
    uint32_t value;
    PROVIZIO__GET_FIELD(field, value);
    return ntohl(value);
}

uint64_t provizio_get_protocol_field_uint64_t(const uint64_t *field)
{
    uint64_t value;
    PROVIZIO__GET_FIELD(field, value);
    return provizio_ntohll(value);
}

float provizio_get_protocol_field_float(const float *field)
{
    float value;
    PROVIZIO__GET_FIELD(field, value);
    return value;
}
