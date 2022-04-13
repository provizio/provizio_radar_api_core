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
            *(FIELD) = (VALUE);                                                                                        \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            memcpy((FIELD), &(VALUE), sizeof(*(FIELD)));                                                               \
        }                                                                                                              \
    } while (0)

#define PROVIZIO__GET_FIELD(FIELD, VALUE)                                                                              \
    do                                                                                                                 \
    {                                                                                                                  \
        if (PROVIZIO__IS_ALIGNED(FIELD))                                                                               \
        {                                                                                                              \
            (VALUE) = *(FIELD);                                                                                        \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            memcpy(&(VALUE), (FIELD), sizeof(VALUE));                                                                  \
        }                                                                                                              \
    } while (0)

uint64_t provizio_ntohll(uint64_t v)
{
    const size_t bits_32 = 32;
    const uint64_t bits_32_mask = 0xFFFFFFFF;

    const int test_value = 1;
    if (*(const char *)(&test_value) == test_value)
    {
        // Byte order needs to be reversed
        // lcov excluded as it's host CPU arch dependent
        // linting disabled as ntohl implementation is up to a platform (it uses asm instructions in some platforms,
        // which clang-tidy hates)
        return ((uint64_t)ntohl((uint32_t)(v & bits_32_mask)) << bits_32) | // NOLINT // LCOV_EXCL_LINE
               ntohl((uint32_t)(v >> bits_32));                             // NOLINT // LCOV_EXCL_LINE
    }

    return v; // LCOV_EXCL_LINE: host CPU arch dependent
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
    // linting disabled as htons implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    value = htons(value); // NOLINT
    PROVIZIO__SET_FIELD(field, value);
}

void provizio_set_protocol_field_uint32_t(uint32_t *field, uint32_t value)
{
    // linting disabled as htonl implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    value = htonl(value); // NOLINT
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
    uint16_t value = 0;
    PROVIZIO__GET_FIELD(field, value);

    // linting disabled as ntohs implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    return ntohs(value); // NOLINT
}

uint32_t provizio_get_protocol_field_uint32_t(const uint32_t *field)
{
    uint32_t value = 0;
    PROVIZIO__GET_FIELD(field, value);

    // linting disabled as ntohl implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    return ntohl(value); // NOLINT
}

uint64_t provizio_get_protocol_field_uint64_t(const uint64_t *field)
{
    uint64_t value = 0;
    PROVIZIO__GET_FIELD(field, value);

    return provizio_ntohll(value);
}

float provizio_get_protocol_field_float(const float *field)
{
    float value = 0;
    PROVIZIO__GET_FIELD(field, value);

    return value;
}
