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

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "provizio/socket.h"

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

uint64_t provizio_ntohll(uint64_t value)
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
        return ((uint64_t)ntohl((uint32_t)(value & bits_32_mask)) << bits_32) | // NOLINT // LCOV_EXCL_LINE
               ntohl((uint32_t)(value >> bits_32));                             // NOLINT // LCOV_EXCL_LINE
    }

    return value; // LCOV_EXCL_LINE: host CPU arch dependent
}

uint64_t provizio_htonll(uint64_t value)
{
    // As reversing, when required, is a symmetrical operation
    return provizio_ntohll(value);
}

float provizio_ntohf(float value)
{
    const float test_value = 1.0F;
    if (((const char *)(&test_value))[3] != 0)
    {
        // LCOV_EXCL_START: host CPU arch dependent
        // Byte order needs to be reversed
        uint32_t value_as_uint32_t; // NOLINT: Initialized right below
        assert(sizeof(value_as_uint32_t) == sizeof(value));
        memcpy(&value_as_uint32_t, &value, sizeof(value_as_uint32_t));

        // Clang-tidy has a number of complains regarding the expression below, but we really have to do a pretty
        // non-standard thing here
        value_as_uint32_t =
            ((value_as_uint32_t & 0xff000000) >> 24) | ((value_as_uint32_t & 0x00ff0000) >> 8) | // NOLINT
            ((value_as_uint32_t & 0x0000ff00) << 8) | ((value_as_uint32_t & 0x000000ff) << 24);  // NOLINT

        memcpy(&value, &value_as_uint32_t, sizeof(value_as_uint32_t));
        return value;
        // LCOV_EXCL_STOP
    }

    return value; // LCOV_EXCL_LINE: host CPU arch dependent
}

float provizio_htonf(float value)
{
    // As reversing, when required, is a symmetrical operation
    return provizio_ntohf(value);
}

void provizio_set_protocol_field_uint8_t(uint8_t *field, uint8_t value)
{
    *field = value;
}

void provizio_set_protocol_field_uint16_t(uint16_t *field, uint16_t value)
{
    // linting disabled as htons implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    value = htons(value);              // NOLINT
    PROVIZIO__SET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy
}

void provizio_set_protocol_field_uint32_t(uint32_t *field, uint32_t value)
{
    // linting disabled as htonl implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    value = htonl(value);              // NOLINT
    PROVIZIO__SET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy
}

void provizio_set_protocol_field_uint64_t(uint64_t *field, uint64_t value)
{
    value = provizio_htonll(value);
    PROVIZIO__SET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy
}

void provizio_set_protocol_field_float(float *field, float value)
{
    value = provizio_htonf(value);
    PROVIZIO__SET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy
}

uint8_t provizio_get_protocol_field_uint8_t(const uint8_t *field)
{
    return *field;
}

uint16_t provizio_get_protocol_field_uint16_t(const uint16_t *field)
{
    uint16_t value = 0;
    PROVIZIO__GET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy

    // linting disabled as ntohs implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    return ntohs(value); // NOLINT
}

uint32_t provizio_get_protocol_field_uint32_t(const uint32_t *field)
{
    uint32_t value = 0;
    PROVIZIO__GET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy

    // linting disabled as ntohl implementation is up to a platform (it uses asm instructions in some platforms,
    // which clang-tidy hates)
    return ntohl(value); // NOLINT
}

uint64_t provizio_get_protocol_field_uint64_t(const uint64_t *field)
{
    uint64_t value = 0;
    PROVIZIO__GET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy

    return provizio_ntohll(value);
}

float provizio_get_protocol_field_float(const float *field)
{
    float value = 0;
    PROVIZIO__GET_FIELD(field, value); // NOLINT: no unrolling needed, false positive of clang-tidy

    return provizio_ntohf(value);
}

#ifdef WIN32
int32_t provizio_gettimeofday(struct timeval *out_timeval)
{
    SYSTEMTIME system_time;
    GetSystemTime(&system_time);

    FILETIME file_time;
    SystemTimeToFileTime(&system_time, &file_time);

    const uint64_t time = ((uint64_t)file_time.dwLowDateTime) + (((uint64_t)file_time.dwHighDateTime) << 32);
    out_timeval->tv_sec = (int32_t)(time / 10000000L);
    out_timeval->tv_usec = (int32_t)(system_time.wMilliseconds * 1000);

    return 0;
}
#else
int32_t provizio_gettimeofday(struct timeval *out_timeval)
{
    if (gettimeofday(out_timeval, NULL) != 0)
    {
        return errno != 0 ? errno : -1; // LCOV_EXCL_LINE: Can't be unit-tested as it depends on the state of the OS
    }

    return 0;
}
#endif // WIN32

int64_t provizio_time_interval_ns(struct timeval *time_b, struct timeval *time_a)
{
    const int64_t nanoseconds_in_second = 1000000000LL;
    const int64_t nanoseconds_in_microsecond = 1000LL;

    return ((int64_t)time_b->tv_sec - (int64_t)time_a->tv_sec) * nanoseconds_in_second +
           ((int64_t)time_b->tv_usec - (int64_t)time_a->tv_usec) * nanoseconds_in_microsecond;
}

float provizio_nanoseconds_to_seconds(int64_t duration_ns)
{
    const int64_t nanoseconds_in_millisecond = 1000000LL; // Required due to limited precision of floats
    const float milliseconds_in_second = 1000.0F;

    return (float)(duration_ns / nanoseconds_in_millisecond) / // NOLINT: Potential losing of precision is indeed
                                                               // expected due to limited float
           milliseconds_in_second;
    // capacity
}
