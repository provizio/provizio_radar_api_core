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

#ifndef PROVIZIO_UTIL
#define PROVIZIO_UTIL

#include <time.h>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#else
#include <sys/time.h>
#endif

#include "provizio/common.h"

/**
 * @brief Sets a new uint8_t value to the specified protocol field using network byte order, regardless of the field
 * alignment
 *
 * @param field Protocol field to be updated using network byte order
 * @param value New value using host byte order to be set
 */
PROVIZIO__EXTERN_C void provizio_set_protocol_field_uint8_t(uint8_t *field, uint8_t value);

/**
 * @brief Sets a new uint16_t value to the specified protocol field using network byte order, regardless of the field
 * alignment
 *
 * @param field Protocol field to be updated using network byte order
 * @param value New value using host byte order to be set
 */
PROVIZIO__EXTERN_C void provizio_set_protocol_field_uint16_t(uint16_t *field, uint16_t value);

/**
 * @brief Sets a new uint32_t value to the specified protocol field using network byte order, regardless of the field
 * alignment
 *
 * @param field Protocol field to be updated using network byte order
 * @param value New value using host byte order to be set
 */
PROVIZIO__EXTERN_C void provizio_set_protocol_field_uint32_t(uint32_t *field, uint32_t value);

/**
 * @brief Sets a new uint64_t value to the specified protocol field using network byte order, regardless of the field
 * alignment
 *
 * @param field Protocol field to be updated using network byte order
 * @param value New value using host byte order to be set
 */
PROVIZIO__EXTERN_C void provizio_set_protocol_field_uint64_t(uint64_t *field, uint64_t value);

/**
 * @brief Sets a new float value to the specified protocol field using network byte order, regardless of the field
 * alignment
 *
 * @param field Protocol field to be updated using network byte order
 * @param value New value using host byte order to be set
 */
PROVIZIO__EXTERN_C void provizio_set_protocol_field_float(float *field, float value);

/**
 * @brief Retrieves a uint8_t value using host byte order from a protocol field, regardless of the field alignment
 *
 * @param field Protocol field using network byte order to be retrieved
 * @return uint8_t value using host byte order
 */
PROVIZIO__EXTERN_C uint8_t provizio_get_protocol_field_uint8_t(const uint8_t *field);

/**
 * @brief Retrieves a uint16_t value using host byte order from a protocol field, regardless of the field alignment
 *
 * @param field Protocol field using network byte order to be retrieved
 * @return uint16_t value using host byte order
 */
PROVIZIO__EXTERN_C uint16_t provizio_get_protocol_field_uint16_t(const uint16_t *field);

/**
 * @brief Retrieves a uint32_t value using host byte order from a protocol field, regardless of the field alignment
 *
 * @param field Protocol field using network byte order to be retrieved
 * @return uint32_t value using host byte order
 */
PROVIZIO__EXTERN_C uint32_t provizio_get_protocol_field_uint32_t(const uint32_t *field);

/**
 * @brief Retrieves a uint64_t value using host byte order from a protocol field, regardless of the field alignment
 *
 * @param field Protocol field using network byte order to be retrieved
 * @return uint64_t value using host byte order
 */
PROVIZIO__EXTERN_C uint64_t provizio_get_protocol_field_uint64_t(const uint64_t *field);

/**
 * @brief Retrieves a float value using host byte order from a protocol field, regardless of the field alignment
 *
 * @param field Protocol field using network byte order to be retrieved
 * @return float value using host byte order
 */
PROVIZIO__EXTERN_C float provizio_get_protocol_field_float(const float *field);

/**
 * @brief Like *nix gettimeofday with timezone argument set to NULL, to measure time intervals
 *
 * @param out_timeval Output timeval
 * @return 0 in case of a success, an error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_gettimeofday(struct timeval *out_timeval);

/**
 * @brief Measures interval of time_b - time_a in nanoseconds
 *
 * @param time_b "Newer" time
 * @param time_a "Older" time
 * @return Time difference measured in nanoseconds (can be negative)
 */
PROVIZIO__EXTERN_C int64_t provizio_time_interval_ns(struct timeval *time_b, struct timeval *time_a);

#endif // PROVIZIO_UTIL
