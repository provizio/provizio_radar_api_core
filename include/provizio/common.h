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

#ifndef PROVIZIO_COMMON
#define PROVIZIO_COMMON

#ifndef _USE_MATH_DEFINES
// Required in MSVC for constants like M_PI
#define _USE_MATH_DEFINES
#endif // _USE_MATH_DEFINES

#ifdef __cplusplus
// C++

#include <cstddef>
#include <cstdint>

#ifndef PROVIZIO__EXTERN_C
#define PROVIZIO__EXTERN_C extern "C"
#endif // PROVIZIO__EXTERN_C

#else
// C

#include <stddef.h>
#include <stdint.h>

#ifndef PROVIZIO__EXTERN_C
#define PROVIZIO__EXTERN_C
#endif // PROVIZIO__EXTERN_C

#endif // __cplusplus

#ifndef PROVIZIO__MTU
// Maximum Transmission Unit (bytes), normally 1500 for Ethernet
#define PROVIZIO__MTU ((size_t)1500)
#endif // PROVIZIO__MTU

#ifndef PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES
// UDP+IP packet header is 28 bytes which leaves MTU (1500 normally) - 28 = 1472 for payload
#define PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES (PROVIZIO__MTU - (size_t)28)
#endif // PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES

/**
 * @brief Specifies a custom function to be called on warning
 *
 * @param warning_function Function pointer or NULL (resets to default)
 * @warning Not thread safe, so it's recommended to call prior to starting any threads
 * @note By default printing to stderr is used on warning.
 */
PROVIZIO__EXTERN_C void provizio_set_on_warning(void (*warning_function)(const char *));

/**
 * @brief Specifies a custom function to be called on error
 *
 * @param warning_function Function pointer or NULL (resets to default)
 * @warning Not thread safe, so it's recommended to call prior to starting any threads
 * @note By default printing to stderr is used on error.
 */
PROVIZIO__EXTERN_C void provizio_set_on_error(void (*error_function)(const char *));

/**
 * @brief Informs about a warning
 *
 * @param message Warning message
 * @see provizio_set_on_warning for more details
 * @return PROVIZIO__EXTERN_C
 */
PROVIZIO__EXTERN_C void provizio_warning(const char *message);

/**
 * @brief Informs about an error
 *
 * @param message Error message
 * @see provizio_set_on_error for more details
 * @return PROVIZIO__EXTERN_C
 */
PROVIZIO__EXTERN_C void provizio_error(const char *message);

#endif // PROVIZIO_COMMON
