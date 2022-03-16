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

#ifndef PROVIZIO_RADAR_API_COMMON
#define PROVIZIO_RADAR_API_COMMON

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

enum provizio_radar_position
{
    provizio_radar_position_front_center = 0,
    provizio_radar_position_front_left = 1,
    provizio_radar_position_front_right = 2,
    provizio_radar_position_rear_left = 3,
    provizio_radar_position_rear_right = 4,

    provizio_radar_position_custom = ((uint16_t)0x1000),

    provizio_radar_position_unknown = ((uint16_t)0xffff),
    provizio_radar_position_max = provizio_radar_position_unknown
};

#endif // PROVIZIO_RADAR_API_COMMON
