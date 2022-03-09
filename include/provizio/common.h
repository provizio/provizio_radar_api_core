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

#ifdef __cplusplus
// C++

#include <cstddef>
#include <cstdint>

#ifndef PROVIZIO__NAMESPACE

#define PROVIZIO__NAMESPACE(name) name // Used to define and address types and free functions within the namespace
#define PROVIZIO__NAMESPACE_GLOBAL(name) ::provizio::name // Used to address types and free functions globally

#define PROVIZIO__NAMESPACE_OPEN                                                                                       \
    namespace provizio                                                                                                 \
    {                                                                                                                  \
        extern "C"                                                                                                     \
        {

#define PROVIZIO__NAMESPACE_CLOSE                                                                                      \
    }                                                                                                                  \
    }

#endif // PROVIZIO__NAMESPACE

#else
// C

#include <stddef.h>
#include <stdint.h>

#ifndef PROVIZIO__NAMESPACE

#define PROVIZIO__NAMESPACE(name)                                                                                      \
    provizio_##name // Used to define and address types and free functions within the namespace
#define PROVIZIO__NAMESPACE_GLOBAL(name) PROVIZIO__NAMESPACE(name) // Used to address types and free functions globally

#define PROVIZIO__NAMESPACE_OPEN
#define PROVIZIO__NAMESPACE_CLOSE

#endif // PROVIZIO__NAMESPACE

#endif // __cplusplus

#ifndef PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES
// Standard MTU is 1500 bytes, UDP+IP packet header is 28 bytes which leaves 1472 for payload
#define PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES ((size_t)1472)
#endif // PROVIZIO__MAX_PAYLOAD_PER_UDP_PACKET_BYTES

#endif // PROVIZIO_COMMON
