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

#ifndef PROVIZIO_RADAR_API_ERRNO
#define PROVIZIO_RADAR_API_ERRNO

#include <errno.h>

#define PROVIZIO_E_TIMEOUT EAGAIN        // Operation timed out
#define PROVIZIO_E_SKIPPED ERANGE        // Packet skipped and ignored
#define PROVIZIO_E_OUT_OF_CONTEXTS EBUSY // Not enough contexts
#define PROVIZIO_E_PROTOCOL EPROTO       // Protocol error
#define PROVIZIO_E_ARGUMENT EINVAL       // Invalid argument
#define PROVIZIO_E_NOT_PERMITTED EPERM   // Operation not permitted

#endif // PROVIZIO_RADAR_API_ERRNO
