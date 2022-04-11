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

int32_t provizio_sockets_initialize(void)
{
#ifdef _WIN32
    WSADATA wsa_data;
    return (int32_t)WSAStartup(MAKEWORD(1, 1), &wsa_data);
#else
    return 0;
#endif
}

int32_t provizio_sockets_deinitialize(void)
{
#ifdef _WIN32
    return (int32_t)WSACleanup();
#else
    return 0;
#endif
}

int32_t provizio_socket_valid(PROVIZIO__SOCKET sock)
{
    return (int32_t)(sock != PROVIZIO__INVALID_SOCKET);
}

int32_t provizio_socket_close(PROVIZIO__SOCKET sock)
{
#ifdef _WIN32
    return (int32_t)closesocket(sock);
#else
    return (int32_t)close(sock);
#endif
}

int32_t provizio_socket_set_recv_timeout(PROVIZIO__SOCKET sock, uint64_t timeout_ns)
{
    int32_t status;
#ifdef _WIN32
    const uint32_t timeout_ms = (uint32_t)(timeout / 1000000ULL);
    status = (int32_t)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms));
#else
    const uint64_t seconds_to_nanoseconds = 1000000000ULL;
    const uint64_t microseconds_to_nanoseconds = 1000ULL;
    struct timeval tv;
    tv.tv_sec = (time_t)(timeout_ns / seconds_to_nanoseconds);
    tv.tv_usec = (suseconds_t)((timeout_ns % seconds_to_nanoseconds) / microseconds_to_nanoseconds);
    status = (int32_t)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    return status;
}
