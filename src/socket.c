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

int provizio_sockets_initialize(void)
{
#ifdef _WIN32
    WSADATA wsa_data;
    return WSAStartup(MAKEWORD(1, 1), &wsa_data);
#else
    return 0;
#endif
}

int provizio_sockets_deinitialize(void)
{
#ifdef _WIN32
    return WSACleanup();
#else
    return 0;
#endif
}

int provizio_socket_close(PROVIZIO__SOCKET sock)
{
#ifdef _WIN32
    return closesocket(sock);
#else
    return close(sock);
#endif
}

bool provizio_socket_valid(PROVIZIO__SOCKET sock)
{
#ifdef _WIN32
    return sock != INVALID_SOCKET;
#else
    return sock != -1;
#endif
}
