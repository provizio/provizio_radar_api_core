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

#ifndef PROVIZIO_SOCKET
#define PROVIZIO_SOCKET

#include "provizio/common.h"

#ifdef _WIN32

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif // _WIN32_WINNT
#include <Ws2tcpip.h>
#include <winsock2.h>

#define PROVIZIO__SOCKET SOCKET
#define PROVIZIO__INVALID_SOCKET INVALID_SOCKET
#define PROVIZIO__RECV_RETURN_TYPE int

#else

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#define PROVIZIO__SOCKET int
#define PROVIZIO__INVALID_SOCKET ((int)-1)
#define PROVIZIO__RECV_RETURN_TYPE ssize_t

#endif

/**
 * @brief Enables sockets operations - to be called once prior to any other API calls
 *
 * @return 0 if successfull, non-zero error code otherwise
 * @note Required in Windows, unless WSAStartup is called somewhere else, can be omitted in other platforms
 */
PROVIZIO__EXTERN_C int32_t provizio_sockets_initialize(void);

/**
 * @brief Terminates sockets operations - to be called once after all the other API calls
 *
 * @return 0 if successfull, non-zero error code otherwise
 * @note Required in Windows, unless WSACleanup is called somewhere else, can be omitted in other platforms
 */
PROVIZIO__EXTERN_C int32_t provizio_sockets_deinitialize(void);

/**
 * @brief Checks if `socket` returned a valid socket object
 *
 * @param sock `socket`-returned socket object
 * @return nonzero if valid, 0 otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_socket_valid(PROVIZIO__SOCKET sock);

/**
 * @brief Closes a previously opened socket
 *
 * @param sock `socket`-returned socket object
 * @return 0 if successfull, non-zero error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_socket_close(PROVIZIO__SOCKET sock);

/**
 * @brief Sets a timeout for recv operations on a previously opened socket
 *
 * @param timeout_ns Timeout in nanoseconds
 * @return 0 if successfull, error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_socket_set_recv_timeout(PROVIZIO__SOCKET sock, uint64_t timeout_ns);

#endif // PROVIZIO_SOCKET
