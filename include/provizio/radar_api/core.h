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

#ifndef PROVIZIO_RADAR_API_CORE
#define PROVIZIO_RADAR_API_CORE

#include "provizio/common.h"
#include "provizio/radar_api/entities.h"
#include "provizio/radar_api/errno.h"
#include "provizio/radar_api/radar_point_cloud.h"
#include "provizio/radar_api/radar_points_accumulation.h"
#include "provizio/radar_api/radar_ranges.h"
#include "provizio/socket.h"
#include "provizio/util.h"

#define PROVIZIO__RADAR_API_DEFAULT_PORT ((uint16_t)7769U) // Shared for radar point cloud and radar entities messages
#define PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_PORT ((uint16_t)7770U)
#define PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_TIMEOUT ((uint64_t)30000000000ULL)
#define PROVIZIO__RADAR_API_REQUEST_RANGE_DEFAULT_PORT                                                                 \
    PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_PORT // Uses "set range to unknown" to request it

/**
 * @brief A single Provizio Radar API connection handle on a single UDP port
 */
typedef struct provizio_radar_api_connection
{
    PROVIZIO__SOCKET sock;
    provizio_radar_point_cloud_api_context *radar_point_cloud_api_contexts;
    size_t num_radar_point_cloud_api_contexts;
    provizio_radar_entities_api_context *radar_entities_api_contexts;
    size_t num_radar_entities_api_contexts;
} provizio_radar_api_connection;

/**
 * @brief Connect to the Provizio radar to start receiving packets by UDP (single radar on a UDP port)
 *
 * @param udp_port UDP port to receive from, by default = PROVIZIO__RADAR_API_DEFAULT_PORT (if 0)
 * @param receive_timeout_ns Max number of nanoseconds provizio_radar_api_receive_packet should wait for a
 * packet, or 0 to wait as long as required
 * @param check_connection Use any non-zero value if the connection is to be checked to be receiving anything prior to
 * returning a successful result
 * @param radar_point_cloud_api_context Initialized provizio_radar_point_cloud_api_context to handle point cloud packets
 * (may be NULL to skip any point cloud packets)
 * @param radar_entities_api_context Initialized provizio_radar_entities_api_context to handle entities packets (may be
 * NULL to skip any entities packets)
 * @param out_connection A provizio_radar_api_connection to store the connection handle
 * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
 *
 * @note The connection has to be eventually closed with provizio_close_radar_connection
 */
PROVIZIO__EXTERN_C int32_t provizio_open_radar_connection(
    uint16_t udp_port, uint64_t receive_timeout_ns, uint8_t check_connection,
    provizio_radar_point_cloud_api_context *radar_point_cloud_api_context,
    provizio_radar_entities_api_context *radar_entities_api_context, provizio_radar_api_connection *out_connection);

/**
 * @brief Connect to the radar API to start receiving packets by UDP (multiple radars on the same UDP port)
 *
 * @param udp_port UDP port to receive from, by default = PROVIZIO__RADAR_API_DEFAULT_PORT
 * @param receive_timeout_ns Max number of nanoseconds provizio_radar_api_receive_packet should wait for a
 * packet, or 0 to wait as long as required
 * @param check_connection Use any non-zero value if the connection is to be checked to be receiving anything prior to
 * returning a successful result
 * @param radar_point_cloud_api_contexts Array of initialized provizio_radar_point_cloud_api_context to handle point
 * cloud packets (may be NULL to skip any point cloud packets)
 * @param num_radar_point_cloud_api_contexts Number of radar_point_cloud_api_contexts, i.e. max numbers of radars to
 * handle (may be 0 to skip any point cloud packets)
 * @param radar_entities_api_contexts Array of initialized provizio_radar_entities_api_context to handle entities
 * packets (may be NULL to skip any entities packets)
 * @param num_radar_entities_api_contexts Number of radar_entities_api_contexts, i.e. max numbers of radars to handle
 * (may be 0 to skip any entities packets)
 * @param out_connection A provizio_radar_api_connection to store the connection handle
 * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
 *
 * @note The connection has to be eventually closed with provizio_close_radars_connection
 */
PROVIZIO__EXTERN_C int32_t provizio_open_radars_connection(
    uint16_t udp_port, uint64_t receive_timeout_ns, uint8_t check_connection,
    provizio_radar_point_cloud_api_context *radar_point_cloud_api_contexts, size_t num_radar_point_cloud_api_contexts,
    provizio_radar_entities_api_context *radar_entities_api_contexts, size_t num_radar_entities_api_contexts,
    provizio_radar_api_connection *out_connection);

/**
 * @brief Receive and handle the next UDP packet using a previously connected API
 *
 * @param connection A previously connected provizio_radar_api_connection
 * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, PROVIZIO_E_SKIPPED if received but skipped,
 * other error value if failed for another reason
 */
PROVIZIO__EXTERN_C int32_t provizio_radar_api_receive_packet(provizio_radar_api_connection *connection);

/**
 * @brief Closes a previously connected radar API (either a single or multiple radars on the same port)
 *
 * @param connection A previously connected provizio_radar_api_connection
 * @return 0 if successful, error code otherwise
 */
PROVIZIO__EXTERN_C int32_t provizio_close_radars_connection(provizio_radar_api_connection *connection);
#define provizio_close_radar_connection provizio_close_radars_connection

/**
 * @brief Makes a radar (or all radars) change a range with a default timeout =
 * PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_TIMEOUT
 *
 * @param radar_position_id Either one of provizio_radar_position enum values or a custom position id. Can also be
 * provizio_radar_position_any to set for all radars
 * @param range Target range to set
 * @param udp_port UDP port to send change range message, by default = PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_PORT
 * (if 0)
 * @param ipv4_address IP address to send change range message, in the standard IPv4 dotted decimal notation. By default
 * = "255.255.255.255" - broadcast (if NULL). Note: broadcasting to 255.255.255.255 may be unsupported in macOS.
 * @param out_actual_radar_range If not NULL, actual radar range after the operation will be stored here if received
 * from the radar (will be provizio_radar_range_unknown otherwise)
 * @return 0 if set successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
 */
PROVIZIO__EXTERN_C int32_t provizio_set_radar_range(provizio_radar_position radar_position_id,
                                                    provizio_radar_range range, uint16_t udp_port,
                                                    const char *ipv4_address,
                                                    provizio_radar_range *out_actual_radar_range);

/**
 * @brief Makes a radar (or all radars) change a range with a custom timeout
 *
 * @param radar_position_id Either one of provizio_radar_position enum values or a custom position id. Can also be
 * provizio_radar_position_any to set for all radars
 * @param range Target range to set
 * @param udp_port UDP port to send change range message, by default = PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_PORT
 * (if 0)
 * @param ipv4_address IP address to send change range message, in the standard IPv4 dotted decimal notation. By default
 * = "255.255.255.255" - broadcast (if NULL). Note: broadcasting to 255.255.255.255 may be unsupported in macOS.
 * @param out_actual_radar_range If not NULL, actual radar range after the operation will be stored here if received
 * from the radar (will be provizio_radar_range_unknown otherwise)
 * @param timeout_ns The operation timeout (nanoseconds)
 * @return 0 if set successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
 */
PROVIZIO__EXTERN_C int32_t provizio_set_radar_range_with_timeout(provizio_radar_position radar_position_id,
                                                                 provizio_radar_range range, uint16_t udp_port,
                                                                 const char *ipv4_address,
                                                                 provizio_radar_range *out_actual_radar_range,
                                                                 uint64_t timeout_ns);

/**
 * @brief Requests the current range a radar operates
 *
 * @param radar_position_id Either one of provizio_radar_position enum values or a custom position id. Can also be
 * provizio_radar_position_any to request from any radar (assumes a single Provizio radar in the network)
 * @param udp_port UDP port to send the request message, by default = PROVIZIO__RADAR_API_REQUEST_RANGE_DEFAULT_PORT
 * (if 0)
 * @param ipv4_address IP address to send request range message, in the standard IPv4 dotted decimal notation. By
 * default = "255.255.255.255" - broadcast (if NULL). Note: broadcasting to 255.255.255.255 may be unsupported in macOS.
 * @param out_current_radar_range Current radar range is stored here if successful (will be provizio_radar_range_unknown
 * otherwise)
 * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
 */
PROVIZIO__EXTERN_C int32_t provizio_request_current_range(provizio_radar_position radar_position_id, uint16_t udp_port,
                                                          const char *ipv4_address,
                                                          provizio_radar_range *out_current_radar_range);

#endif // PROVIZIO_RADAR_API_CORE
