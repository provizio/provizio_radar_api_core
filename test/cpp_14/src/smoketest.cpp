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

// The purpose of this C++ test is to make sure the API C headers can successfully be included in a C++ code and don't
// crash when API functions are invoked. The logic is completely tested in the c_99 test.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "provizio/radar_api/core.h"
#include "provizio/radar_api/radar_points_accumulation.h"
#include "provizio/socket.h"

namespace
{
    std::unique_ptr<void, std::function<void(void *)>> make_guard(const PROVIZIO__SOCKET sock)
    {
        // LCOV_EXCL_START: lcov isn't great at handling C++ lambdas
        return std::unique_ptr<void, std::function<void(void *)>>{
            nullptr, [sock](void *ignored) {
                (void)ignored;

                if (provizio_socket_valid(sock) != 0 && provizio_socket_close(sock) != 0)
                {
                    throw std::runtime_error{"Failed to close a socket"};
                }
            }};
        // LCOV_EXCL_STOP
    }

    std::unique_ptr<void, std::function<void(void *)>> make_guard(provizio_radar_api_connection &connection)
    {
        // LCOV_EXCL_START: lcov isn't great at handling C++ lambdas
        return std::unique_ptr<void, std::function<void(void *)>>{
            nullptr, [&](void *ignored) {
                (void)ignored;

                if (provizio_socket_valid(connection.sock) != 0 && provizio_close_radar_connection(&connection) != 0)
                {
                    throw std::runtime_error{"Failed to close an API connection"}; // LCOV_EXCL_LINE: Shouldn't happen
                }
            }};
        // LCOV_EXCL_STOP
    }

    template <typename functor>
    void callback_wrapper(const provizio_radar_point_cloud *point_cloud,
                          struct provizio_radar_point_cloud_api_context *context)
    {
        (*static_cast<functor *>(context->user_data))(point_cloud, context);
    }
} // namespace

int main(int argc, char *argv[])
{
    std::int32_t error_code = 0;
    try
    {
        (void)argc;
        (void)argv;

        if (provizio_sockets_initialize() != 0)
        {
            throw std::runtime_error{"provizio_sockets_initialize failed!"}; // LCOV_EXCL_LINE: Shouldn't happen
        }

        constexpr std::uint16_t port_number = 10200 + PROVIZIO__RADAR_API_DEFAULT_PORT;
        constexpr std::uint32_t start_frame_index = 500;
        constexpr std::uint64_t timestamp = 0x0123456701234567ULL;
        constexpr std::uint16_t radar_position_id = provizio_radar_position_front_left;
        constexpr std::uint16_t num_points = 1;
        constexpr std::uint16_t radar_mode = provizio_radar_mode_long_range;
        constexpr std::uint64_t timeout_ns = 1000000000ULL;
        constexpr float point_x = 1.0F;
        constexpr float point_y = 2.0F;
        constexpr float point_z = 3.0F;
        constexpr float point_velocity = 4.0F;
        constexpr float point_signal_to_noise_ratio = 5.0F;
        constexpr float fix_east = 1.0F;
        constexpr float fix_north = 2.0F;
        constexpr float fix_up = 3.0F;
        constexpr float orientation_x = 1.0F;
        constexpr float orientation_y = 2.0F;
        constexpr float orientation_z = 3.0F;

        std::atomic<bool> finish{false};
        std::mutex exception_in_thread_mutex;
        std::string exception_in_send_thread;
        std::thread send_messages_thread{[&]() {
            try
            {
                const auto send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                const auto socket_guard = make_guard(send_socket);
                const std::chrono::milliseconds sleep_between_frames{100};

                if (provizio_socket_valid(send_socket) == 0)
                {
                    throw std::runtime_error{"Failed to open a socket"}; // LCOV_EXCL_LINE: Shouldn't happen
                }

                sockaddr_in my_address; // NOLINT: Initialized in the very next line
                std::memset(&my_address, 0, sizeof(my_address));
                my_address.sin_family = AF_INET;
                my_address.sin_port = 0;
                my_address.sin_addr.s_addr = INADDR_ANY;

                if (bind(send_socket,
                         reinterpret_cast<const sockaddr *>(&my_address), // NOLINT: reinterpret_cast is required here
                         sizeof(my_address)) != 0)
                {
                    throw std::runtime_error{"Failed to bind a socket"}; // LCOV_EXCL_LINE: Shouldn't happen
                }

                sockaddr_in target_address; // NOLINT: Initialized in the very next line
                std::memset(&target_address, 0, sizeof(target_address));
                target_address.sin_family = AF_INET;
                target_address.sin_port =
                    htons(port_number); // NOLINT: clang-tidy doesn't like htons, but it's defined a system header
                target_address.sin_addr.s_addr = inet_addr("127.0.0.1");

                provizio_radar_point_cloud_packet packet;
                std::memset(&packet, 0, sizeof(packet));
                provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.packet_type,
                                                     PROVIZIO__RADAR_API_POINT_CLOUD_PACKET_TYPE);
                provizio_set_protocol_field_uint16_t(&packet.header.protocol_header.protocol_version,
                                                     PROVIZIO__RADAR_API_POINT_CLOUD_PROTOCOL_VERSION);
                provizio_set_protocol_field_uint64_t(&packet.header.timestamp, timestamp);
                provizio_set_protocol_field_uint16_t(&packet.header.radar_position_id, radar_position_id);
                provizio_set_protocol_field_uint16_t(&packet.header.total_points_in_frame, num_points);
                provizio_set_protocol_field_uint16_t(&packet.header.num_points_in_packet, num_points);
                provizio_set_protocol_field_uint16_t(&packet.header.radar_mode, radar_mode);
                provizio_set_protocol_field_float(&packet.radar_points[0].x_meters, point_x);
                provizio_set_protocol_field_float(&packet.radar_points[0].y_meters, point_y);
                provizio_set_protocol_field_float(&packet.radar_points[0].z_meters, point_z);
                provizio_set_protocol_field_float(&packet.radar_points[0].radar_relative_radial_velocity_m_s,
                                                  point_velocity);
                provizio_set_protocol_field_float(&packet.radar_points[0].signal_to_noise_ratio,
                                                  point_signal_to_noise_ratio);
                provizio_set_protocol_field_float(&packet.radar_points[0].ground_relative_radial_velocity_m_s,
                                                  -point_velocity);

                for (auto frame_index = start_frame_index; !finish; ++frame_index) // NOLINT: Don't unroll
                {
                    provizio_set_protocol_field_uint32_t(&packet.header.frame_index, frame_index);

                    // reinterpret_cast is required here (due to the underlying C API)
                    if (sendto(send_socket, reinterpret_cast<const char *>(&packet), // NOLINT
                               static_cast<uint16_t>(provizio_radar_point_cloud_packet_size(&packet.header)), 0,
                               reinterpret_cast<const sockaddr *>(&target_address), // NOLINT
                               sizeof(target_address)) == -1)
                    {
                        throw std::runtime_error{"Failed to send a packet!"}; // LCOV_EXCL_LINE: Shouldn't happen
                    }

                    std::this_thread::sleep_for(sleep_between_frames);
                }
            }
            // LCOV_EXCL_START: Shouldn't happen
            catch (const std::exception &e)
            {
                const std::lock_guard<std::mutex> lock{exception_in_thread_mutex};
                exception_in_send_thread = e.what();
            }
            // LCOV_EXCL_STOP
        }};

        try
        {
            constexpr std::size_t num_accumulated_point_clouds = 2;
            std::array<provizio_accumulated_radar_point_cloud, // NOLINT: False positive: initialized next
                       num_accumulated_point_clouds>
                accumulated_point_clouds;
            provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds.data(), num_accumulated_point_clouds);

            auto received_point_cloud = std::make_unique<provizio_radar_point_cloud>();
            auto callback = [&](const provizio_radar_point_cloud *point_cloud,
                                provizio_radar_point_cloud_api_context *ignored) {
                (void)ignored;

                std::memcpy(received_point_cloud.get(), point_cloud, sizeof(provizio_radar_point_cloud));

                provizio_enu_fix fix;
                fix.position = {fix_east, fix_north, fix_up};
                provizio_quaternion_set_euler_angles(orientation_x, orientation_y, orientation_z, &fix.orientation);
                provizio_accumulate_radar_point_cloud_static(point_cloud, &fix, accumulated_point_clouds.data(),
                                                             num_accumulated_point_clouds);

                finish = true;
            };

            auto api_context = std::make_unique<provizio_radar_point_cloud_api_context>();
            provizio_radar_point_cloud_api_context_init(&callback_wrapper<decltype(callback)>, &callback,
                                                        api_context.get());

            provizio_radar_api_connection connection;
            const auto connection_guard = make_guard(connection);
            error_code = provizio_open_radar_connection(port_number, timeout_ns, 1, api_context.get(), &connection);
            if (error_code != 0)
            {
                // LCOV_EXCL_START: Shouldn't happen
                const std::lock_guard<std::mutex> lock{exception_in_thread_mutex};
                throw std::runtime_error{!exception_in_send_thread.empty() ? exception_in_send_thread
                                                                           : "Failed to establish an API connection"};
            }
            // LCOV_EXCL_STOP

            error_code = provizio_radar_api_receive_packet(&connection);
            if (error_code != 0)
            {
                // LCOV_EXCL_START: Shouldn't happen
                const std::lock_guard<std::mutex> lock{exception_in_thread_mutex};
                throw std::runtime_error{!exception_in_send_thread.empty() ? exception_in_send_thread
                                                                           : "Failed to receive a packet"};
            }
            // LCOV_EXCL_STOP

            if (received_point_cloud->num_points_expected != num_points ||
                received_point_cloud->num_points_received != num_points ||
                received_point_cloud->radar_position_id != radar_position_id ||
                received_point_cloud->timestamp != timestamp || received_point_cloud->radar_mode != radar_mode ||
                received_point_cloud->radar_points[0].x_meters != point_x ||
                received_point_cloud->radar_points[0].y_meters != point_y ||
                received_point_cloud->radar_points[0].z_meters != point_z ||
                received_point_cloud->radar_points[0].radar_relative_radial_velocity_m_s != point_velocity ||
                received_point_cloud->radar_points[0].signal_to_noise_ratio != point_signal_to_noise_ratio ||
                received_point_cloud->radar_points[0].ground_relative_radial_velocity_m_s != -point_velocity)
            {
                throw std::runtime_error{"Incorrect point cloud received"}; // LCOV_EXCL_LINE: Shouldn't happen
            }

            if (provizio_accumulated_radar_point_clouds_count(accumulated_point_clouds.data(),
                                                              num_accumulated_point_clouds) != 1 ||
                provizio_accumulated_radar_points_count(accumulated_point_clouds.data(),
                                                        num_accumulated_point_clouds) != 1)
            {
                throw std::runtime_error{"Failed to accumulate"}; // LCOV_EXCL_LINE: Shouldn't happen
            }
        }
        // LCOV_EXCL_START: Shouldn't happen
        catch (...)
        {
            // Make sure the thread gets correctly shut down even in case of an exception in the main thread
            finish = true;
            send_messages_thread.join();
            throw;
        }
        // LCOV_EXCL_STOP

        send_messages_thread.join();

        provizio_sockets_deinitialize();
        return 0;
    }
    // LCOV_EXCL_START: Shouldn't happen
    catch (const std::exception &e)
    {
        provizio_sockets_deinitialize();

        std::cerr << e.what() << " (error code = " << error_code << ")\n";
        return error_code != 0 ? error_code : -1;
    }
    // LCOV_EXCL_STOP
}
