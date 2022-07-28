# provizio_radar_api_core

- [provizio_radar_api_core](#provizio_radar_api_core)
  - [Usage](#usage)
    - [Building and Linking](#building-and-linking)
    - [Initialization](#initialization)
    - [Connection](#connection)
    - [Receiving Point Clouds](#receiving-point-clouds)
      - [Live UDP](#live-udp)
      - [Replay or Custom Transport](#replay-or-custom-transport)
    - [Changing Radar Modes](#changing-radar-modes)
    - [Shutting Down](#shutting-down)
  - [UDP Protocol](#udp-protocol)
    - [Radar Point Clouds](#radar-point-clouds)
    - [Radar Modes](#radar-modes)

The official C library providing API for communicating with Provizio radars.

- Open-source as Apache License 2.0 (see [LICENSE](LICENSE))
- Written in C (supports C99+, can also be used in C++ and other client languages)​
- Supports Linux, macOS, Windows; x64/x86/ARM
- Built with CMake 3.1.0+
- No external dependencies (unit tests that can be disabled have open-source dependencies under compatible licenses, automatically resolved during build from Provizio forks when enabled)​
- No dynamic allocations internally, uses client-preallocated objects (can use stack, heap, custom allocation)​
- No threads created internally but supports single-threaded and multi-threaded use​
- Provides built-in UDP interfacing, but also allows for integrating any custom transport or replays​
- MISRA-compatible​
- Complete unit-tests coverage (validated in CI)​

## Usage

### Building and Linking

**provizio_radar_api_core** is a static C Library built with CMake 3.1.0+.
There is a number of options to use it in your project. Some of the options:

1. For [CMake](https://cmake.org/)-based C/C++ projects, you may use `ExternalProject_Add`, f.e.

    ```CMake
    # Include support for ExternalProject_Add
    include(ExternalProject)

    set(PROVIZIO_RADAR_CORE_API_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_radar_api_core_build")
    set(PROVIZIO_RADAR_CORE_API_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_radar_api_core")
    set(PROVIZIO_RADAR_CORE_API_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
    set(PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT "provizio/provizio_radar_api_core") # Or your own fork
    set(PROVIZIO_RADAR_CORE_API_GITHUB_BRANCH "master") # Or a specific tag you'd like to use
    set(PROVIZIO_RADAR_CORE_API_INSTALL_DIR "${PROVIZIO_RADAR_CORE_API_BINARY_DIR}/install")
    ExternalProject_Add(libprovizio_radar_api_core
        GIT_REPOSITORY "https://github.com/${PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT}.git" # or "git@github.com:${PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT}.git" for ssh access, or your custom repo
        GIT_TAG "${PROVIZIO_RADAR_CORE_API_GITHUB_BRANCH}"
        PREFIX "${PROVIZIO_RADAR_CORE_API_PREFIX}"
        SOURCE_DIR "${PROVIZIO_RADAR_CORE_API_SOURCE_DIR}"
        BINARY_DIR "${PROVIZIO_RADAR_CORE_API_BINARY_DIR}"
        CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}" "-DENABLE_CHECK_FORMAT=OFF" "-DBUILD_TESTING=OFF"
    )
    add_dependencies(<your cmake target name> libprovizio_radar_api_core)
    target_include_directories(<your cmake target name> SYSTEM PUBLIC "${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}/include")
    if ("${CMAKE_VERSION}" VERSION_GREATER_EQUAL "3.13.0")
        target_link_directories(<your cmake target name> PUBLIC "${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}/lib")
    else ()
        link_directories("${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}/lib")
    endif ("${CMAKE_VERSION}" VERSION_GREATER_EQUAL "3.13.0")
    target_link_libraries(<your cmake target name> provizio_radar_api_core)
    ```

2. Another option for C/C++ CMake-based projects is `add_subdirectory`. **provizio_radar_api_core** can be copied as a folder / made a git submodule of your repo. Then it's accessible as

    ```CMake
    add_subdirectory(<relative path to provizio_radar_api_core>)
    target_link_libraries(<your cmake target name> provizio_radar_api_core)
    ```

3. **provizio_radar_api_core** can be built and installed in standard system paths and then used / linked directly. F.e.:

    ```Bash
    cd /tmp
    git clone --branch master --depth 1 https://github.com/provizio/provizio_radar_api_core.git # Or your own fork and a specific version tag
    mkdir -p provizio_radar_api_core/build
    cd provizio_radar_api_core/build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_CHECK_FORMAT=OFF -DBUILD_TESTING=OFF
    cmake --build .
    sudo cmake --install .
    ```

4. **provizio_radar_api_core** can be built and installed to a temporary folder during your project's build. F.e. in a `Makefile`:

    ```Make
    PROVIZIO_RADAR_API_CORE_REPO := https://github.com/provizio/provizio_radar_api_core.git # Or your own fork
    PROVIZIO_RADAR_API_CORE_BRANCH := master # Or a specific version tag
    PROVIZIO_RADAR_API_CORE_BUILD_TYPE := Release # Or Debug
    PROVIZIO_RADAR_API_CORE_SRC := ./build/provizio_radar_api_core
    PROVIZIO_RADAR_API_CORE_BUILD := $(PROVIZIO_RADAR_API_CORE_SRC)/build
    PROVIZIO_RADAR_API_CORE_INSTALL := $(PROVIZIO_RADAR_API_CORE_BUILD)/install

    $(PROVIZIO_RADAR_API_CORE_INSTALL)/lib/libprovizio_radar_api_core.a:
        mkdir -p build
        if [ ! -d "$(PROVIZIO_RADAR_API_CORE_SRC)/include/provizio" ]; then rm -rf $(PROVIZIO_RADAR_API_CORE_SRC) && cd build && git clone --depth 1 --branch $(PROVIZIO_RADAR_API_CORE_BRANCH) $(PROVIZIO_RADAR_API_CORE_REPO); fi
        mkdir -p $(PROVIZIO_RADAR_API_CORE_BUILD)
        cd $(PROVIZIO_RADAR_API_CORE_BUILD) && cmake .. -DCMAKE_INSTALL_PREFIX=install -DCMAKE_BUILD_TYPE=$(PROVIZIO_RADAR_API_CORE_BUILD_TYPE) -DENABLE_CHECK_FORMAT=OFF -DBUILD_TESTING=OFF
        cd $(PROVIZIO_RADAR_API_CORE_BUILD) && cmake --build .
        cd $(PROVIZIO_RADAR_API_CORE_BUILD) && cmake --build . --target install --config $(PROVIZIO_RADAR_API_CORE_BUILD_TYPE)

    <your Makefile target>: $(PROVIZIO_RADAR_API_CORE_INSTALL)/lib/libprovizio_radar_api_core.a <any other dependencies>
        ...
    ```

### Initialization

1. Include the core header

    ```C
    #include "provizio/radar_api/core.h"
    ```

2. Optionally, set your custom warning and error handlers (otherwise printing to `stderr` will be used by default)

    ```C
    // Somewhere in a header
    
    void your_warning_handler(const char *warning);
    void your_error_handler(const char *error);

    // In a C/C++ function

    /**
    * @brief Specifies a custom function to be called on warning
    *
    * @param warning_function Function pointer or NULL (resets to default)
    * @warning Not thread safe, so it's recommended to call prior to starting any threads
    * @note By default printing to stderr is used on warning.
    */
    provizio_set_on_warning(&your_warning_handler);
    
    /**
    * @brief Specifies a custom function to be called on error
    *
    * @param warning_function Function pointer or NULL (resets to default)
    * @warning Not thread safe, so it's recommended to call prior to starting any threads
    * @note By default printing to stderr is used on error.
    */
    provizio_set_on_error(&your_error_handler);
    ```

3. For live UDP use, initialize sockets system once per process run

    ```C
    /**
    * @brief Enables sockets operations - to be called once prior to any other API calls
    *
    * @return 0 if successfull, non-zero error code otherwise
    * @note Required in Windows, unless WSAStartup is called somewhere else, can be omitted in other platforms
    */
    int32_t status = provizio_sockets_initialize();
    ```

4. Create and initialize a `provizio_radar_point_cloud_api_context` or an array of a few.
    The contexts are used to receive and assemble radar point clouds.
    You'll need at least as many of them as there are radars configured to use same UDP port in the network.
    The size of a single `provizio_radar_point_cloud_api_context` exceeds 2Mb, so you may prefer to place them in heap unless your app is built with a large stack size.

    ```C
    // Somewhere in a header

    // Your point cloud handling callback - see Receiving Point Clouds for more details
    void your_radar_point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
                                         provizio_radar_point_cloud_api_context *context);

    // In a C/C++ function

    void* your_callback_data = <your optional callback data, may be NULL>;

    provizio_radar_point_cloud_api_context api_context;  // Large object, may cause stack overflow depending on your stack size
    
    /**
    * @brief Initializes a provizio_radar_point_cloud_api_context object to handle a single radar
    *
    * @param callback Function to be called on receiving a complete or partial radar point cloud
    * @param user_data Custom argument to be passed to the callback, may be NULL
    * @param context The provizio_radar_point_cloud_api_context object to initialize
    *
    * @warning radar_position_id of all packets handled by this context must be same
    */
    provizio_radar_point_cloud_api_context_init(&your_radar_point_cloud_callback, your_callback_data, &api_context);
    ```

    or

    ```C
    // Somewhere in a header

    // Your point cloud handling callback - see Receiving Point Clouds for more details
    void your_radar_point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
                                         provizio_radar_point_cloud_api_context *context);

    // In a C/C++ function

    const uint16_t num_contexts = <your max number of radars on the same UDP port>;
    void* your_callback_data = <your optional callback data, may be NULL>;

    provizio_radar_point_cloud_api_context *api_contexts =
        (provizio_radar_point_cloud_api_context *)malloc(sizeof(provizio_radar_point_cloud_api_context) * num_contexts);
    
    /**
    * @brief Initializes multiple provizio_radar_point_cloud_api_context objects to handle packets from multiple radars
    *
    * @param callback Function to be called on receiving a complete or partial radar point cloud
    * @param user_data Custom argument to be passed to the callback, may be NULL
    * @param contexts Array of num_contexts of provizio_radar_point_cloud_api_context objects to initialize
    * @param num_contexts Number of contexts (i.e. max numbers of radars to handle) to initialize
    */
    provizio_radar_point_cloud_api_contexts_init(&your_radar_point_cloud_callback, your_callback_data, api_contexts, num_contexts);
    ```

### Connection

When the API is used in live UDP mode, [initialization](#initialization) is followed by connection.

- Single radar on a UDP port use case:

    ```C
    const uint16_t udp_port = PROVIZIO__RADAR_API_DEFAULT_PORT;
    const uint64_t receive_timeout_ns = 5000000000ULL; // 0.5 sec here, use 0 to never time out
    const uint8_t check_connection = 0; // Specify any non-zero value to wait for a valid packet during the connection

    /**
     * @brief A single Provizio Radar API connection handle on a single UDP port
     */
    provizio_radar_api_connection connection;

    /**
     * @brief Connect to the Provizio radar to start receiving packets by UDP (single radar on a UDP port)
     *
     * @param udp_port UDP port to receive from, by default = PROVIZIO__RADAR_API_DEFAULT_PORT
     * @param receive_timeout_ns Max number of nanoseconds provizio_radar_api_receive_packet should wait for a
     * packet, or 0 to wait as long as required
     * @param check_connection Use any non-zero value if the connection is to be checked to be receiving anything prior to
     * returning a successful result
     * @param radar_point_cloud_api_context Initialized provizio_radar_point_cloud_api_context to handle point cloud packets
     * (may be NULL to skip any point cloud packets)
     * @param out_connection A provizio_radar_api_connection to store the connection handle
     * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
     *
     * @note The connection has to be eventually closed with provizio_radar_api_close_connection
     */
    int32_t status = provizio_open_radar_connection(udp_port, receive_timeout_ns, check_connection, &api_context, &connection);
    ```

- Multiple radars on the same UDP port use case:

    ```C
    const uint16_t udp_port = PROVIZIO__RADAR_API_DEFAULT_PORT;
    const uint64_t receive_timeout_ns = 5000000000ULL; // 0.5 sec here, use 0 to never time out
    const uint8_t check_connection = 0; // Specify any non-zero value to wait for a valid packet during the connection

    /**
     * @brief A single Provizio Radar API connection handle on a single UDP port
     */
    provizio_radar_api_connection connection;

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
    * @param out_connection A provizio_radar_api_connection to store the connection handle
    * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
    *
    * @note The connection has to be eventually closed with provizio_radar_api_close_connection
    */
    int32_t status = provizio_open_radars_connection(udp_port, receive_timeout_ns, check_connection, api_contexts, num_contexts, &connection);
    ```

### Receiving Point Clouds

The Radar API supports missing and reordered packets, as seen with UDP. As a result be aware that the point clouds
callback may sometimes receive incomplete point clouds.
The callback is guaranteed to be called with ever increasing values of `frame_index` (i.e. it never receives an older
point cloud after a newer one). The only potential case when `frame_index` can drop is resetting on exceeding
`4294967295` (i.e. over 4 billion) frames. But even then `timestamp` values can only grow.

```C
void your_radar_point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
                                     provizio_radar_point_cloud_api_context *context)
{
    // Radar position: either one of predefined values such as provizio_radar_position_front_center or a custom configured id
    const provizio_radar_position radar_position_id = (provizio_radar_position)context->radar_position_id;

    // As specified when initializing the context
    my_custom_user_data_type *user_data = (my_custom_user_data_type *)context->user_data;

    // 0-based radar frame index
    const uint32_t frame_index = point_cloud->frame_index;

    // Time of the frame capture measured in absolute number of nanoseconds since the start of the GPS Epoch (midnight on Jan 6, 1980)
    const uint64_t timestamp = point_cloud->timestamp;

    // Either one of provizio_radar_position enum values or a custom position id
    const uint16_t radar_position_id = point_cloud->radar_position_id;

    // Number of points in the entire frame
    const uint16_t num_points_expected = point_cloud->num_points_expected;

    // Number of points in the frame received so far
    const uint16_t num_points_received = point_cloud->num_points_received;

    // Radar mode (short range / medium range / long range / ultra-long range) used to capture the point cloud
    const provizio_radar_mode radar_mode = (provizio_radar_mode)point_cloud->radar_mode;

    if (num_points_received < num_points_expected)
    {
        // Received an incomplete point cloud due to some missing packets
    }
    else
    {
        // A complete point cloud received

        assert(num_points_received == num_points_expected);  // num_points_received may never exceed num_points_expected
    }

    for (uint16_t i = 0; i < num_points_received; ++i)
    {
        const float x_meters = point_cloud->radar_points[i].x_meters;         // Forward, radar relative
        const float y_meters = point_cloud->radar_points[i].y_meters;         // Left, radar relative
        const float z_meters = point_cloud->radar_points[i].z_meters;         // Up, radar relative
        const float velocity_m_s = point_cloud->radar_points[i].velocity_m_s; // Forward, radar relative
        const float signal_to_noise_ratio = point_cloud->radar_points[i].signal_to_noise_ratio;

        // Do whatever you like with the point
    }
}
```

#### Live UDP

Once connected, you can start receiving UDP protocol packets:

```C
/**
 * @brief Receive and handle the next UDP packet using a previously connected API
 *
 * @param connection A previously connected provizio_radar_api_connection
 * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, PROVIZIO_E_SKIPPED if received but skipped,
 * other error value if failed for another reason
 */
int32_t status = provizio_radar_api_receive_packet(&connection);
```

This call may invoke `your_radar_point_cloud_callback` up to
`PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT` times (2 by default) serially.

#### Replay or Custom Transport

You may want to handle a radar protocol packet that has been received in some other way (f.e. read from a recording
file or delivered over some custom transport implementation).

There are a few options for this:

- Single-radar, known packet type use case:

    ```C
    const provizio_radar_point_cloud_packet *packet = your_packet;
    const size_t packet_size = your_packet_size;

    /**
    * @brief Handles a single radar point cloud UDP packet from a single radar
    *
    * @param context Previously initialized provizio_radar_point_cloud_api_context
    * @param packet Valid provizio_radar_point_cloud_packet
    * @param packet_size The size of the packet, to check data is valid and avoid out-of-bounds access
    * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
    * other error code in case of another error
    *
    * @warning radar_position_id of all packets handled by this context must be same (returns an error otherwise)
    */
    int32_t status = provizio_handle_radar_point_cloud_packet(&context, packet, packet_size);
    ```

- Multiple-radars, known packet type use case:

    ```C
    const provizio_radar_point_cloud_packet *packet = your_packet;
    const size_t packet_size = your_packet_size;

    /**
    * @brief Handles a single radar point cloud UDP packet from one of multiple radars
    *
    * @param contexts Previously initialized array of num_contexts of provizio_radar_point_cloud_api_context objects
    * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
    * @param packet Valid provizio_radar_point_cloud_packet
    * @param packet_size The size of the packet, to check data is valid and avoid out-of-bounds access
    * @return 0 in case the packet was handled successfully, PROVIZIO_E_SKIPPED in case the packet was skipped as obsolete,
    * PROVIZIO_E_OUT_OF_CONTEXTS in case num_contexts is not enough, other error code in case of another error
    */
    int32_t status = provizio_handle_radars_point_cloud_packet(contexts, num_contexts, packet, packet_size);
    ```

- Single-radar, unknown packet type use case:

    ```C
    const void *payload = your_payload;
    const size_t payload_size = your_payload_size;

    /**
    * @brief Handles a single Provizio Radar API UDP packet from a single radar, that can be a correct
    * provizio_radar_point_cloud_packet or something else
    *
    * @param context Previously initialized provizio_radar_point_cloud_api_context
    * @param payload The payload of the UDP packet
    * @param payload_size The size of the payload in bytes
    * @return 0 if it's a provizio_radar_point_cloud_packet and it was handled successfully, PROVIZIO_E_SKIPPED if it's not
    * a provizio_radar_point_cloud_packet, other error code if it's a provizio_radar_point_cloud_packet but its handling
    * failed for another reason
    *
    * @warning if it's a provizio_radar_point_cloud_packet, radar_position_id of all packets handled by this context must
    * be same (returns an error otherwise)
    */
    int32_t status = provizio_handle_possible_radar_point_cloud_packet(&context, payload, payload_size);
    ```

- Multiple-radars, unknown packet type use case:

    ```C
    const void *payload = your_payload;
    const size_t payload_size = your_payload_size;

    /**
    * @brief Handles a single Provizio Radar API UDP packet from one of multiple radars, that can be a correct
    * provizio_radar_point_cloud_packet or something else
    *
    * @param contexts Previously initialized array of num_contexts of provizio_radar_point_cloud_api_context objects
    * @param num_contexts Number of contexts (i.e. max numbers of radars to handle)
    * @param payload The payload of the UDP packet
    * @param payload_size The size of the payload in bytes
    * @return 0 if it's a provizio_radar_point_cloud_packet and it was handled successfully, PROVIZIO_E_SKIPPED if it's not
    * a provizio_radar_point_cloud_packet, PROVIZIO_E_OUT_OF_CONTEXTS in case num_contexts is not enough, other error code
    * if it's a provizio_radar_point_cloud_packet but its handling failed for another reason
    */
    int32_t status = provizio_handle_possible_radars_point_cloud_packet(contexts, num_contexts, payload, payload_size);
    ```

Either of these calls may invoke `your_radar_point_cloud_callback` up to
`PROVIZIO__RADAR_POINT_CLOUD_API_CONTEXT_IMPL_POINT_CLOUDS_BEING_RECEIVED_COUNT` times (2 by default) serially.

### Changing Radar Modes

Provizio radars can operate in various modes, such as short range, medium range, long range, and ultra long range.
Current radar modes are specified in `provizio_radar_point_cloud`, you can change modes using `provizio_set_radar_mode`:

```C
/**
 * @brief Makes a radar (or all radars) change a mode
 *
 * @param radar_position_id Either one of provizio_radar_position enum values or a custom position id. Can also be
 * provizio_radar_position_any to set for all radars
 * @param mode Target mode to set
 * @param udp_port UDP port to send change mode message, by default = PROVIZIO__RADAR_API_SET_MODE_DEFAULT_PORT
 * (if 0)
 * @param ipv4_address IP address to send change mode message, in the standard IPv4 dotted decimal notation. By default
 * = "255.255.255.255" - broadcast (if NULL)
 * @return 0 if received successfully, PROVIZIO_E_TIMEOUT if timed out, other error value if failed for another reason
 */
int32_t status = provizio_set_radar_mode(radar_position_id, mode, udp_port, ipv4_address);
```

### Shutting Down

When you finished with the API, it has to be properly shut down.

1. In case you established a `provizio_radar_api_connection`, close the connection:

    ```C
    /**
    * @brief Closes a previously connected radar API (either a single or multiple radars on the same port)
    *
    * @param connection A previously connected provizio_radar_api_connection
    * @return 0 if successfull, error code otherwise
    */
    int32_t status = provizio_close_radars_connection(&connection);
    ```

2. In case you allocated `provizio_radar_point_cloud_api_context`[s], deallocate them correctly.

3. In case you called `provizio_sockets_initialize`, deinitialize sockets correctly:

    ```C
    /**
    * @brief Terminates sockets operations - to be called once after all the other API calls
    *
    * @return 0 if successfull, non-zero error code otherwise
    * @note Required in Windows, unless WSACleanup is called somewhere else, can be omitted in other platforms
    */
    int32_t status = provizio_sockets_deinitialize();
    ```

## UDP Protocol

The Provizio radars UDP protocol:

- Binary packets up to 1472 bytes long​
- Can tolerate lost packets​
- Point Clouds: up to 65535 points per point cloud, up to 72 points per UDP packet​
- Supports single or multiple radars, on the same or different ports​
- Permits identifying radars by position ids to allow for replacing radars easily​

### Radar Point Clouds

Radar point clouds are published as broadcast UDP packets, default UDP port number: 7769.
Point clouds are sent as multiple packets per frame (i.e. single read that can be assumed instantaneous).
Each packet has a size of 1472 bytes or less, depending on a number of points it contains.

Each packet has the following structure (all fields use network bytes order when applicable, no padding):

| Field                          | Size (bytes)                         | Data Type | Description                                                                                                                     |
| ------------------------------ | ------------------------------------ | --------- | ------------------------------------------------------------------------------------------------------------------------------- |
| packet_type                    | 2                                    | uint16_t  | Always = 1, can't change even on protocol updates                                                                               |
| protocol_version               | 2                                    | uint16_t  | Currently = 1, to be incremented on any protocol changes (used for backward compatibility)                                      |
| frame_index                    | 4                                    | uint32_t  | 0-based frame index (resets back to 0 if ever exceeds 4294967295)                                                               |
| timestamp                      | 8                                    | uint64_t  | Time of the frame capture measured in absolute number of nanoseconds since the start of the GPS Epoch (midnight on Jan 6, 1980) |
| radar_position_id              | 2                                    | uint16_t  | Either one of provizio_radar_position enum values or a custom position id                                                       |
| total_points_in_frame          | 2                                    | uint16_t  | Total number of points in the frame #frame_index, never exceeds 65535                                                           |
| num_points_in_packet           | 2                                    | uint16_t  | Number of points in the current packet, never exceeds (1472 - 24) / 20                                                          |
| radar_mode                     | 2                                    | uint16_t  | Radar mode, one of provizio_radar_mode enum values                                                                              |
| point_0: x_meters              | 4                                    | float     | Radar-relative X (forward) position of the point in meters                                                                      |
| point_0: y_meters              | 4                                    | float     | Radar-relative Y (left) position of the point in meters                                                                         |
| point_0: z_meters              | 4                                    | float     | Radar-relative Z (up) position of the point in meters                                                                           |
| point_0: velocity_m_s          | 4                                    | float     | Radar-relative velocity of the point in meters per second                                                                       |
| point_0: signal_to_noise_ratio | 4                                    | float     | Signal-to-noise ratio (unitless)                                                                                                |
| point_1: ...                   |                                      |           |                                                                                                                                 |
| ...                            |                                      |           |                                                                                                                                 |
| **Total**                      | **24 + (20 * num_points_in_packet)** |           | Never exceeds 1472 bytes                                                                                                        |

### Radar Modes

Setting radar modes is done via sending UDP packets to an appropriate port of a radar (or all radars in the local
network), default UDP port number: 7770. The radar sends back an acknowledgement packet.

Set mode packet:

| Field                          | Size (bytes)                         | Data Type | Description                                                                                                                     |
| ------------------------------ | ------------------------------------ | --------- | ------------------------------------------------------------------------------------------------------------------------------- |
| packet_type                    | 2                                    | uint16_t  | Always = 2, can't change even on protocol updates                                                                               |
| protocol_version               | 2                                    | uint16_t  | Currently = 1, to be incremented on any protocol changes (used for backward compatibility)                                      |
| radar_position_id              | 2                                    | uint16_t  | Either one of provizio_radar_position enum values or a custom position id                                                       |
| radar_mode                     | 2                                    | uint16_t  | One of provizio_radar_mode enum values                                                                                          |
| **Total**                      | **8**                                |           |                                                                                                                                 |

Acknowledgement packet:

| Field                          | Size (bytes)                         | Data Type | Description                                                                                                                     |
| ------------------------------ | ------------------------------------ | --------- | ------------------------------------------------------------------------------------------------------------------------------- |
| packet_type                    | 2                                    | uint16_t  | Always = 3, can't change even on protocol updates                                                                               |
| protocol_version               | 2                                    | uint16_t  | Currently = 1, to be incremented on any protocol changes (used for backward compatibility)                                      |
| radar_position_id              | 2                                    | uint16_t  | Either one of provizio_radar_position enum values or a custom position id                                                       |
| requested_radar_mode           | 2                                    | uint16_t  | One of provizio_radar_mode enum values                                                                                          |
| error_code                     | 4                                    | int32_t   | 0 for success, PROVIZIO_E_NOT_PERMITTED if the mode is not supported                                                            |
| **Total**                      | **12**                               |           |                                                                                                                                 |
