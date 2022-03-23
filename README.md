# provizio_radar_api_core

The C API for communicating with Provizio radars

## UDP Protocol

### Radar Point Clouds

Radar point clouds are published as broadcast UDP packets, default UDP port number: 7769.
Point clouds are sent as multiple packets per frame (i.e. single read that can be assumed instantaneous).
Each packet has a size of 1472 bytes or less, depending on a number of points it contains.

Each packet has the following structure (all fields use network bytes order when applicable, no padding):

| Field                         | Size (bytes) | Data Type | Description                                                                                            |
|-------------------------------|--------------|-----------|--------------------------------------------------------------------------------------------------------|
| packet_type                   | 2            | uint16_t  | Always = 1, can't change even on protocol updates                                                      |
| protocol_version              | 2            | uint16_t  | Currently = 1, to be incremented on any protocol changes (used for backward compatibility)             |
| frame_index                   | 4            | uint32_t  | 0-based frame index (resets back to 0 if ever exceeds 4294967295)                                      |
| timestamp                     | 8            | uint64_t  | Time of the frame capture measured in milliseconds since the UNIX epoch (January 1, 1970 00:00:00 UTC) |
| radar_position_id             | 2            | uint16_t  | Either one of provizio_radar_position enum values or a custom position id                              |
| total_points_in_frame         | 2            | uint16_t  | Total number of points in the frame #frame_index, never exceeds 65535                                  |
| num_points_in_packet          | 2            | uint16_t  | Number of points in the current packet, never exceeds (1472 - 24) / 20                                 |
| (reserved)                    | 2            |           | Not used currently, kept for better alignment and potential future use                                 |
| point_0: x_meters             | 4            | float     | Radar-relative X (forward) position of the point in meters                                             |
| point_0: y_meters             | 4            | float     | Radar-relative Y (left) position of the point in meters                                                |
| point_0: z_meters             | 4            | float     | Radar-relative Z (up) position of the point in meters                                                  |
| point_0: velocity_m_s         | 4            | float     | Radar-relative velocity of the point in meters per second                                              |
| point_0: signal_to_noise_ratio| 4            | float     | Signal-to-noise ratio (unitless)                                                                       |
| point_1: ...                  |              |           |                                                                                                        |
| ...                           |              |           |                                                                                                        |
| **Total**                     | **24 + (20 * num_points_in_packet)** | | Never exceeds 1472 bytes                                                                 |
