# svdsim_proto Wire Format

`svdsim_proto` defines the wire format between `svdsim_server` and `svdsim_bridge`. The transport is
ZeroMQ over TCP with a `PAIR` socket: the server binds `tcp://*:<port>`, the bridge connects to
`tcp://<hostname>:<port>`. One socket pair exists per vehicle.

All traffic is carried by a single protobuf envelope, `SimDataMessage`, which contains a timestamp
and a `oneof` with an inbound or an outbound payload. Schema evolution is therefore a matter of
adding fields to the existing messages; there is no separate message type per topic on the wire.

## Envelope

```protobuf
message SimDataMessage {
  Timestamp timestamp = 1;
  oneof payload {
    ServerInbound server_inbound = 6;
    ServerOutbound server_outbound = 7;
  }
}

message Timestamp {
  int64 seconds = 1;
  double fraction = 2;
}
```

`fraction` is the sub-second part in seconds, so the absolute time is
`seconds + fraction`. The bridge converts it into a `builtin_interfaces/msg/Time` and uses it as the
stamp of every message that has a header; `CollisionData` carries no header, so it is published
without a stamp.

## Inbound messages (ROS → server)

| Message | Fields | Sent when |
|---|---|---|
| `VelocityCommand` | `double value` | On the corresponding bridge topic |
| `SteeringCommand` | `double value` | On the corresponding bridge topic |
| `ResetVehicleState` | `double x`, `double y`, `double yaw` | On `initialpose` |
| `PingMessage` | `uint64 sequence_number` | Not sent by `svdsim_bridge`; the server handles it if received and answers with `PongMessage` |

`ResetVehicleState` resets the motion model state, the drive model filter, the encoder counters and
the collider pose, and clears any pending command for that vehicle.

## Outbound messages (server → ROS)

ROS topics and rates are listed in [svdsim_bridge](svdsim_bridge.md#topics).

| Message | Fields |
|---|---|
| `BroadcastTimestamp` | none (empty message; the payload only signals that the envelope timestamp is current) |
| `WheelEncoderData` | `int32 rear_left_pulse_count`, `int32 rear_right_pulse_count` |
| `SteeringFeedback` | `double value` |
| `LidarScanData` | `min_range`, `max_range`, `horizontal_fov`, `points_per_scan`, `scan_rate`, `repeated double ranges` |
| `IMUSampleData` | `accel_x/y/z`, `gyro_x/y/z`, `mag_x/y/z` |
| `GroundTruthData` | `x`, `y`, `yaw`, `vx`, `vy`, `yaw_rate` |
| `SparringPoseMarker` | `x`, `y`, `yaw`, `entity_id` |
| `SparringLookaheadMarker` | `target_x`, `target_y`, `entity_id` |
| `CollisionCountData` | `obstacle_collision_count`, `vehicle_collision_count`, `in_collision_with_obstacle`, `in_collision_with_vehicle` |
| `PongMessage` | `uint64 sequence_number` |

`LidarScanData` carries the full sensor description in every message, so the bridge does not need to
know the scan geometry in advance; it simply copies the values into the `LaserScan` metadata fields.
`GroundTruthData` is referenced to `base_link`, i.e. the midpoint of the rear axle at ground level,
with velocities expressed in the vehicle frame.

## Encoding and flow control

- Messages are serialised with `SerializeToArray` and sent as a single ZeroMQ frame.
- The send high-water mark is 100 messages on the TCP socket and all sends use the non-blocking
  (`dontwait`) flag. When the queue is full, a new message is rejected and `publish` returns false;
  the oldest queued message is not dropped.
- Each endpoint also uses an internal `inproc` `PAIR` socket so that the ZeroMQ socket is only
  touched by one thread; the ROS callbacks publish into the `inproc` socket and a dedicated spinner
  thread performs the TCP transfer.
- The bridge polls with a 20 ms timeout, which keeps the receive thread responsive without spinning.

## Adding a field

1. Add the field to `proto/sim_messages.proto` with a new, unused field number.
2. Regenerate the C++ sources through the package's CMake rule (`protobuf_generate_cpp`).
3. Populate it in `svdsim_server` and consume it in `svdsim_bridge`.
4. Keep field numbers stable: they are the wire identity. Never reuse the number of a removed field.

Because both ends are built from the same package, `svdsim_proto`, `svdsim_bridge` and
`svdsim_server` must be rebuilt together after a schema change so that the installed headers and the
running binaries agree; a proto/binary mismatch is detected by protobuf as a parse failure and
manifests as missing data.

## See also

- [Bridge parameters](svdsim_bridge.md)
- [Message definitions](svdsim_msgs.md)
- [Simulation rates](svdsim_server/simulation_rates.md)
