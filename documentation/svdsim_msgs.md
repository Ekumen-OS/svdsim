# svdsim_msgs Message Definitions

`svdsim_msgs` contains the custom ROS 2 messages exchanged by the bridge in addition to the standard
sensor messages. They are thin wrappers around values that have no standard equivalent in
`sensor_msgs` or `nav_msgs`.

All four messages are defined in `svd_simulator/svdsim_msgs/msg/` and generated with
`rosidl_default_generators`.

## Message overview

| Message | Direction | Purpose |
|---|---|---|
| `StampedSteering` | ROS → server, server → ROS | Steering angle command and feedback |
| `StampedVelocity` | ROS → server | Longitudinal velocity command |
| `StampedEncoderData` | server → ROS | Cumulative rear wheel encoder counts |
| `CollisionData` | server → ROS | Collision event counters and current state |

## StampedSteering

```text
std_msgs/Header header
float64 steering   # Steering angle in radians
```

As a command it is the requested front wheel angle; as feedback it is either the simulated angle or
the echoed command, depending on `use_true_steering_feedback_sensor` in `svdsim_server`. The sign
convention is positive for a left turn, matching the Ackermann convention used by the controller
stack.

The bridge ignores the `header` of incoming commands. The value is read by the remote-control poll
and is latched as a reference for the vehicle. The header stamp therefore does not influence when
the command takes effect.

## StampedVelocity

```text
std_msgs/Header header
float64 velocity   # Velocity in m/s
```

Longitudinal velocity reference. The reference is shaped by the drive model before it becomes an
acceleration command; see [vehicle dynamics parameters](svdsim_server/vehicle_dynamics.md#velocity).
Negative values request reverse motion.

## StampedEncoderData

```text
std_msgs/Header header
int64 rear_left_encoder   # Rear left wheel encoder pulse count
int64 rear_right_encoder  # Rear right wheel encoder pulse count
```

Absolute, monotonically increasing tick counts. They are reset when the vehicle pose is reset. The
resolution is set by `wheel_encoders.pulses_per_revolution` and
`wheel_encoders.conversion_ratio`; see [Wheel encoder parameters](svdsim_server/wheel_encoders.md).

The fields are cumulative pulse *counts*, not deltas and not velocities; computing a velocity
requires differencing consecutive messages with their header stamps.

## CollisionData

```text
uint64 obstacle_collision_count
uint64 vehicle_collision_count
bool in_collision_with_obstacle
bool in_collision_with_vehicle
```

The counters count transitions from "no overlap" to "overlap", not the number of physics steps spent
in contact. The booleans describe the state at publication time. The topic QoS is given in the
[bridge topic table](svdsim_bridge.md#published).

## See also

- [Bridge parameters](svdsim_bridge.md)
- [Protobuf definitions](svdsim_proto.md)
- [Wheel encoder parameters](svdsim_server/wheel_encoders.md)
- [Vehicle footprint parameters](svdsim_server/vehicle_footprint.md)
