# svdsim_bridge Parameters and Topics

`svdsim_bridge` is the ROS 2 adapter between `svdsim_server` and the rest of the stack. It connects to
one vehicle's ZeroMQ endpoint, publishes the sensor streams as standard ROS messages, forwards
controller commands back to the simulator, and republishes the simulation clock.

The node (`svdsim_bridge_node`, class `svdsim_bridge::SvdsimBridgeNode`) is composable and runs
either standalone (`ros2 run svdsim_bridge svdsim_bridge_node`) or inside a component container.
The complete parameter file with the defaults below is
[`config_examples/svdsim_bridge_default.yaml`](config_examples/svdsim_bridge_default.yaml).

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `hostname` | string | `localhost` | Host running `svdsim_server`. |
| `port` | int | 4242 | ZeroMQ port of the vehicle. Must equal the `port` column of that vehicle in `vehicle_grid.csv`. |
| `lidar_frame_id` | string | `laser` | Frame id of `LaserScan`. Must match the URDF/tf frame of the LiDAR. |
| `imu_frame_id` | string | `imu` | Frame id of `Imu` and `MagneticField`. |
| `map_frame_id` | string | `map` | Frame id of `Odometry` (ground truth) and of the sparring markers. Also the only frame accepted by `initialpose`. |
| `base_link_frame_id` | string | `base_link` | Child frame of the ground-truth `Odometry`. |

All parameters are declared without descriptors, and none of them are validated at runtime. A
mismatch between `port` and the grid file only manifests as a silent absence of data, because the
ZeroMQ client retries the connection.

## Topics

The node creates its endpoints with node-local names. Launch files may remap them; the
`Remapped name` column lists the names used by the Monaco RoboRacer bringup, where the simulator is
integrated with the racing stack. Standalone launches see the node-local names.

### Published

| Topic | Remapped name | Type | Frame | Rate | QoS |
|---|---|---|---|---|---|
| `/clock` | `/clock` (not remapped) | `rosgraph_msgs/msg/Clock` | - | `command_rate` | depth 10 |
| `scan` | `/sensors/scan_raw` | `sensor_msgs/msg/LaserScan` | `lidar_frame_id` | `lidar.scan_rate` | depth 5 |
| `imu` | `/sensors/imu` | `sensor_msgs/msg/Imu` | `imu_frame_id` | `imu.sample_rate` | depth 10 |
| `magnetometer` | `/sensors/magnetometer` | `sensor_msgs/msg/MagneticField` | `imu_frame_id` | `imu.sample_rate` | depth 10 |
| `encoder_data` | `/svdsim/encoder_data` | `svdsim_msgs/msg/StampedEncoderData` | - | `odometry_rate` | depth 10 |
| `steering_feedback` | `/svdsim/steering_feedback` | `svdsim_msgs/msg/StampedSteering` | - | `odometry_rate` | depth 10 |
| `ground_truth` | `/sensors/groundtruth` | `nav_msgs/msg/Odometry` | `map_frame_id` → `base_link_frame_id` | `ground_truth_rate` | depth 10 |
| `collision_data` | `collision_data` (not remapped) | `svdsim_msgs/msg/CollisionData` | - | `command_rate` | transient local, depth 10 |
| `sparring/markers` | `sparring/markers` (not remapped) | `visualization_msgs/msg/MarkerArray` | `map_frame_id` | 10 Hz (wall timer) | depth 10 |

### Subscribed

| Topic | Remapped name | Type | Frame | Rate | QoS |
|---|---|---|---|---|---|
| `steering_command` | `/controls/steering_command` | `svdsim_msgs/msg/StampedSteering` | - | on message | reliable, depth 10 |
| `velocity_command` | `/controls/velocity_command` | `svdsim_msgs/msg/StampedVelocity` | - | on message | reliable, depth 10 |
| `initialpose` | `initialpose` (not remapped) | `geometry_msgs/msg/PoseWithCovarianceStamped` | `map_frame_id` | on message | reliable, depth 10 |

Each subscription forwards its payload as the matching inbound protobuf message: `steering` [rad] as
a `SteeringCommand`, `velocity` [m/s] as a `VelocityCommand`, and `initialpose` as a
`ResetVehicleState`. Messages on `initialpose` whose `frame_id` differs from `map_frame_id` are
rejected with an error log. When the node is started directly with `ros2 run` (see below), the
node-local names apply, not the remapped ones.

## Message content details

Field definitions and the `time_increment` caveat are given in
[LiDAR parameters](svdsim_server/lidar.md#published-message).

### IMU

`linear_acceleration` and `angular_velocity` hold the raw sensor values. Orientation is not
available: `orientation_covariance[0]` is set to `-1` to announce that the field must be ignored.
All other covariance entries are zero, so consumers must supply their own noise model (which is what
a downstream odometry fusion node does).

### Odometry (ground truth)

`pose` is the `base_link` pose in the map frame and `twist` is expressed in `base_link`, matching
`nav_msgs/msg/Odometry` conventions. Both covariances remain zero: the ground truth is noise free
and must not be fused as if it were a measurement.

## Clock and `use_sim_time`

The bridge publishes the simulation time as `/clock`. Any node that should run on simulation time
must be launched with `use_sim_time:=true`; the bringup does this for the whole stack. Because the
server only advances the clock while it is running, a paused or crashed server freezes the ROS time
of every consumer, which is the intended behaviour for reproducible experiments.

Because the server emits one `BroadcastTimestamp` payload per activation of `RemoteControlSystem`,
the bridge publishes one `/clock` message per payload.

## Sparring markers

Marker updates are cached in two maps keyed by entity id and republished by a wall timer, so the
visualisation keeps refreshing even when the server stops sending markers (for example after a
reset). The `sparring_pose` namespace contains one arrow per sparring vehicle, the
`sparring_lookahead` namespace one sphere per sparring vehicle at the pure-pursuit target point.

## Running more than one vehicle

One bridge instance serves exactly one vehicle. For a multi-vehicle scenario, start one bridge per
remote vehicle with a distinct `port`, and distinct frame ids when the URDF describes several cars:

```bash
ros2 run svdsim_bridge svdsim_bridge_node --ros-args \
  -p port:=9999 -p lidar_frame_id:=laser -p map_frame_id:=map
ros2 run svdsim_bridge svdsim_bridge_node --ros-args \
  -p port:=10000 -p lidar_frame_id:=laser_2 -p map_frame_id:=map
```

Sparring vehicles do not need a bridge; they are driven entirely inside the server.

## See also

- [SVD simulator overview](README.md)
- [svdsim_server parameter reference](svdsim_server/README.md)
- [Message definitions](svdsim_msgs.md)
- [Protobuf definitions](svdsim_proto.md)
