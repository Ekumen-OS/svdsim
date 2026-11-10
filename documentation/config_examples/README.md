# Configuration examples

Reference YAML files for the nodes of the SVD simulator. Each file lists **every parameter the node
declares, set to its default value**, so it doubles as a template and as a record of the defaults
documented in the parameter tables.

| File | Node | Contents |
|------|------|----------|
| [`svdsim_server_default.yaml`](svdsim_server_default.yaml) | `svdsim_server_node` | All `svdsim_server` parameters at their defaults |
| [`svdsim_bridge_default.yaml`](svdsim_bridge_default.yaml) | `svdsim_bridge_node` | All `svdsim_bridge` parameters at their defaults |
| [`vehicle_grid.csv`](vehicle_grid.csv) | `svdsim_server_node` | Example vehicle grid for `grid_listing_file` |

## Using the examples

Copy the files next to the rest of your simulation assets and pass them with `--params-file`:

```bash
ros2 run svdsim_server svdsim_server_node --ros-args \
  --params-file /path/to/svdsim_server_default.yaml

ros2 run svdsim_bridge svdsim_bridge_node --ros-args \
  --params-file /path/to/svdsim_bridge_default.yaml
```

Both nodes can also be launched as composable nodes and loaded into a container; the parameter file
is passed to the container in the same way.

## Before the first launch

`map_path` and `grid_listing_file` are declared read-only and must be non-empty, so the example does
not start as provided. Set both to absolute paths:

```yaml
svdsim_server:
  ros__parameters:
    map_path: "/path/to/map.yaml"
    grid_listing_file: "/path/to/vehicle_grid.csv"
```

The map file follows the `map_server` layout (`image`, `resolution`, `origin`, and optionally
`negate`, `occupied_thresh`, `free_thresh`). The grid file format is shown in
[`vehicle_grid.csv`](vehicle_grid.csv) and described in
[Map and grid configuration](../svdsim_server/map_and_grid.md).

Requests for sparring vehicles additionally require `sparring.trajectory_file`.

## Rate constraints

`command_rate`, `odometry_rate`, `ground_truth_rate`, `imu.sample_rate` and `lidar.scan_rate` must be
integer divisors of `physics_rate`. With the defaults (`physics_rate: 1000.0`) that is satisfied, but
a custom `physics_rate` requires matching adjustments. See
[Simulation rates](../svdsim_server/simulation_rates.md).

## Relationship to the parameter tables

The "Default" column of every parameter table in this documentation is taken from the same node
declarations as these files. When a parameter is changed in the source, both the table and the
example must be updated together.
