# SVD Simulator Documentation

The SVD simulator (`svd_simulator/`) is a lightweight 2D simulator for autonomous racing. It is
composed of four packages.

This folder documents the **parameters and interfaces** of those packages. Each package has its own
page, and the server parameters are split along the sections of the configuration file so that a
parameter can be found by the block it belongs to; they are indexed in the
[section index](svdsim_server/README.md#configuration-sections).

## Package map

| Package | Page | Role |
|---|---|---|
| `svdsim_server` | [Parameter reference](svdsim_server/README.md) | Physics, vehicle dynamics, sensor models, ZeroMQ endpoints |
| `svdsim_bridge` | [Parameters and topics](svdsim_bridge.md) | ROS 2 adapter for one vehicle |
| `svdsim_msgs` | [Messages](svdsim_msgs.md) | Custom ROS 2 message definitions |
| `svdsim_proto` | [Protobuf definitions](svdsim_proto.md) | ZeroMQ wire format shared by server and bridge |

For the architecture overview, the model bibliography and the feature list, see
[`svd_simulator/README.md`](../../svd_simulator/README.md).

## How to read the parameter tables

Every parameter table has the same columns:

| Column | Meaning |
|---|---|
| **Parameter** | Fully qualified ROS parameter name as used in `ros__parameters`. |
| **Type** | ROS parameter type: `double`, `bool`, `int` or `string`. |
| **Default** | Value declared in the node source. A key that is absent from a parameter file keeps this value. |
| **Description** | Semantics and units. Units are bracketed, for example `[rad]` or `[m/s]`. |

## Configuration files

Both nodes read standard ROS 2 parameter files. Complete examples with every parameter listed at its
default value are provided in [`config_examples/`](config_examples/README.md):

| File | Node |
|---|---|
| [`config_examples/svdsim_server_default.yaml`](config_examples/svdsim_server_default.yaml) | `svdsim_server_node` |
| [`config_examples/svdsim_bridge_default.yaml`](config_examples/svdsim_bridge_default.yaml) | `svdsim_bridge_node` |

## Quick start

```bash
# Simulation server
ros2 run svdsim_server svdsim_server_node --ros-args \
  --params-file documentation/svd_simulator/config_examples/svdsim_server_default.yaml

# ROS 2 bridge for one vehicle
ros2 run svdsim_bridge svdsim_bridge_node --ros-args \
  --params-file documentation/svd_simulator/config_examples/svdsim_bridge_default.yaml
```

`map_path` and `grid_listing_file` are required and empty in the examples, so both must be set to
absolute paths before the first launch. The grid file format and the map layout are described in
[Map and grid](svdsim_server/map_and_grid.md).

## Regenerating the figures

The plots in these pages are produced from the same equations that the simulator implements, using the
parameter values of a representative example platform rather than the node defaults:

```bash
python3 documentation/svd_simulator/figures/generate_figures.py
```

The script requires Python 3 with `numpy` and `matplotlib` and writes the PNG files next to itself in
`figures/`. Update the constants at the top of the script when the example platform changes, then
re-run it so that the documentation does not drift from the figures.

## Reporting documentation gaps

If a parameter is missing here, check `svdsim_server/src/svdsim_server_node.cpp`, function
`declare_parameters()`, which is the authoritative list of declared parameters and their ranges. The
documentation is intended to cover every parameter declared there; a divergence is a bug in this
documentation.
