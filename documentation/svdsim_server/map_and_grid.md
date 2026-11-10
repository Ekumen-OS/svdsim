# Map and Grid Configuration

**Configuration section:** `map_path`, `grid_listing_file` (top level)
**Source:** `svdsim_server/src/utils/map_loader.cpp`, `svdsim_server/src/configuration/grid_csv_reader.cpp`

This section defines the static world and the vehicles that are spawned in it.

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `map_path` | string | `""` | Absolute path to a ROS-style map YAML file, including the `.yaml` extension. Must be non-empty. |
| `grid_listing_file` | string | `""` | Absolute path to the CSV file that lists the spawned vehicles. Must be non-empty. |

The node receives plain absolute paths, and no preprocessing is applied to them:

```yaml
svdsim_server:
  ros__parameters:
    map_path: "/path/to/maps/hq_street_track_cw/map.yaml"
    grid_listing_file: "/path/to/config/vehicle_grid.csv"
```

## Map file format

The loader accepts the standard ROS map YAML layout:

```yaml
image: map.pgm
mode: trinary
resolution: 0.05
origin: [-18.2, -5.2, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

| Key | Required | Default | Meaning |
|---|---|---|---|
| `image` | yes | - | Image file name, resolved relative to the YAML file. |
| `resolution` | yes | - | Cell size [m/cell]. |
| `origin` | yes | - | Pose `[x, y, theta]` of the lower-left image corner in the world frame. Three values are mandatory. |
| `negate` | no | `0` | `0`/`false`: black pixels are obstacles. `1`/`true`: white pixels are obstacles. Integers and booleans are both accepted. |
| `occupied_thresh` | no | `0.65` | A cell is an obstacle when the computed occupancy probability is greater than or equal to this value. |
| `free_thresh` | no | `0.196` | Cells at or below this probability are free. Cells between `free_thresh` and `occupied_thresh` are treated as free. |
| `mode` | no | - | Ignored by the loader; kept for compatibility with tools such as Nav2 map server. |

Occupancy conversion follows the ROS convention:

$$
p = \begin{cases}
1 - \dfrac{v}{255} & \text{if } \mathit{negate} = 0 \\[6pt]
\dfrac{v}{255} & \text{if } \mathit{negate} = 1
\end{cases}
$$

where $v \in [0, 255]$ is the grayscale pixel value.

The map is loaded once at startup. The loader throws on a missing file, a missing required key, an
`origin` field with a length other than three, or an unreadable image; see
[Validation rules](README.md#validation-rules).

## Vehicle grid file

The grid file is a CSV document with a mandatory header row:

```csv
id,initx,inity,inityaw,port,type
vehicle1,0.0,0.0,0.0,9999,remote
```

| Column | Type | Unit | Meaning |
|---|---|---|---|
| `id` | string | - | Vehicle identifier, used in logs and markers. |
| `initx` | double | m | Initial `x` position of `base_link` in the world frame. |
| `inity` | double | m | Initial `y` position of `base_link` in the world frame. |
| `inityaw` | double | rad | Initial yaw of `base_link`. |
| `port` | int | - | TCP port of the vehicle's ZeroMQ control server, [1, 65535]; validated for every row, and a server is opened only for `remote` vehicles. |
| `type` | string | - | `remote` for a vehicle controlled over ZeroMQ, `sparring` for an autonomous vehicle that follows a trajectory. |

Rules enforced by the reader:

- The file must contain a header row with the six columns in the order shown above and exactly six
  fields per row; a file without a header or without data rows is rejected.
- All numeric fields must be finite; `nan` and `inf` are rejected.
- `id` must be non-empty and consist of alphanumeric characters, `_` or `-`; `id` values must be
  unique across the file.
- `port` must be unique across the file, including sparring rows.
- Every sparring vehicle requires `sparring.trajectory_file` to be non-empty; the server throws
  otherwise.

The `port` value must match the `port` parameter of the bridge that drives the vehicle
(`svdsim_bridge` defaults to 4242; use the same value in both files). One bridge connects to one vehicle;
additional remote vehicles need additional bridge instances with different ports and frame IDs.

## Example: adding a sparring vehicle

```csv
id,initx,inity,inityaw,port,type
vehicle1,0.0,0.0,0.0,9999,remote
sparring_1,5.0,0.0,0.0,9998,sparring
```

The trajectory is shared by all sparring vehicles and is set once through
`sparring.trajectory_file`; see [Sparring vehicle parameters](sparring.md).

## See also

- [Simulation rates](simulation_rates.md)
- [Sparring vehicle parameters](sparring.md)
- [Vehicle footprint parameters](vehicle_footprint.md)
