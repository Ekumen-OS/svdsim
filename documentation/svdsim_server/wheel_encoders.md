# Wheel Encoder Parameters

**Configuration sections:** `wheel_encoders`, plus the top-level `use_true_steering_feedback_sensor`
**Source:** `svdsim_server/models/pulse_encoder_model.cpp`, `svdsim_server/ecs/systems/odometry_sensor_system.cpp`

The rear wheels are instrumented with pulse encoders that integrate the simulated wheel rotation into
an absolute pulse count streamed to the bridge.

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `wheel_encoders.pulses_per_revolution` | double | 4096.0 | Encoder pulses per wheel revolution. |
| `wheel_encoders.conversion_ratio` | double | 1.0 | Gear or conversion ratio between the encoder shaft and the wheel. |
| `use_true_steering_feedback_sensor` | bool | `true` | When true, the reported steering angle is the simulated wheel angle. When false, the last commanded steering reference is echoed. |

## Tick computation

Each rear wheel keeps a cumulative count of full rotations $n_{cycles}$ and a raw angle $\varphi$
that lies in $[0, 2\pi)$ while the wheel turns forward. The encoder output is

$$
\text{ticks} = \operatorname{round}\!\left(\left(n_{cycles} + \frac{\varphi}{2\pi}\right) \cdot r \cdot \text{ppr}\right)
$$

with $r$ the `conversion_ratio` and $\text{ppr}$ the `pulses_per_revolution`. Every update folds
$\operatorname{trunc}(\varphi/2\pi)$ turns into $n_{cycles}$; because `trunc` rounds towards zero, a
negative raw angle is never folded and remains in $(-2\pi, 0]$. Reverse motion is therefore
represented by a fractional term that reduces `ticks` without decrementing $n_{cycles}$.

The quantisation step is far below the per-step motion at racing speed.

## Notes

- The encoder rate is `odometry_rate`, which must be non-zero; see [Simulation rates](simulation_rates.md).
- Counts are published as `svdsim_msgs/msg/StampedEncoderData`; the field semantics are documented in
  [Message definitions](../svdsim_msgs.md).
- `pulses_per_revolution` scales the tick value and therefore sets the resolution of the counts. The
  default value matches the VESC/Hall setup of the RoboRacer so that the same odometry calibration
  can be reused.
- The parameter pair does not introduce sensor noise. If encoder noise is required for estimator
  testing, it has to be modelled downstream.
- Keep `use_true_steering_feedback_sensor` equal to the value used in the realbot configuration when
  the goal is to validate a controller that will be deployed on hardware.

## See also

- [Vehicle dynamics parameters](vehicle_dynamics.md)
- [Bridge parameters](../svdsim_bridge.md)
