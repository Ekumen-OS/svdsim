#!/usr/bin/env python3

# Copyright 2026 Ekumen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate the figures used by the SVD simulator parameter documentation.

All curves are computed with the same equations that are implemented in
``svdsim_server`` and with the parameter values of a representative 1/10-scale
example platform. The values are deliberately different from the node defaults
where the default is degenerate for an illustration (for example a non-zero
steering hysteresis gap). Keeping the computation here, next to the
documentation, guarantees that the plots can be reproduced after a parameter
change.

Usage (from the repository root):

    python3 documentation/svd_simulator/figures/generate_figures.py
"""

import os

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

OUTPUT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))

# Representative example platform (not the node defaults).
TIRE_LONGITUDINAL = {"Dn": 0.72, "B": 8.0, "C": 2.1, "E": 0.97}
TIRE_LATERAL = {"Dn": 1.0, "B": 0.1, "C": 1.7, "E": 0.5}

WHEELBASE = 0.32
FRONT_TRACK_WIDTH = 0.236
REAR_TRACK_WIDTH = 0.236
WHEEL_LENGTH = 0.06
WHEEL_WIDTH = 0.045
FRONT_OVERHANG = 0.09
REAR_OVERHANG = 0.08
EFFECTIVE_WHEEL_RADIUS = 0.0535
SPRUNG_MASS = 3.47
CENTER_OF_MASS_X = 0.15532
CENTER_OF_MASS_Z = EFFECTIVE_WHEEL_RADIUS + 0.01434
ROLL_CENTER_HEIGHT = EFFECTIVE_WHEEL_RADIUS
SPRING_STIFFNESS = 500.0
MAX_INCLINATION_ANGLE = 0.174533

STEERING_MAX = 0.40
STEERING_VELOCITY_MAX = 3.2
STEERING_HYSTERESIS_GAP = 0.13
VELOCITY_MAX = 6.0
VELOCITY_MIN = -2.0
ACCELERATION_MAX = 20.0
BRAKING_DECELERATION = 20.0
MOTOR_DRIVE_RISE_TIME = 0.1
PHYSICS_RATE = 1000.0
SENSOR_RATE = 40.0

IMU_ACCELEROMETER_NOISE_DENSITY = 1.40e-02
IMU_GYROSCOPE_NOISE_DENSITY = 1.75e-03
IMU_ACCELEROMETER_TAU = 3600.0
IMU_MAGNETOMETER_TAU = 300.0

LIDAR_HORIZONTAL_FOV = 4.712
LIDAR_HORIZONTAL_RESOLUTION = 0.00436
LIDAR_SCAN_RATE = 40.0
LIDAR_MIN_DISTANCE = 0.2
LIDAR_MAX_DISTANCE = 10.0

SPARRING_LOOKAHEAD_DISTANCE = 0.8
SPARRING_MAX_STEERING = 0.6

# Sensor mounting offsets from base_link.
IMU_X_OFFSET = 0.08
IMU_Z_OFFSET = EFFECTIVE_WHEEL_RADIUS + 0.055
LIDAR_X_OFFSET = 0.2733

plt.rcParams.update(
    {
        "figure.dpi": 200,
        "savefig.dpi": 200,
        "savefig.bbox": "tight",
        "font.size": 9,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "legend.frameon": False,
        "lines.linewidth": 1.6,
    }
)


def save(figure, name):
    """Save a figure as PNG and close it."""
    path = os.path.join(OUTPUT_DIRECTORY, name)
    figure.savefig(path)
    plt.close(figure)
    print(f"wrote {path}")


def pacejka(params, x, vertical_load):
    """Pacejka Magic Formula, F = D sin(C atan(Bx - E(Bx - atan(Bx))))."""
    D = params["Dn"] * vertical_load
    Bx = params["B"] * x
    return D * np.sin(params["C"] * np.arctan(Bx - params["E"] * (Bx - np.arctan(Bx))))


def pacejka_stiffness(params, vertical_load):
    """Slope of the Magic Formula at the origin, dF/dx at x = 0."""
    return params["B"] * params["C"] * params["Dn"] * vertical_load


def plot_pacejka_longitudinal():
    slip_ratio = np.linspace(-1.0, 1.0, 801)
    figure, axes = plt.subplots(figsize=(5.2, 3.2))
    loads = (10.0, 25.0, 50.0)
    colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(loads)))
    for load, color in zip(loads, colors):
        axes.plot(slip_ratio, pacejka(TIRE_LONGITUDINAL, slip_ratio, load), color=color, label=f"$F_z$ = {load:.0f} N")
    axes.axhline(0.0, color="black", linewidth=0.8)
    axes.set_xlabel("Longitudinal slip ratio $s$ [-]")
    axes.set_ylabel("Longitudinal force $F_x$ [N]")
    axes.set_title("Longitudinal Pacejka curve ($D_n$=0.72, B=8, C=2.1, E=0.97)")
    axes.legend(title="Vertical load")
    save(figure, "pacejka_longitudinal.png")


def plot_pacejka_lateral():
    slip_angle_deg = np.linspace(-20.0, 20.0, 801)
    figure, axes = plt.subplots(figsize=(5.2, 3.2))
    loads = (10.0, 25.0, 50.0)
    colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(loads)))
    for load, color in zip(loads, colors):
        axes.plot(slip_angle_deg, pacejka(TIRE_LATERAL, slip_angle_deg, load), color=color, label=f"$F_z$ = {load:.0f} N")
    axes.axhline(0.0, color="black", linewidth=0.8)
    axes.set_xlabel("Slip angle $\\alpha$ [deg]")
    axes.set_ylabel("Lateral force $F_y$ [N]")
    axes.set_title("Lateral Pacejka curve ($D_n$=1.0, B=0.1, C=1.7, E=0.5)")
    axes.legend(title="Vertical load")
    save(figure, "pacejka_lateral.png")


def mnc_combined_forces(slip_ratio, slip_angle, vertical_load):
    """Modified Nicolas-Comstock combined slip forces for scalar inputs."""
    alpha_deg = np.rad2deg(slip_angle)
    f0x = pacejka(TIRE_LONGITUDINAL, slip_ratio, vertical_load)
    f0y = pacejka(TIRE_LATERAL, alpha_deg, vertical_load)

    # The MNC formulation is undefined when either pure force vanishes; the model
    # implementation falls back to the pure slip forces in that case.
    if abs(f0x) < 1e-9 or abs(f0y) < 1e-9:
        return f0x, f0y

    c_s = pacejka_stiffness(TIRE_LONGITUDINAL, vertical_load)
    c_alpha = pacejka_stiffness(TIRE_LATERAL, vertical_load) * 180.0 / np.pi

    cos_alpha = np.cos(slip_angle)
    sin_alpha = np.sin(slip_angle)
    tan_alpha = np.tan(slip_angle) if abs(cos_alpha) > 1e-6 else np.copysign(1e6, sin_alpha)
    base = np.sqrt((slip_ratio * f0y) ** 2 + (f0x * tan_alpha) ** 2)

    fx = f0x * f0y * np.sqrt((slip_ratio * c_alpha) ** 2 + (cos_alpha * f0x) ** 2) / (base * c_alpha)
    fy = (
        f0x
        * f0y
        * np.sqrt(((1.0 - slip_ratio) * cos_alpha * f0y) ** 2 + (sin_alpha * c_s) ** 2)
        / (base * c_s * cos_alpha)
    )
    return fx, fy


def plot_mnc_combined_slip():
    slip_ratio = np.linspace(-0.6, 0.6, 601)
    vertical_load = 25.0
    slip_angle = np.deg2rad(6.0)

    pure_fx = pacejka(TIRE_LONGITUDINAL, slip_ratio, vertical_load)
    pure_fy = np.full_like(slip_ratio, pacejka(TIRE_LATERAL, np.rad2deg(slip_angle), vertical_load))
    combined_fx = np.array([mnc_combined_forces(s, slip_angle, vertical_load)[0] for s in slip_ratio])
    combined_fy = np.array([mnc_combined_forces(s, slip_angle, vertical_load)[1] for s in slip_ratio])

    figure, axes = plt.subplots(1, 2, figsize=(7.6, 3.0))
    axes[0].plot(slip_ratio, pure_fx, color="#1f77b4", label="Pure slip")
    axes[0].plot(slip_ratio, combined_fx, color="#d62728", linestyle="--", label="MNC combined")
    axes[0].set_xlabel("Longitudinal slip ratio $s$ [-]")
    axes[0].set_ylabel("$F_x$ [N]")
    axes[0].set_title("Longitudinal force")
    axes[0].legend()

    axes[1].plot(slip_ratio, pure_fy, color="#1f77b4", label="Pure slip")
    axes[1].plot(slip_ratio, combined_fy, color="#d62728", linestyle="--", label="MNC combined")
    axes[1].set_xlabel("Longitudinal slip ratio $s$ [-]")
    axes[1].set_ylabel("$F_y$ [N]")
    axes[1].set_title("Lateral force")
    axes[1].legend()

    figure.suptitle(
        f"Combined slip interaction at $\\alpha$ = 6 deg, $F_z$ = {vertical_load:.0f} N", fontsize=9
    )
    save(figure, "mnc_combined_slip.png")


def plot_vehicle_geometry():
    """Top view of the example platform: chassis, collision footprint and sensor offsets."""
    half_track = FRONT_TRACK_WIDTH / 2.0
    half_body = 0.145
    footprint_front = WHEELBASE + FRONT_OVERHANG
    footprint_rear = REAR_OVERHANG
    footprint_width = FRONT_TRACK_WIDTH + 2.0 * WHEEL_WIDTH
    footprint_half_width = footprint_width / 2.0

    figure, axes = plt.subplots(figsize=(6.8, 4.0))

    # Collision footprint and chassis outline.
    axes.add_patch(
        plt.Rectangle(
            (-footprint_rear, -footprint_half_width),
            footprint_front + footprint_rear,
            2.0 * footprint_half_width,
            fill=False,
            linestyle=(0, (1.5, 1.5)),
            linewidth=1.4,
            edgecolor="#7f7f7f",
            label="Collision footprint",
        )
    )
    axes.add_patch(
        plt.Rectangle(
            (-REAR_OVERHANG, -half_body),
            WHEELBASE + FRONT_OVERHANG + REAR_OVERHANG,
            2.0 * half_body,
            fill=False,
            linewidth=1.0,
            edgecolor="#1f77b4",
            label="Chassis outline",
        )
    )

    # Wheels.
    for x_center in (0.0, WHEELBASE):
        for y_center in (half_track, -half_track):
            axes.add_patch(
                plt.Rectangle(
                    (x_center - WHEEL_LENGTH / 2.0, y_center - WHEEL_WIDTH / 2.0),
                    WHEEL_LENGTH,
                    WHEEL_WIDTH,
                    color="#4c4c4c",
                    zorder=3,
                )
            )

    # Centerline.
    axes.plot(
        [-REAR_OVERHANG, WHEELBASE + FRONT_OVERHANG],
        [0.0, 0.0],
        color="#1f77b4",
        linewidth=0.8,
        linestyle=(0, (4, 2)),
        zorder=2,
    )

    # Wheelbase dimension below the vehicle.
    axes.annotate(
        "",
        xy=(0.0, -0.205),
        xytext=(WHEELBASE, -0.205),
        arrowprops={"arrowstyle": "<->", "color": "#1f77b4", "linewidth": 1.0},
    )
    axes.text(WHEELBASE / 2.0, -0.225, f"wheelbase = {WHEELBASE:.2f} m", ha="center", fontsize=8)

    # Track width dimension at the right.
    dimension_x = 0.465
    axes.annotate(
        "",
        xy=(dimension_x, -half_track),
        xytext=(dimension_x, half_track),
        arrowprops={"arrowstyle": "<->", "color": "#2ca02c", "linewidth": 1.0},
    )
    axes.text(
        dimension_x + 0.018,
        0.0,
        f"track = {FRONT_TRACK_WIDTH:.3f} m",
        va="center",
        rotation=90,
        fontsize=8,
        color="#2ca02c",
    )

    # base_link origin.
    axes.plot(0.0, 0.0, marker="o", color="black", markersize=5, zorder=4)
    axes.annotate(
        "base_link\n(rear axle, z = 0)",
        (0.0, 0.0),
        xytext=(-0.33, -0.07),
        fontsize=8,
        arrowprops={"arrowstyle": "->", "color": "black", "linewidth": 0.8},
    )

    # Sensor and centre-of-mass markers on the centerline.
    axes.plot(CENTER_OF_MASS_X, 0.0, marker="X", color="#d62728", markersize=8, zorder=5)
    axes.text(CENTER_OF_MASS_X, 0.058, "CoM", ha="center", fontsize=8, color="#d62728")
    axes.plot(IMU_X_OFFSET, 0.0, marker="s", color="#ff7f0e", markersize=6, zorder=5)
    axes.text(IMU_X_OFFSET, -0.082, "IMU", ha="center", fontsize=8, color="#ff7f0e")
    axes.plot(LIDAR_X_OFFSET, 0.0, marker="^", color="#9467bd", markersize=8, zorder=5)
    axes.text(LIDAR_X_OFFSET, 0.058, "LiDAR", ha="center", fontsize=8, color="#9467bd")

    # Offset summary.
    summary = (
        "Offsets from base_link:\n"
        f"CoM    x = {CENTER_OF_MASS_X:.3f} m, z = {CENTER_OF_MASS_Z:.3f} m\n"
        f"IMU    x = {IMU_X_OFFSET:.3f} m, z = {IMU_Z_OFFSET:.3f} m\n"
        f"LiDAR  x = {LIDAR_X_OFFSET:.4f} m"
    )
    axes.text(
        -0.335,
        0.325,
        summary,
        fontsize=7.5,
        va="top",
        family="monospace",
        bbox={"boxstyle": "round,pad=0.35", "fc": "white", "ec": "#bbbbbb"},
    )

    # Footprint values.
    axes.text(
        (footprint_front - footprint_rear) / 2.0,
        -0.275,
        f"footprint: front_distance = {footprint_front:.2f} m, "
        f"rear_distance = {footprint_rear:.2f} m, width = {footprint_width:.3f} m",
        ha="center",
        fontsize=7.5,
        color="#666666",
    )

    axes.set_xlim(-0.35, 0.56)
    axes.set_ylim(-0.32, 0.34)
    axes.set_aspect("equal")
    axes.set_xlabel("x, vehicle frame [m]")
    axes.set_ylabel("y, vehicle frame [m]")
    axes.set_title("Vehicle frame, geometry and sensor offsets (example platform)")
    axes.legend(loc="upper right", fontsize=8)
    save(figure, "vehicle_geometry.png")


def steering_model_equilibrium(reference, steering, dt):
    """One step of the SimpleDriveModel steering law."""
    reference = float(np.clip(reference, -STEERING_MAX, STEERING_MAX))
    gap_half = STEERING_HYSTERESIS_GAP / 2.0
    error_to_reference = reference - steering
    if error_to_reference > 0.0:
        adjusted = max(reference - gap_half, steering)
    else:
        adjusted = min(reference + gap_half, steering)
    velocity = (adjusted - steering) / dt
    velocity = float(np.clip(velocity, -STEERING_VELOCITY_MAX, STEERING_VELOCITY_MAX))
    velocity_to_max = (STEERING_MAX - steering) / dt
    velocity_to_min = (-STEERING_MAX - steering) / dt
    velocity = float(np.clip(velocity, velocity_to_min, velocity_to_max))
    return steering + velocity * dt


def plot_steering_hysteresis():
    dt = 1.0 / PHYSICS_RATE
    duration = 6.0
    steps = int(duration / dt)
    reference_profile = np.concatenate(
        [
            np.linspace(0.0, STEERING_MAX, steps // 2),
            np.linspace(STEERING_MAX, -STEERING_MAX, steps),
            np.linspace(-STEERING_MAX, 0.0, steps // 2),
        ]
    )

    figure, axes = plt.subplots(figsize=(5.2, 3.4))
    steering = 0.0
    achieved = []
    for reference in reference_profile:
        steering = steering_model_equilibrium(reference, steering, dt)
        achieved.append(steering)
    achieved = np.asarray(achieved)

    axes.plot(reference_profile, achieved, color="#d62728", label="Achieved steering")
    axes.plot(
        [-STEERING_MAX, STEERING_MAX],
        [-STEERING_MAX, STEERING_MAX],
        color="black",
        linestyle="--",
        linewidth=1.0,
        label="Ideal (achieved = reference)",
    )
    axes.set_xlabel("Commanded steering angle [rad]")
    axes.set_ylabel("Achieved steering angle [rad]")
    axes.set_title(
        f"Steering hysteresis loop, gap = {STEERING_HYSTERESIS_GAP:.2f} rad\n"
        f"(steady-state lag = gap/2 = {STEERING_HYSTERESIS_GAP / 2.0:.3f} rad)"
    )
    axes.legend(loc="upper left")
    save(figure, "steering_hysteresis.png")


def smoothing_filter_step(rise_time, dt, duration):
    """Step response of SecondOrderSmoothingFilter for a unit step."""
    alpha = 0.781616
    omega_n = alpha / rise_time
    kp = 2.0 * omega_n
    ki = omega_n * omega_n
    denominator = 1.0 + kp * dt + ki * dt * dt
    a1 = (2.0 + kp * dt) / denominator
    a2 = -1.0 / denominator
    b0 = (kp * dt + ki * dt * dt) / denominator
    b1 = -(kp * dt) / denominator

    w1 = 0.0
    w2 = 0.0
    samples = int(duration / dt)
    output = np.zeros(samples + 1)
    time = np.arange(samples + 1) * dt
    for index in range(1, samples + 1):
        w = 1.0 + a1 * w1 + a2 * w2
        output[index] = b0 * w + b1 * w1
        w2 = w1
        w1 = w
    return time, output


def plot_velocity_reference_filter():
    figure, axes = plt.subplots(figsize=(5.2, 3.2))
    for rise_time, color in ((0.02, "#2ca02c"), (0.1, "#1f77b4"), (0.3, "#d62728")):
        time, response = smoothing_filter_step(rise_time, 1.0 / PHYSICS_RATE, 1.0)
        axes.plot(time * 1000.0, response, color=color, label=f"rise time = {rise_time:.2f} s")
    axes.axhline(1.0, color="black", linestyle="--", linewidth=1.0)
    axes.axhline(0.9, color="#7f7f7f", linestyle=":", linewidth=1.0)
    axes.set_xlabel("time [ms]")
    axes.set_ylabel("Smoothed reference / commanded reference [-]")
    axes.set_title(
        f"Velocity reference smoothing, configured rise time = {MOTOR_DRIVE_RISE_TIME:.2f} s"
    )
    axes.legend(loc="lower right")
    save(figure, "velocity_ref_filter.png")


def plot_imu_noise():
    figure, axes = plt.subplots(1, 2, figsize=(7.6, 3.0))

    rates = np.logspace(0.0, 3.0, 200)
    axes[0].loglog(
        rates,
        IMU_ACCELEROMETER_NOISE_DENSITY / np.sqrt(1.0 / rates),
        linestyle="--",
        label="accelerometer, hypothetical",
    )
    axes[0].loglog(
        rates,
        IMU_GYROSCOPE_NOISE_DENSITY / np.sqrt(1.0 / rates),
        linestyle="--",
        label="gyroscope, hypothetical",
    )
    axes[0].axhline(
        IMU_ACCELEROMETER_NOISE_DENSITY,
        color="#1f77b4",
        linestyle="-.",
        linewidth=1.0,
        label="accelerometer, implemented: dt = 1 s",
    )
    axes[0].axhline(
        IMU_GYROSCOPE_NOISE_DENSITY,
        color="#ff7f0e",
        linestyle="-.",
        linewidth=1.0,
        label="gyroscope, implemented: dt = 1 s",
    )
    axes[0].axvline(SENSOR_RATE, color="#7f7f7f", linestyle=":", linewidth=1.0)
    axes[0].text(SENSOR_RATE * 1.05, 2e-1, f"{SENSOR_RATE:.0f} Hz", fontsize=8, color="#7f7f7f")
    axes[0].set_xlabel("sample rate [Hz]")
    axes[0].set_ylabel("discrete white noise std. dev. [-]")
    axes[0].set_title("Discrete white noise")
    axes[0].set_ylim(5e-4, 1.0)
    axes[0].legend(fontsize=6.5, loc="lower right", framealpha=0.9)

    time = np.linspace(0.0, 3.0 * IMU_ACCELEROMETER_TAU, 400)
    axes[1].plot(
        time / 3600.0,
        np.exp(-time / IMU_ACCELEROMETER_TAU),
        label=f"$\\tau$ = {IMU_ACCELEROMETER_TAU:.0f} s",
    )
    time = np.linspace(0.0, 3.0 * IMU_MAGNETOMETER_TAU, 400)
    axes[1].plot(
        time / 3600.0,
        np.exp(-time / IMU_MAGNETOMETER_TAU),
        label=f"$\\tau$ = {IMU_MAGNETOMETER_TAU:.0f} s",
    )
    axes[1].set_xlabel("time [h]")
    axes[1].set_ylabel("bias autocorrelation $e^{-t/\\tau}$ [-]")
    axes[1].set_title("Gauss-Markov bias decay")
    axes[1].legend()

    figure.tight_layout()
    save(figure, "imu_noise.png")


def plot_lidar_scan_pattern():
    points_per_scan = int(LIDAR_HORIZONTAL_FOV / LIDAR_HORIZONTAL_RESOLUTION) + 1
    angles = -LIDAR_HORIZONTAL_FOV / 2.0 + np.arange(points_per_scan) * (
        LIDAR_HORIZONTAL_FOV / (points_per_scan - 1)
    )
    timestamps_per_ray = (PHYSICS_RATE / LIDAR_SCAN_RATE) * (
        LIDAR_HORIZONTAL_FOV / (2.0 * np.pi)
    ) / (points_per_scan - 1)

    figure = plt.figure(figsize=(7.6, 3.4))
    polar = figure.add_subplot(1, 2, 1, projection="polar")
    polar.plot(angles, np.full_like(angles, LIDAR_MAX_DISTANCE), ".", markersize=1.2, color="#1f77b4")
    polar.plot(angles, np.full_like(angles, LIDAR_MIN_DISTANCE), ".", markersize=1.2, color="#d62728")
    polar.set_ylim(0.0, LIDAR_MAX_DISTANCE * 1.05)
    polar.set_title(
        f"Beam pattern: {points_per_scan} beams,\n"
        f"FOV = {np.rad2deg(LIDAR_HORIZONTAL_FOV):.0f} deg, "
        f"step = {np.rad2deg(LIDAR_HORIZONTAL_FOV / (points_per_scan - 1)):.3f} deg",
        fontsize=8,
    )
    polar.grid(alpha=0.4)

    timeline = figure.add_subplot(1, 2, 2)
    relative_time_ms = (points_per_scan - 1 - np.arange(points_per_scan)) * timestamps_per_ray * 1000.0
    timeline.plot(relative_time_ms, angles, ".", markersize=2.0, color="#1f77b4")
    timeline.set_xlabel("time before scan end [ms]")
    timeline.set_ylabel("beam angle [rad]")
    timeline.set_title(
        f"Scan timing: one ray every {timestamps_per_ray * 1000.0:.3f} ms\n"
        f"(scan period = {1000.0 / LIDAR_SCAN_RATE:.1f} ms)",
        fontsize=8,
    )

    save(figure, "lidar_scan_pattern.png")


def plot_sensor_schedule():
    figure, axes = plt.subplots(figsize=(6.4, 2.6))
    duration_ms = 100.0
    physics_period_ms = 1000.0 / PHYSICS_RATE
    sensor_period_ms = 1000.0 / SENSOR_RATE

    physics_ticks = np.arange(0.0, duration_ms + 1e-9, physics_period_ms)
    sensor_ticks = np.arange(0.0, duration_ms + 1e-9, sensor_period_ms)

    axes.vlines(physics_ticks, 0.85, 1.0, color="#1f77b4", linewidth=0.6)
    axes.vlines(sensor_ticks, 0.55, 0.72, color="#d62728", linewidth=1.6)
    axes.text(duration_ms * 0.01, 0.92, f"physics steps, {PHYSICS_RATE:.0f} Hz", fontsize=8, color="#1f77b4")
    axes.text(
        duration_ms * 0.01,
        0.60,
        f"sensor and command systems, {SENSOR_RATE:.0f} Hz ({int(PHYSICS_RATE / SENSOR_RATE)} physics steps per period)",
        fontsize=8,
        color="#d62728",
    )
    axes.annotate(
        "",
        xy=(0.0, 0.35),
        xytext=(sensor_period_ms, 0.35),
        arrowprops={"arrowstyle": "<->", "linewidth": 1.0, "color": "black"},
    )
    axes.text(sensor_period_ms / 2.0, 0.38, f"{sensor_period_ms:.1f} ms", ha="center", fontsize=8)
    axes.set_xlim(0.0, duration_ms)
    axes.set_ylim(0.0, 1.1)
    axes.set_yticks([])
    axes.set_xlabel("simulation time [ms]")
    axes.set_title("System scheduling: sensor and command rates must be integer divisors of physics_rate")
    save(figure, "sensor_schedule.png")


def plot_suspension_inclination():
    def inclination(acceleration, is_longitudinal):
        lever_arms = np.array(
            [
                [WHEELBASE - CENTER_OF_MASS_X, FRONT_TRACK_WIDTH / 2.0],
                [WHEELBASE - CENTER_OF_MASS_X, -FRONT_TRACK_WIDTH / 2.0],
                [-CENTER_OF_MASS_X, REAR_TRACK_WIDTH / 2.0],
                [-CENTER_OF_MASS_X, -REAR_TRACK_WIDTH / 2.0],
            ]
        )
        direction = np.array([1.0, 0.0]) if is_longitudinal else np.array([0.0, 1.0])
        force = SPRUNG_MASS * acceleration
        height_offset = CENTER_OF_MASS_Z - ROLL_CENTER_HEIGHT
        sum_projection_squared = float(np.sum((lever_arms @ direction) ** 2))
        theta = -(height_offset * force) / (SPRING_STIFFNESS * sum_projection_squared)
        return np.clip(theta, -MAX_INCLINATION_ANGLE, MAX_INCLINATION_ANGLE)

    accelerations = np.linspace(-2.0 * 9.81, 6.0 * 9.81, 400)
    figure, axes = plt.subplots(figsize=(5.6, 3.2))
    axes.plot(accelerations / 9.81, np.rad2deg(inclination(accelerations, True)), label="Pitch (longitudinal)")
    axes.plot(accelerations / 9.81, np.rad2deg(inclination(accelerations, False)), label="Roll (lateral)")
    axes.axhline(np.rad2deg(MAX_INCLINATION_ANGLE), color="#7f7f7f", linestyle=":", linewidth=1.0)
    axes.axhline(-np.rad2deg(MAX_INCLINATION_ANGLE), color="#7f7f7f", linestyle=":", linewidth=1.0)
    axes.text(
        accelerations[0] / 9.81,
        np.rad2deg(MAX_INCLINATION_ANGLE) + 0.08,
        f"max_inclination_angle = {np.rad2deg(MAX_INCLINATION_ANGLE):.1f} deg",
        fontsize=7,
        color="#7f7f7f",
    )
    axes.axhline(0.0, color="black", linewidth=0.8)
    axes.set_xlabel("specific force [g]")
    axes.set_ylabel("body inclination [deg]")
    axes.set_title(
        "Quasi-static inclinations, $k_s$ = "
        f"{SPRING_STIFFNESS:.0f} N/m, $h_{{eff}}$ = {CENTER_OF_MASS_Z - ROLL_CENTER_HEIGHT:.4f} m"
    )
    axes.legend(loc="upper left")
    save(figure, "suspension_inclination.png")


def plot_pure_pursuit():
    lateral_offset = np.linspace(-0.8, 0.8, 401)
    distance = SPARRING_LOOKAHEAD_DISTANCE
    curvature = 2.0 * lateral_offset / distance**2
    steering = np.arctan(WHEELBASE * curvature)
    saturated = np.abs(steering) > SPARRING_MAX_STEERING

    figure, axes = plt.subplots(figsize=(5.4, 3.2))
    axes.plot(lateral_offset, np.rad2deg(steering), color="#1f77b4", label="Pure pursuit steering")
    axes.plot(
        lateral_offset[saturated],
        np.rad2deg(np.clip(steering[saturated], -SPARRING_MAX_STEERING, SPARRING_MAX_STEERING)),
        color="#d62728",
        linewidth=2.4,
        alpha=0.5,
        label="Saturated at $\\delta_{max}$",
    )
    axes.axhline(np.rad2deg(SPARRING_MAX_STEERING), color="#7f7f7f", linestyle=":", linewidth=1.0)
    axes.axhline(-np.rad2deg(SPARRING_MAX_STEERING), color="#7f7f7f", linestyle=":", linewidth=1.0)
    axes.set_xlabel("target lateral offset in vehicle frame [m]")
    axes.set_ylabel("steering command [deg]")
    axes.set_title(
        f"Pure pursuit mapping, lookahead = {distance:.2f} m, wheelbase = {WHEELBASE:.2f} m"
    )
    axes.legend(loc="upper left")
    save(figure, "pure_pursuit.png")


def main():
    plot_pacejka_longitudinal()
    plot_pacejka_lateral()
    plot_mnc_combined_slip()
    plot_vehicle_geometry()
    plot_steering_hysteresis()
    plot_velocity_reference_filter()
    plot_imu_noise()
    plot_lidar_scan_pattern()
    plot_sensor_schedule()
    plot_suspension_inclination()
    plot_pure_pursuit()


if __name__ == "__main__":
    main()
