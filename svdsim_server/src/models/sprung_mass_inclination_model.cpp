// Copyright 2026 Ekumen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "svdsim_server/models/sprung_mass_inclination_model.hpp"

#include <algorithm>

namespace svdsim_server {

SprungMassInclinationModel::SprungMassInclinationModel(const SprungMassInclinationParameters& params)
    : params_(params),
      imu_sensor_(
          Sophus::SE3d(
              Sophus::SO3d(),  // No rotation
              Eigen::Vector3d(params.center_of_mass_forward_distance, 0.0, params.center_of_mass_height)),
          0.0,
          IMUNoiseParameters{}) {
  // Compute lever arm vectors for the four corners of the vehicle relative to the center of mass
  // Assuming the CoM pose translation is in the vehicle frame, and corners are defined in vehicle frame
  // Layout: Front is +X, Left is +Y in vehicle frame
  const double cg_x = params_.center_of_mass_forward_distance;
  const double cg_y = 0.0;  // Assumed on centerline

  // Lever arms in the horizontal plane (XY)
  lever_arms_[0] = Eigen::Vector2d(params_.wheelbase - cg_x, params_.front_track_width / 2.0 - cg_y);
  lever_arms_[1] = Eigen::Vector2d(params_.wheelbase - cg_x, -params_.front_track_width / 2.0 - cg_y);
  lever_arms_[2] = Eigen::Vector2d(-cg_x, params_.rear_track_width / 2.0 - cg_y);
  lever_arms_[3] = Eigen::Vector2d(-cg_x, -params_.rear_track_width / 2.0 - cg_y);
}

Sophus::SO3d SprungMassInclinationModel::compute_body_orientation(
    const Sophus::SE3d& T_iv,
    const Eigen::Vector3d& r_ddot_vi_i,
    const Eigen::Vector3d& omega_vi_v,
    const Eigen::Vector3d& omega_dot_vi_v) {
  // Sample the accelerometer at the center of mass with no body inclination (C_vb = identity)
  const Sophus::SO3d C_vb_identity = Sophus::SO3d();
  const Eigen::Vector3d acceleration_at_cg =
      imu_sensor_.sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb_identity);

  // Calculate the perturbation force acting on the sprung mass
  const Eigen::Vector3d perturbation_force = params_.sprung_mass * acceleration_at_cg;

  // The perturbation force F_a acts at the center of mass, creating a moment about the roll center
  // that causes the body to rotate about an axis perpendicular to the force direction.
  // The suspension springs resist by compressing/extending differentially.
  //
  // For quasi-static equilibrium with small angle approximation (sin(\theta) \approx \theta):
  //   - Let u_F be the unit vector in the direction of F_horizontal
  //   - Rotation axis is perpendicular to u_F in the horizontal plane
  //   - Effective lever arm from roll center to CoG: h_eff = h_cg - h_rc
  //   - Moment from force: M = h_eff * |F_horizontal|
  //   - Each spring at position r_i has effective lever arm: d_i = |r_i \cdot u_F|
  //   - Spring deflection: \delta_i \approx \theta * d_i
  //   - Restoring moment: M_restore = k_s * \theta * \sum(d_i^2) = k_s * \theta * \sum((r_i \cdot u_F)^2)
  //   - Equilibrium: h_eff * |F_horizontal| = k_s * \theta * \sum((r_i \cdot u_F)^2)
  //   - Solution: \theta = (h_eff * |F_horizontal|) / (k_s * \sum((r_i \cdot u_F)^2))
  // notice that the lever arm is squared because it affects both the deflection and
  // the moment arm of the spring force.

  // Extract horizontal force components (XY plane)
  const Eigen::Vector2d F_horizontal(perturbation_force.x(), perturbation_force.y());
  const double F_horizontal_magnitude = F_horizontal.norm();

  // Handle zero force case
  if (F_horizontal_magnitude < 1e-10) {
    return Sophus::SO3d();  // No inclination
  }

  // Compute u_F: unit vector in the direction of the force (2D in horizontal plane)
  const Eigen::Vector2d u_F = F_horizontal / F_horizontal_magnitude;

  // Effective lever arm: distance from roll center to center of mass
  const double h_eff = params_.center_of_mass_height - params_.roll_center_height;

  // Compute effective moment of inertia: \sum((r_i \cdot u_F)^2)
  double sum_projection_squared = 0.0;
  for (const auto& lever_arm : lever_arms_) {
    const double projection = lever_arm.dot(u_F);
    sum_projection_squared += projection * projection;
  }

  // Compute rotation angle from equilibrium equation
  // sign is because we tilt in the opposite direction of the force
  const double theta_unclamped =
      -(h_eff * F_horizontal_magnitude) / (params_.spring_stiffness * sum_projection_squared);

  // Clamp the angle to maximum inclination
  const double theta = std::clamp(theta_unclamped, -params_.max_inclination_angle, params_.max_inclination_angle);

  // Rotation axis: perpendicular to u_F (90 degrees counter-clockwise in horizontal plane)
  const Eigen::Vector3d rotation_axis(-u_F.y(), u_F.x(), 0.0);

  // Construct SO3 rotation: rotate by angle theta about rotation_axis
  const Sophus::SO3d body_orientation = Sophus::SO3d::exp(theta * rotation_axis);

  return body_orientation;
}

}  // namespace svdsim_server
