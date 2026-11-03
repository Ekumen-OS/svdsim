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

#ifndef MONACOF1_SIM_SERVER_MODELS_SPRUNG_MASS_INCLINATION_MODEL_HPP_
#define MONACOF1_SIM_SERVER_MODELS_SPRUNG_MASS_INCLINATION_MODEL_HPP_

#include <array>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "svdsim_server/models/imu_sensor_model.hpp"

namespace svdsim_server {

/**
 * @brief Parameters for the sprung mass inclination model
 *
 * Contains the physical parameters needed to compute vehicle body orientation
 * variations due to acceleration-induced load transfer.
 */
struct SprungMassInclinationParameters {
  /// Height of the center of mass above the ground plane [m]
  double center_of_mass_height = 0.0;

  /// Forward distance of the center of mass from the center of the rear axle [m]
  double center_of_mass_forward_distance = 0.0;

  /// Wheelbase (distance between front and rear axles) [m]
  double wheelbase = 0.0;

  /// Front track width (distance between left and right front wheels) [m]
  double front_track_width = 0.0;

  /// Rear track width (distance between left and right rear wheels) [m]
  double rear_track_width = 0.0;

  /// Sprung mass (mass supported by the suspension) [kg]
  double sprung_mass = 0.0;

  /// Spring stiffness at each corner [N/m]
  double spring_stiffness = 0.0;

  /// Roll center height above ground [m] (default: effective wheel radius)
  double roll_center_height = 0.0325;

  /// Maximum inclination angle [rad] (default: 10 degrees = 0.174533 rad)
  double max_inclination_angle = 0.174533;
};

/**
 * @brief Model for computing vehicle body inclination due to acceleration
 *
 * This model computes the orientation (SO3) of the vehicle body frame relative
 * to a nominal orientation, accounting for pitch and roll induced by longitudinal
 * and lateral accelerations acting on the center of mass.
 *
 * The model assumes a quasi-static suspension response where the body orientation
 * adjusts instantaneously to balance moments created by inertial forces.
 */
class SprungMassInclinationModel {
 public:
  /**
   * @brief Construct a new Sprung Mass Inclination Model
   *
   * @param params Physical parameters of the sprung mass system
   */
  explicit SprungMassInclinationModel(const SprungMassInclinationParameters& params);

  /**
   * @brief Compute vehicle body orientation from vehicle dynamics
   *
   * Calculates the SO3 orientation of the vehicle body frame accounting for
   * pitch and roll induced by acceleration and angular motion.
   *
   * @param T_iv Vehicle pose in inertial frame (transforms from vehicle to inertial)
   * @param r_ddot_vi_i Linear acceleration of vehicle origin in inertial frame [m/s^2]
   * @param omega_vi_v Angular velocity of vehicle relative to inertial, expressed in vehicle frame [rad/s]
   * @param omega_dot_vi_v Angular acceleration of vehicle relative to inertial, expressed in vehicle frame [rad/s^2]
   * @return SO3 orientation of the vehicle body frame
   *
   * @note This function is currently incomplete and will be implemented later
   */
  [[nodiscard]] Sophus::SO3d compute_body_orientation(
      const Sophus::SE3d& T_iv,
      const Eigen::Vector3d& r_ddot_vi_i,
      const Eigen::Vector3d& omega_vi_v,
      const Eigen::Vector3d& omega_dot_vi_v);

 private:
  /// Model parameters
  SprungMassInclinationParameters params_;

  /// IMU sensor model for computing acceleration at center of mass
  IMUSensorModel imu_sensor_;

  /// Lever arm vectors for the four corners relative to center of mass [m]
  /// Order: [front-left, front-right, rear-left, rear-right]
  /// 2D vectors in the horizontal plane (XY)
  std::array<Eigen::Vector2d, 4> lever_arms_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_MODELS_SPRUNG_MASS_INCLINATION_MODEL_HPP_
