// Copyright 2025 Ekumen
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

#ifndef SVDSIM_SERVER_MODELS_SIMPLE_DRIVE_MODEL_HPP_
#define SVDSIM_SERVER_MODELS_SIMPLE_DRIVE_MODEL_HPP_

#include "svdsim_server/utils/second_order_smoothing_filter.hpp"

namespace svdsim_server {

/**
 * @brief Parameters for the SimpleDriveModel
 */
struct SimpleDriveModelParameters {
  double steering_velocity_max;     ///< Maximum steering velocity [rad/s]
  double steering_angle_max;        ///< Maximum steering angle [rad] (min is the negative of this)
  double steering_hysteresis_gap;   ///< Steering hysteresis loop width [rad]. The effective steering
                                    ///< reference lags the commanded reference by half this value, and
                                    ///< commands within half the gap of the current angle are ignored.
  double velocity_max;              ///< Maximum vehicle velocity [m/s]
  double velocity_min;              ///< Minimum vehicle velocity [m/s]
  double velocity_deadband;         ///< Velocity dead band magnitude [m/s] (references below this are treated as zero)
  double drive_motor_acceleration;  ///< Maximum vehicle acceleration [m/s²]
  double drive_braking_deceleration;  ///< Braking deceleration magnitude [m/s²] (positive value)
  double motor_drive_rise_time;       ///< Rise time for velocity reference smoothing filter [s]
  double dt;                          ///< Time step duration [s]
};

/**
 * @brief Simple drive model for vehicle control
 *
 * This class implements a bang-bang drive model that converts steering angle and velocity
 * references into steering velocity commands and acceleration commands. The model uses:
 * - Bang-bang steering control: applies maximum steering velocity toward the reference
 * - Bang-bang throttle control: applies maximum acceleration or braking based on velocity error
 * - Second-order smoothing filter on velocity reference to reduce input jitter
 *
 * The model enforces physical limits on steering angles and vehicle velocity while respecting
 * rate constraints to prevent violation of these limits within a single time step.
 */
class SimpleDriveModel {
 public:
  /**
   * @brief Construct a simple drive model with specified parameters
   *
   * @param params Model parameters
   */
  explicit SimpleDriveModel(const SimpleDriveModelParameters& params);

  /**
   * @brief Reset the drive model state
   *
   * Resets the internal smoothing filter state.
   */
  void reset();

  /**
   * @brief Compute steering velocity command using bang-bang control
   *
   * @param steering_ref Desired steering angle [rad]
   * @param steering_feedback Current steering angle [rad]
   * @return Steering velocity command [rad/s]
   */
  [[nodiscard]] double update_steering_velocity(double steering_ref, double steering_feedback) const;

  /**
   * @brief Compute acceleration command using bang-bang control
   *
   * @param velocity_ref Desired velocity [m/s]
   * @param velocity_feedback Current velocity [m/s]
   * @return Acceleration command [m/s²]
   */
  [[nodiscard]] double update_acceleration(double velocity_ref, double velocity_feedback);

 private:
  SimpleDriveModelParameters params_;
  SecondOrderSmoothingFilter velocity_ref_filter_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_MODELS_SIMPLE_DRIVE_MODEL_HPP_
