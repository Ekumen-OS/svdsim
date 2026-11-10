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

#ifndef SVDSIM_SERVER_ECS_SERVICES_SIMULATION_CLOCK_HPP_
#define SVDSIM_SERVER_ECS_SERVICES_SIMULATION_CLOCK_HPP_

#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/wall_time.hpp"

namespace svdsim_server {

/**
 * @brief Clock service for managing simulation time
 *
 * Provides conversion between discrete simulation ticks and continuous
 * simulated time based on a fixed physics rate. The clock maintains
 * an internal tick counter and converts between tick-based timestamps
 * and real simulated time.
 */
class SimulationClock {
 public:
  /// @brief Type used for real-valued time representation
  using RealType = WallTime::RealType;

  /**
   * @brief Construct a new Simulation Clock
   *
   * @param physics_rate Physics simulation update rate [Hz]
   * @throws std::invalid_argument if physics_rate <= 0
   */
  explicit SimulationClock(RealType physics_rate);

  /**
   * @brief Get the current discrete tick count
   *
   * @return Current simulation tick (DiscreteTime)
   */
  [[nodiscard]] DiscreteTime get_ticks() const;

  /**
   * @brief Get the current simulated time
   *
   * @return Current simulated time as WallTime
   */
  [[nodiscard]] WallTime get_time() const;

  /**
   * @brief Convert a discrete tick count to simulated wall time
   *
   * @param ticks Discrete tick count
   * @return Simulated wall time corresponding to the tick count
   */
  [[nodiscard]] WallTime ticks_to_real(DiscreteTime ticks) const;

  /**
   * @brief Advance the simulation clock by one tick
   */
  void tick();

  /**
   * @brief Reset the clock to zero
   */
  void reset();

  /**
   * @brief Get the physics rate
   * @return Physics update rate [Hz]
   */
  [[nodiscard]] RealType physics_rate() const { return physics_rate_; }

  /**
   * @brief Get the time step per tick
   * @return Time step [s]
   */
  [[nodiscard]] RealType time_step() const { return time_step_; }

 private:
  /// @brief Physics simulation update rate [Hz]
  RealType physics_rate_;

  /// @brief Time step per simulation tick [s]
  RealType time_step_;

  /// @brief Current simulation tick count
  DiscreteTime current_ticks_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_SERVICES_SIMULATION_CLOCK_HPP_
