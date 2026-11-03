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

#ifndef MONACOF1_SIM_SERVER_MODELS_RUNGE_KUTTA_HPP_
#define MONACOF1_SIM_SERVER_MODELS_RUNGE_KUTTA_HPP_

#include <functional>

namespace svdsim_server {

/**
 * @brief Type alias for dynamics function signature
 *
 * The dynamics function computes the state derivative given current state and input.
 *
 * @tparam State State vector type
 * @tparam Input Control input vector type
 */
template <typename State, typename Input>
using DynamicsFunction = std::function<State(const State&, const Input&)>;

/**
 * @brief Fourth-order Runge-Kutta (RK4) numerical integration
 *
 * Performs one integration step using the classic RK4 method for autonomous systems.
 * The method provides fourth-order accuracy by computing four intermediate slopes
 * and combining them with appropriate weights.
 *
 * Integration formula:
 * - k1 = f(x, u)
 * - k2 = f(x + dt/2 * k1, u)
 * - k3 = f(x + dt/2 * k2, u)
 * - k4 = f(x + dt * k3, u)
 * - x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
 *
 * @tparam State State vector type (must support scalar multiplication and addition)
 * @tparam Input Control input vector type
 *
 * @param dynamics_function Function computing state derivative: dx/dt = f(x, u)
 * @param state Current state vector
 * @param input Control input vector (held constant during integration step)
 * @param dt Time step for integration [s]
 * @return State Updated state vector after one integration step
 */
template <typename State, typename Input>
State rk4_step(
    const DynamicsFunction<State, Input>& dynamics_function,
    const State& state,
    const Input& input,
    double dt) {
  // Compute k1: slope at the beginning of interval
  const State k1 = dynamics_function(state, input);

  // Compute k2: slope at midpoint using k1
  const State k2 = dynamics_function(state + (dt / 2.0) * k1, input);

  // Compute k3: slope at midpoint using k2
  const State k3 = dynamics_function(state + (dt / 2.0) * k2, input);

  // Compute k4: slope at the end of interval using k3
  const State k4 = dynamics_function(state + dt * k3, input);

  // Combine slopes with RK4 weights
  return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * @brief Fourth-order Runge-Kutta (RK4) integration with multiple steps
 *
 * Performs multiple RK4 integration steps to advance the state over a longer time horizon.
 * This is useful when you want to integrate over a total time using smaller sub-steps
 * for improved accuracy.
 *
 * @tparam State State vector type (must support scalar multiplication and addition)
 * @tparam Input Control input vector type
 *
 * @param dynamics_function Function computing state derivative: dx/dt = f(x, u)
 * @param state Initial state vector
 * @param input Control input vector (held constant over all steps)
 * @param dt Time step for each integration sub-step [s]
 * @param num_steps Number of integration steps to perform
 * @return State Final state vector after all integration steps
 */
template <typename State, typename Input>
State rk4_integrate(
    const DynamicsFunction<State, Input>& dynamics_function,
    const State& state,
    const Input& input,
    double dt,
    int num_steps) {
  State current_state = state;
  for (int i = 0; i < num_steps; ++i) {
    current_state = rk4_step(dynamics_function, current_state, input, dt);
  }
  return current_state;
}

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_MODELS_RUNGE_KUTTA_HPP_
