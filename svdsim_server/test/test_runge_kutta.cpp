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

// Non-ROS dependencies
#include <gtest/gtest.h>
#include <cmath>
#include <eigen3/Eigen/Core>

// Project headers
#include "svdsim_server/models/runge_kutta.hpp"

namespace svdsim_server {

class RungeKuttaTest : public ::testing::Test {
 protected:
  // Test tolerance for numerical integration
  static constexpr double kTolerance = 1e-6;
  static constexpr double kLargeTolerance = 1e-3;
};

// Test simple exponential decay: dx/dt = -k*x
// Analytical solution: x(t) = x0 * exp(-k*t)
TEST_F(RungeKuttaTest, ExponentialDecay) {
  using State = Eigen::Matrix<double, 1, 1>;
  using Input = Eigen::Matrix<double, 1, 1>;

  const double k = 0.5;    // Decay constant
  const double x0 = 10.0;  // Initial state
  const double dt = 0.01;  // Time step
  const double total_time = 1.0;
  const int num_steps = static_cast<int>(total_time / dt);

  // Dynamics: dx/dt = -k*x (input is ignored for this test)
  DynamicsFunction<State, Input> dynamics = [k](const State& x, const Input&) -> State { return -k * x; };

  State initial_state;
  initial_state(0) = x0;

  Input dummy_input;
  dummy_input(0) = 0.0;

  // Integrate using RK4
  State final_state = rk4_integrate(dynamics, initial_state, dummy_input, dt, num_steps);

  // Analytical solution
  const double expected = x0 * std::exp(-k * total_time);

  EXPECT_NEAR(final_state(0), expected, kLargeTolerance);
}

// Test simple harmonic oscillator: d²x/dt² = -ω²*x
// State = [position, velocity]
// dx/dt = v, dv/dt = -ω²*x
// Analytical solution: x(t) = A*cos(ωt) + B*sin(ωt)
// With x0=1, v0=0: x(t) = cos(ωt), v(t) = -ω*sin(ωt)
TEST_F(RungeKuttaTest, SimpleHarmonicOscillator) {
  using State = Eigen::Matrix<double, 2, 1>;
  using Input = Eigen::Matrix<double, 1, 1>;

  const double omega = 2.0;                  // Angular frequency
  const double x0 = 1.0;                     // Initial position
  const double v0 = 0.0;                     // Initial velocity
  const double dt = 0.0002;                  // Small time step for accuracy
  const double period = 2.0 * M_PI / omega;  // One full period

  // Dynamics: [v, -ω²*x]
  DynamicsFunction<State, Input> dynamics = [omega](const State& state, const Input&) -> State {
    State derivative;
    derivative(0) = state(1);                   // dx/dt = v
    derivative(1) = -omega * omega * state(0);  // dv/dt = -ω²*x
    return derivative;
  };

  State initial_state;
  initial_state(0) = x0;  // position
  initial_state(1) = v0;  // velocity

  Input dummy_input;
  dummy_input(0) = 0.0;

  // Test at t = 0 (initial state - maximum position)
  State state_t0 = initial_state;
  EXPECT_NEAR(state_t0(0), 1.0, kTolerance);  // x = cos(0) = 1
  EXPECT_NEAR(state_t0(1), 0.0, kTolerance);  // v = -ω*sin(0) = 0

  // Test at t = T/4 (quarter period - maximum negative velocity)
  const double t_quarter = period / 4.0;
  const int steps_quarter = static_cast<int>(t_quarter / dt);
  State state_t_quarter = rk4_integrate(dynamics, initial_state, dummy_input, dt, steps_quarter);
  EXPECT_NEAR(state_t_quarter(0), 0.0, kLargeTolerance);     // x = cos(π/2) = 0
  EXPECT_NEAR(state_t_quarter(1), -omega, kLargeTolerance);  // v = -ω*sin(π/2) = -ω

  // Test at t = T/2 (half period - minimum position)
  const double t_half = period / 2.0;
  const int steps_half = static_cast<int>(t_half / dt);
  State state_t_half = rk4_integrate(dynamics, initial_state, dummy_input, dt, steps_half);
  EXPECT_NEAR(state_t_half(0), -1.0, kLargeTolerance);  // x = cos(π) = -1
  EXPECT_NEAR(state_t_half(1), 0.0, kLargeTolerance);   // v = -ω*sin(π) = 0

  // Test at t = 3T/4 (three-quarter period - maximum positive velocity)
  const double t_three_quarter = 3.0 * period / 4.0;
  const int steps_three_quarter = static_cast<int>(t_three_quarter / dt);
  State state_t_three_quarter = rk4_integrate(dynamics, initial_state, dummy_input, dt, steps_three_quarter);
  EXPECT_NEAR(state_t_three_quarter(0), 0.0, kLargeTolerance);    // x = cos(3π/2) = 0
  EXPECT_NEAR(state_t_three_quarter(1), omega, kLargeTolerance);  // v = -ω*sin(3π/2) = ω

  // Test at t = T (full period - return to initial state)
  const int steps_full = static_cast<int>(period / dt);
  State state_t_full = rk4_integrate(dynamics, initial_state, dummy_input, dt, steps_full);
  EXPECT_NEAR(state_t_full(0), x0, kLargeTolerance);  // position
  EXPECT_NEAR(state_t_full(1), v0, kLargeTolerance);  // velocity
}

// Test constant acceleration: dv/dt = a, dx/dt = v
// Analytical solution: x(t) = x0 + v0*t + 0.5*a*t²
TEST_F(RungeKuttaTest, ConstantAcceleration) {
  using State = Eigen::Matrix<double, 2, 1>;
  using Input = Eigen::Matrix<double, 1, 1>;

  const double a = 2.0;   // Acceleration
  const double x0 = 0.0;  // Initial position
  const double v0 = 1.0;  // Initial velocity
  const double dt = 0.01;

  // Dynamics with acceleration from input
  DynamicsFunction<State, Input> dynamics = [](const State& state, const Input& input) -> State {
    State derivative;
    derivative(0) = state(1);  // dx/dt = v
    derivative(1) = input(0);  // dv/dt = a
    return derivative;
  };

  State initial_state;
  initial_state(0) = x0;  // position
  initial_state(1) = v0;  // velocity

  Input acceleration_input;
  acceleration_input(0) = a;

  // Single step test
  State next_state = rk4_step(dynamics, initial_state, acceleration_input, dt);

  // Analytical solution after one time step
  const double expected_pos = x0 + v0 * dt + 0.5 * a * dt * dt;
  const double expected_vel = v0 + a * dt;

  EXPECT_NEAR(next_state(0), expected_pos, kTolerance);
  EXPECT_NEAR(next_state(1), expected_vel, kTolerance);
}

// Test multi-step vs single-step integration
TEST_F(RungeKuttaTest, MultiStepVsSingleStep) {
  using State = Eigen::Matrix<double, 1, 1>;
  using Input = Eigen::Matrix<double, 1, 1>;

  const double k = 0.3;
  const double x0 = 5.0;
  const double dt = 0.1;
  const int num_steps = 5;

  DynamicsFunction<State, Input> dynamics = [k](const State& x, const Input&) -> State { return -k * x; };

  State initial_state;
  initial_state(0) = x0;

  Input dummy_input;
  dummy_input(0) = 0.0;

  // Multi-step integration
  State result_multi = rk4_integrate(dynamics, initial_state, dummy_input, dt, num_steps);

  // Sequential single-step integration
  State result_single = initial_state;
  for (int i = 0; i < num_steps; ++i) {
    result_single = rk4_step(dynamics, result_single, dummy_input, dt);
  }

  // Should produce identical results
  EXPECT_DOUBLE_EQ(result_multi(0), result_single(0));
}

// Test with higher-dimensional state (5D like SingleTrackKinematicModel)
TEST_F(RungeKuttaTest, HighDimensionalState) {
  using State = Eigen::Matrix<double, 5, 1>;
  using Input = Eigen::Matrix<double, 2, 1>;

  // Simple decoupled dynamics: each state component decays independently
  const double k = 0.2;
  DynamicsFunction<State, Input> dynamics = [k](const State& state, const Input&) -> State {
    // Use Eigen's element-wise operations (broadcasting)
    return -k * state;
  };

  State initial_state;
  initial_state << 1.0, 2.0, 3.0, 4.0, 5.0;

  Input dummy_input;
  dummy_input << 0.0, 0.0;

  const double dt = 0.01;
  const double total_time = 0.5;
  const int num_steps = static_cast<int>(total_time / dt);

  State final_state = rk4_integrate(dynamics, initial_state, dummy_input, dt, num_steps);

  // Each component should decay exponentially
  for (int i = 0; i < 5; ++i) {
    const double expected = initial_state(i) * std::exp(-k * total_time);
    EXPECT_NEAR(final_state(i), expected, kLargeTolerance);
  }
}

// Test zero dynamics (state should remain constant)
TEST_F(RungeKuttaTest, ZeroDynamics) {
  using State = Eigen::Matrix<double, 3, 1>;
  using Input = Eigen::Matrix<double, 1, 1>;

  // Zero dynamics: dx/dt = 0
  DynamicsFunction<State, Input> dynamics = [](const State&, const Input&) -> State {
    State derivative;
    derivative.setZero();
    return derivative;
  };

  State initial_state;
  initial_state << 1.0, 2.0, 3.0;

  Input dummy_input;
  dummy_input(0) = 0.0;

  const double dt = 0.1;
  const int num_steps = 10;

  State final_state = rk4_integrate(dynamics, initial_state, dummy_input, dt, num_steps);

  // State should remain unchanged
  EXPECT_DOUBLE_EQ(final_state(0), initial_state(0));
  EXPECT_DOUBLE_EQ(final_state(1), initial_state(1));
  EXPECT_DOUBLE_EQ(final_state(2), initial_state(2));
}

// Test input-dependent dynamics
TEST_F(RungeKuttaTest, InputDependentDynamics) {
  using State = Eigen::Matrix<double, 1, 1>;
  using Input = Eigen::Matrix<double, 1, 1>;

  // Dynamics: dx/dt = input
  DynamicsFunction<State, Input> dynamics = [](const State&, const Input& input) -> State {
    State derivative;
    derivative(0) = input(0);
    return derivative;
  };

  State initial_state;
  initial_state(0) = 0.0;

  Input constant_input;
  constant_input(0) = 2.0;

  const double dt = 0.01;
  const int num_steps = 100;  // Total time = 1.0

  State final_state = rk4_integrate(dynamics, initial_state, constant_input, dt, num_steps);

  // With constant input of 2.0 for 1.0 seconds: x = x0 + u*t = 0 + 2*1 = 2
  EXPECT_NEAR(final_state(0), 2.0, kTolerance);
}

}  // namespace svdsim_server

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
