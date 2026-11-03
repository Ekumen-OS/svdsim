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

// Non-ROS dependencies
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// Project headers
#include "svdsim_server/utils/pid_controller.hpp"

namespace svdsim_server {

class PIDControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dt_ = 0.01;  // 10 ms sampling time
  }

  double dt_;
};

TEST_F(PIDControllerTest, ProportionalOnlyControl) {
  // Configure a P-only controller
  PIDControllerParams params;
  params.kp = 2.0;
  params.ki = 0.0;
  params.kd = 0.0;

  PIDController uut(params, dt_);

  // Test cases: positive error, negative error, zero error
  const std::vector<double> references = {10.0, 3.0, 5.0};
  const std::vector<double> feedbacks = {5.0, 8.0, 5.0};
  const std::vector<double> expected_outputs = {10.0, -10.0, 0.0};

  for (size_t i = 0; i < references.size(); ++i) {
    const double output = uut.calculate(references[i], feedbacks[i]);
    EXPECT_DOUBLE_EQ(output, expected_outputs[i]);
  }
}

TEST_F(PIDControllerTest, ProportionalOnlyWithOutputLimits) {
  // Configure a P-only controller with output limits
  PIDControllerParams params;
  params.kp = 2.0;
  params.ki = 0.0;
  params.kd = 0.0;
  params.output_min = -5.0;
  params.output_max = 5.0;

  PIDController uut(params, dt_);

  // Test cases: saturate at max, saturate at min, within limits
  const std::vector<double> references = {10.0, 0.0, 2.0};
  const std::vector<double> feedbacks = {0.0, 10.0, 0.0};
  const std::vector<double> expected_outputs = {5.0, -5.0, 4.0};  // Clamped or within limits

  for (size_t i = 0; i < references.size(); ++i) {
    const double output = uut.calculate(references[i], feedbacks[i]);
    EXPECT_DOUBLE_EQ(output, expected_outputs[i]);
  }
}

TEST_F(PIDControllerTest, ResetClearsState) {
  // Configure a P-only controller
  PIDControllerParams params;
  params.kp = 1.0;
  params.ki = 0.0;
  params.kd = 0.0;

  PIDController uut(params, dt_);

  // Run a calculation to set internal state
  uut.calculate(10.0, 5.0);

  // Reset the controller
  uut.reset();

  // After reset, behavior should be the same as a fresh controller
  const double reference = 10.0;
  const double feedback = 5.0;
  const double expected_output = params.kp * (reference - feedback);

  const double output = uut.calculate(reference, feedback);
  EXPECT_DOUBLE_EQ(output, expected_output);
}

TEST_F(PIDControllerTest, IntegralOnlyControlWithoutAntiWindup) {
  // Configure an I-only controller without anti-windup
  PIDControllerParams params;
  params.kp = 0.0;
  params.ki = 0.5;
  params.kd = 0.0;
  params.enable_anti_windup = false;

  PIDController uut(params, dt_);

  // Apply constant error and verify integral accumulation
  const double reference = 10.0;
  const double feedback = 8.0;
  const double error = reference - feedback;  // 2.0

  double expected_integral = 0.0;

  // Run multiple iterations
  for (int i = 0; i < 5; ++i) {
    expected_integral += error * dt_;
    const double expected_output = params.ki * expected_integral;

    const double output = uut.calculate(reference, feedback);
    EXPECT_DOUBLE_EQ(output, expected_output);
  }
}

TEST_F(PIDControllerTest, IntegralOnlyControlVaryingError) {
  // Configure an I-only controller
  PIDControllerParams params;
  params.kp = 0.0;
  params.ki = 1.0;
  params.kd = 0.0;

  PIDController uut(params, dt_);

  // Test cases: varying errors that accumulate in the integral
  const std::vector<double> references = {5.0, 2.0};
  const std::vector<double> feedbacks = {3.0, 3.0};
  const std::vector<double> expected_outputs = {0.02, 0.01};  // Accumulated integral * ki

  for (size_t i = 0; i < references.size(); ++i) {
    const double output = uut.calculate(references[i], feedbacks[i]);
    EXPECT_DOUBLE_EQ(output, expected_outputs[i]);
  }
}

TEST_F(PIDControllerTest, IntegralWithAntiWindupEnabled) {
  // Configure an I-only controller with anti-windup and output limits
  PIDControllerParams params;
  params.kp = 0.0;
  params.ki = 1.0;
  params.kd = 0.0;
  params.output_min = -10.0;
  params.output_max = 10.0;
  params.enable_anti_windup = true;

  PIDController uut(params, dt_);

  // Apply large constant error that will saturate the output
  const double reference = 20.0;
  const double feedback = 0.0;

  // Run multiple iterations - output should saturate and integral should stop growing
  double last_output = 0.0;
  for (int i = 0; i < 100; ++i) {
    const double output = uut.calculate(reference, feedback);

    // Output should be clamped to max
    EXPECT_LE(output, params.output_max);
    EXPECT_GE(output, params.output_min);

    // Once saturated, output should stay at the limit
    if (output >= params.output_max) {
      EXPECT_DOUBLE_EQ(output, params.output_max);
    }
    last_output = output;
  }

  // Verify we reached saturation
  EXPECT_DOUBLE_EQ(last_output, params.output_max);

  // Now apply opposite error - with anti-windup, controller should respond quickly
  // because the integral hasn't wound up excessively
  const double new_reference = -20.0;
  const double output_after_reversal = uut.calculate(new_reference, feedback);

  // With anti-windup, the output should change significantly on first reversal
  // Without anti-windup, it would stay saturated for many iterations
  EXPECT_LT(output_after_reversal, params.output_max);
}

TEST_F(PIDControllerTest, IntegralWithAntiWindupDisabled) {
  // Configure an I-only controller without anti-windup and output limits
  PIDControllerParams params;
  params.kp = 0.0;
  params.ki = 1.0;
  params.kd = 0.0;
  params.output_min = -10.0;
  params.output_max = 10.0;
  params.enable_anti_windup = false;

  PIDController uut(params, dt_);

  // Apply large constant error that will saturate the output
  const double reference = 20.0;
  const double feedback = 0.0;

  // Run multiple iterations - integral will keep growing despite saturation
  for (int i = 0; i < 100; ++i) {
    const double output = uut.calculate(reference, feedback);

    // Output should be clamped to max
    EXPECT_LE(output, params.output_max);
    EXPECT_GE(output, params.output_min);
  }

  // Now apply opposite error - without anti-windup, integral has wound up
  // so it will take many iterations to unwind
  int iterations_to_unsaturate = 0;
  for (int i = 0; i < 200; ++i) {
    const double new_reference = -20.0;
    const double output = uut.calculate(new_reference, feedback);

    if (output < params.output_max - 0.01) {
      iterations_to_unsaturate = i + 1;
      break;
    }
  }

  // Without anti-windup, it should take many iterations to unsaturate
  EXPECT_GT(iterations_to_unsaturate, 10);
}

TEST_F(PIDControllerTest, DerivativeOnlyControlZeroTimeConstant) {
  // Configure a D-only controller with zero time constant (no filtering)
  PIDControllerParams params;
  params.kp = 0.0;
  params.ki = 0.0;
  params.kd = 2.0;
  params.derivative_time_constant = 0.0;  // No filtering

  PIDController uut(params, dt_);

  // Test cases: error changes from 0→5→5→2→-3
  const std::vector<double> references = {10.0, 10.0, 10.0, 5.0};
  const std::vector<double> feedbacks = {5.0, 5.0, 8.0, 8.0};
  const std::vector<double> expected_outputs = {1000.0, 0.0, -600.0, -1000.0};

  for (size_t i = 0; i < references.size(); ++i) {
    const double output = uut.calculate(references[i], feedbacks[i]);
    EXPECT_DOUBLE_EQ(output, expected_outputs[i]);
  }
}

TEST_F(PIDControllerTest, DerivativeOnlyControlWithFiltering) {
  // Configure a D-only controller with time constant of 0.5 seconds
  PIDControllerParams params;
  params.kp = 0.0;
  params.ki = 0.0;
  params.kd = 2.0;
  params.derivative_time_constant = 0.5;  // 0.5 second time constant

  PIDController uut(params, dt_);

  // Apply constant error derivative (error increases linearly)
  // Error will go: 0 → 0.02 → 0.04 → 0.06 → ... (increment of dt * 1 each step)
  const double error_derivative = 1.0;

  // Run for 0.5 seconds (50 iterations with dt = 0.01)
  const int iterations = static_cast<int>(params.derivative_time_constant / dt_);
  double output = 0.0;

  for (int i = 0; i <= iterations; ++i) {
    const double error = error_derivative * i * dt_;
    const double reference = error;  // Reference tracks to maintain constant derivative
    const double feedback = 0.0;
    output = uut.calculate(reference, feedback);
  }

  // After one time constant the expected output is (1 - exp(-1)) * 2
  const double expected_output = (1.0 - std::exp(-1.0)) * 2.0;

  EXPECT_NEAR(output, expected_output, 0.01);
}

TEST_F(PIDControllerTest, FullPIDControlAllTermsAddCorrectly) {
  // Configure a full PID controller with all gains non-zero
  PIDControllerParams params;
  params.kp = 1.0;
  params.ki = 2.0;
  params.kd = 3.0;
  params.derivative_time_constant = 0.0;  // No filtering for simplicity

  PIDController uut(params, dt_);

  // Test cases with varying reference values, constant feedback
  // Errors: 5.0 → 3.0 → 1.0 → -2.0 → 0.0
  const std::vector<double> references = {10.0, 8.0, 6.0, 3.0, 5.0};
  const std::vector<double> feedbacks = {5.0, 5.0, 5.0, 5.0, 5.0};
  // Expected outputs calculated as P + I + D for each iteration
  const std::vector<double> expected_outputs = {1505.1, -596.84, -598.82, -901.86, 600.14};

  for (size_t i = 0; i < references.size(); ++i) {
    const double output = uut.calculate(references[i], feedbacks[i]);
    EXPECT_DOUBLE_EQ(output, expected_outputs[i]);
  }
}

}  // namespace svdsim_server
