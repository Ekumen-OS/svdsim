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
#include <numbers>

// Project headers
#include "svdsim_server/models/pulse_encoder_model.hpp"

namespace svdsim_server {

class PulseEncoderModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Typical encoder parameters
    pulses_per_revolution_ = 1000.0;  // 1000 pulses per wheel revolution
    conversion_ratio_ = 2.0;          // 2:1 gear ratio (motor to wheel)

    encoder_ = std::make_unique<PulseEncoderModel>(conversion_ratio_, pulses_per_revolution_);
  }

  double pulses_per_revolution_;
  double conversion_ratio_;
  std::unique_ptr<PulseEncoderModel> encoder_;
};

TEST_F(PulseEncoderModelTest, ZeroPosition) {
  const auto result = encoder_->calculate_encoder_output(0, 0.0);
  EXPECT_EQ(result, 0);
}

TEST_F(PulseEncoderModelTest, OneCompleteRotation) {
  {
    const auto result = encoder_->calculate_encoder_output(1, 0.0);
    const auto expected = static_cast<int32_t>(1.0 * conversion_ratio_ * pulses_per_revolution_);
    EXPECT_EQ(result, expected);
  }
  {
    const auto result = encoder_->calculate_encoder_output(-1, 0.0);
    const auto expected = static_cast<int32_t>(-1.0 * conversion_ratio_ * pulses_per_revolution_);
    EXPECT_EQ(result, expected);
  }
}

TEST_F(PulseEncoderModelTest, HalfRotationAngle) {
  {
    const auto result = encoder_->calculate_encoder_output(0, M_PI);
    const auto expected = static_cast<int32_t>(std::round(0.5 * conversion_ratio_ * pulses_per_revolution_));
    EXPECT_EQ(result, expected);
  }
  {
    const auto result = encoder_->calculate_encoder_output(0, -M_PI);
    const auto expected = static_cast<int32_t>(std::round(-0.5 * conversion_ratio_ * pulses_per_revolution_));
    EXPECT_EQ(result, expected);
  }
}

TEST_F(PulseEncoderModelTest, CombinedCyclesAndAngle) {
  const auto result = encoder_->calculate_encoder_output(-2, M_PI / 2.0);
  const auto expected = static_cast<int32_t>(std::round((-2.0 + 0.25) * conversion_ratio_ * pulses_per_revolution_));
  EXPECT_EQ(result, expected);
}

// Test getters
TEST_F(PulseEncoderModelTest, GetConversionRatio) {
  EXPECT_DOUBLE_EQ(encoder_->conversion_ratio(), conversion_ratio_);
}

TEST_F(PulseEncoderModelTest, GetPulsesPerRevolution) {
  EXPECT_DOUBLE_EQ(encoder_->pulses_per_revolution(), pulses_per_revolution_);
}

}  // namespace svdsim_server
