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

#include <gtest/gtest.h>

#include "svdsim_server/ecs/services/simulation_clock.hpp"

namespace svdsim_server {

class SimulationClockTest : public ::testing::Test {
 protected:
  void SetUp() override { tolerance_ = 1e-9; }

  double tolerance_;
};

TEST_F(SimulationClockTest, ConstructionWithValidRate) {
  SimulationClock clock(50.0);
  EXPECT_NEAR(clock.physics_rate(), 50.0, tolerance_);
  EXPECT_NEAR(clock.time_step(), 0.02, tolerance_);
  EXPECT_EQ(clock.get_ticks(), 0);
}

TEST_F(SimulationClockTest, ConstructionWithInvalidRate) {
  EXPECT_THROW(SimulationClock(0.0), std::invalid_argument);
  EXPECT_THROW(SimulationClock(-10.0), std::invalid_argument);
}

TEST_F(SimulationClockTest, GetTicksInitialState) {
  SimulationClock clock(100.0);
  EXPECT_EQ(clock.get_ticks(), 0);
}

TEST_F(SimulationClockTest, GetTimeInitialState) {
  SimulationClock clock(50.0);
  auto time = clock.get_time();
  EXPECT_EQ(time.seconds, 0);
  EXPECT_NEAR(time.fractional, 0.0, tolerance_);
}

TEST_F(SimulationClockTest, TickIncrementsCounter) {
  SimulationClock clock(100.0);

  clock.tick();
  EXPECT_EQ(clock.get_ticks(), 1);

  clock.tick();
  EXPECT_EQ(clock.get_ticks(), 2);

  clock.tick();
  EXPECT_EQ(clock.get_ticks(), 3);
}

TEST_F(SimulationClockTest, GetTimeAfterTicks) {
  SimulationClock clock(50.0);  // 0.02 seconds per tick

  clock.tick();
  auto time1 = clock.get_time();
  EXPECT_EQ(time1.seconds, 0);
  EXPECT_NEAR(time1.fractional, 0.02, tolerance_);

  // Tick 49 more times to reach 1.0 seconds
  for (int i = 0; i < 49; ++i) {
    clock.tick();
  }
  auto time50 = clock.get_time();
  EXPECT_EQ(time50.seconds, 1);
  EXPECT_NEAR(time50.fractional, 0.0, tolerance_);

  // One more tick
  clock.tick();
  auto time51 = clock.get_time();
  EXPECT_EQ(time51.seconds, 1);
  EXPECT_NEAR(time51.fractional, 0.02, tolerance_);
}

TEST_F(SimulationClockTest, TicksToRealConversion) {
  SimulationClock clock(100.0);  // 0.01 seconds per tick

  auto time0 = clock.ticks_to_real(0);
  EXPECT_EQ(time0.seconds, 0);
  EXPECT_NEAR(time0.fractional, 0.0, tolerance_);

  auto time1 = clock.ticks_to_real(1);
  EXPECT_EQ(time1.seconds, 0);
  EXPECT_NEAR(time1.fractional, 0.01, tolerance_);

  auto time50 = clock.ticks_to_real(50);
  EXPECT_EQ(time50.seconds, 0);
  EXPECT_NEAR(time50.fractional, 0.5, tolerance_);

  auto time100 = clock.ticks_to_real(100);
  EXPECT_EQ(time100.seconds, 1);
  EXPECT_NEAR(time100.fractional, 0.0, tolerance_);

  auto time250 = clock.ticks_to_real(250);
  EXPECT_EQ(time250.seconds, 2);
  EXPECT_NEAR(time250.fractional, 0.5, tolerance_);
}

TEST_F(SimulationClockTest, ResetClock) {
  SimulationClock clock(50.0);

  // Advance the clock
  for (int i = 0; i < 100; ++i) {
    clock.tick();
  }
  EXPECT_EQ(clock.get_ticks(), 100);

  // Reset
  clock.reset();
  EXPECT_EQ(clock.get_ticks(), 0);

  auto time = clock.get_time();
  EXPECT_EQ(time.seconds, 0);
  EXPECT_NEAR(time.fractional, 0.0, tolerance_);
}

TEST_F(SimulationClockTest, GetTimeMatchesTicksToReal) {
  SimulationClock clock(100.0);

  for (int i = 0; i < 200; ++i) {
    auto time_from_get = clock.get_time();
    auto time_from_convert = clock.ticks_to_real(clock.get_ticks());

    EXPECT_EQ(time_from_get.seconds, time_from_convert.seconds);
    EXPECT_NEAR(time_from_get.fractional, time_from_convert.fractional, tolerance_);

    clock.tick();
  }
}

TEST_F(SimulationClockTest, PhysicsRateAccuracy) {
  // Test that time calculations are accurate for various physics rates
  std::vector<double> rates = {10.0, 50.0, 100.0, 200.0, 1000.0};

  for (double rate : rates) {
    SimulationClock clock(rate);
    DiscreteTime ticks = static_cast<DiscreteTime::ValueType>(rate * 10);  // 10 seconds worth of ticks

    auto time = clock.ticks_to_real(ticks);
    double total_seconds = time.to_seconds();
    EXPECT_NEAR(total_seconds, 10.0, tolerance_);
  }
}

}  // namespace svdsim_server
