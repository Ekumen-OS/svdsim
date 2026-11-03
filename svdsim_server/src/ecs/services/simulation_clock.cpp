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

#include "svdsim_server/ecs/services/simulation_clock.hpp"

#include <stdexcept>

namespace svdsim_server {

SimulationClock::SimulationClock(RealType physics_rate)
    : physics_rate_(physics_rate), time_step_(1.0 / physics_rate), current_ticks_(0) {
  if (physics_rate <= 0.0) {
    throw std::invalid_argument("Physics rate must be positive");
  }
}

DiscreteTime SimulationClock::get_ticks() const {
  return current_ticks_;
}

WallTime SimulationClock::get_time() const {
  return ticks_to_real(current_ticks_);
}

WallTime SimulationClock::ticks_to_real(DiscreteTime ticks) const {
  RealType total_seconds = static_cast<RealType>(ticks.value()) * time_step_;
  return WallTime::from_seconds(total_seconds);
}

void SimulationClock::tick() {
  ++current_ticks_;
}

void SimulationClock::reset() {
  current_ticks_ = 0;
}

}  // namespace svdsim_server
