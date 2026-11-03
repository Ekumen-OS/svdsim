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

#ifndef MONACOF1_SIM_SERVER_EVENT_QUEUE_ACTIVATION_EVENT_HPP_
#define MONACOF1_SIM_SERVER_EVENT_QUEUE_ACTIVATION_EVENT_HPP_

// Standard library
#include <type_traits>
#include <utility>

// Project headers
#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {

/**
 * @brief Template base class for system activation events
 *
 * This template class defines the interface for activation events that are scheduled to execute at precise
 * simulation timestamps. Events carry a payload of type PayloadType which must be move constructible
 * and move assignable.
 *
 * @tparam PayloadType The type of payload carried by the event (must be move constructible and move assignable)
 */
template <typename PayloadType>
class ActivationEvent {
  static_assert(std::is_move_constructible_v<PayloadType>, "PayloadType must be move constructible");
  static_assert(std::is_move_assignable_v<PayloadType>, "PayloadType must be move assignable");

 public:
  /**
   * @brief Construct a new activation event with a payload
   *
   * @param timestamp The simulation timestamp when this event should execute
   * @param payload The event payload (moved into the event)
   */
  explicit ActivationEvent(DiscreteTime timestamp, PayloadType&& payload, std::size_t priority = 0)
      : timestamp_(timestamp), payload_(std::move(payload)), priority_(priority) {}

  /**
   * @brief Get the timestamp when this event should execute
   *
   * @return DiscreteTime The execution timestamp
   */
  [[nodiscard]] DiscreteTime timestamp() const { return timestamp_; }

  /**
   * @brief Get the priority of this event
   *
   * @return std::size_t The event priority (lower values indicate higher priority)
   */
  [[nodiscard]] std::size_t priority() const { return priority_; }

  /**
   * @brief Get a const reference to the event payload
   *
   * @return const PayloadType& The event payload
   */
  [[nodiscard]] const PayloadType payload() const { return payload_; }

  /**
   * @brief Get a mutable reference to the event payload
   *
   * @return PayloadType& The event payload
   */
  [[nodiscard]] PayloadType payload() { return payload_; }

 private:
  DiscreteTime timestamp_;
  PayloadType payload_;
  std::size_t priority_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_EVENT_QUEUE_ACTIVATION_EVENT_HPP_
