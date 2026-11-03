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

#ifndef MONACOF1_SIM_SERVER_EVENT_QUEUE_ACTIVATION_QUEUE_HPP_
#define MONACOF1_SIM_SERVER_EVENT_QUEUE_ACTIVATION_QUEUE_HPP_

// Standard library
#include <memory>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <vector>

// Project headers
#include "svdsim_server/event_queue/activation_event.hpp"

namespace svdsim_server {

// Type trait to detect if a type is a specialization of ActivationEvent
template <typename>
struct is_activation_event : std::false_type {};

template <typename PayloadType>
struct is_activation_event<ActivationEvent<PayloadType>> : std::true_type {};

template <typename T>
inline constexpr bool is_activation_event_v = is_activation_event<T>::value;

/**
 * @brief Priority queue for timestamped system activations
 *
 * ActivationQueue maintains a priority queue of timestamped invocations ordered first by their timestamps
 * (earlier timestamps have higher priority), and then by their priority values (lower priority values
 * execute first when timestamps are equal).
 *
 * @tparam EventType The type of invocation to store (must be a specialization of ActivationEvent)
 */
template <typename EventType>
class ActivationQueue {
  static_assert(is_activation_event_v<EventType>, "EventType must be a specialization of ActivationEvent");

 public:
  ActivationQueue() = default;
  ~ActivationQueue() = default;

  // Non-copyable, non-movable
  ActivationQueue(const ActivationQueue&) = delete;
  ActivationQueue& operator=(const ActivationQueue&) = delete;
  ActivationQueue(ActivationQueue&&) = delete;
  ActivationQueue& operator=(ActivationQueue&&) = delete;

  /**
   * @brief Insert a new invocation (moves the invocation into the queue)
   *
   * @param event Invocation to insert (will be moved)
   */
  void insert(EventType&& event) { queue_.push(std::move(event)); }

  /**
   * @brief Remove the invocation at the front of the queue
   *
   * @throws std::runtime_error if the queue is empty
   */
  void pop() {
    if (queue_.empty()) {
      throw std::runtime_error("ActivationQueue::pop() called on empty queue");
    }
    queue_.pop();
  }

  /**
   * @brief Access the invocation at the front (const reference)
   *
   * @return const EventType& Reference to the front invocation
   * @throws std::runtime_error if the queue is empty
   */
  [[nodiscard]] const EventType& front() const {
    if (queue_.empty()) {
      throw std::runtime_error("ActivationQueue::front() called on empty queue");
    }
    return queue_.top();
  }

  /**
   * @brief Check if the queue is empty
   *
   * @return bool True if the queue is empty, false otherwise
   */
  [[nodiscard]] bool empty() const { return queue_.empty(); }

  /**
   * @brief Get the number of invocations in the queue
   *
   * @return std::size_t The number of invocations
   */
  [[nodiscard]] std::size_t size() const { return queue_.size(); }

 private:
  struct EventCompare {
    bool operator()(const EventType& lhs, const EventType& rhs) const {
      if (lhs.timestamp() != rhs.timestamp()) {
        return lhs.timestamp() > rhs.timestamp();
      }
      // Timestamps are equal, use priority as tiebreaker (lower priority values execute first)
      return lhs.priority() > rhs.priority();
    }
  };

  using QueueType = std::priority_queue<EventType, std::vector<EventType>, EventCompare>;

  QueueType queue_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_EVENT_QUEUE_ACTIVATION_QUEUE_HPP_
