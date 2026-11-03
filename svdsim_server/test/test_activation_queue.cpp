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
#include <algorithm>
#include <stdexcept>
#include <vector>

#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/event_queue/activation_event.hpp"
#include "svdsim_server/event_queue/activation_queue.hpp"

using svdsim_server::ActivationEvent;
using svdsim_server::ActivationQueue;
using svdsim_server::DiscreteTime;

namespace {

/**
 * @brief Simple payload for test invocations
 */
using DummyPayload = int;

/**
 * @brief Test activation event type using DummyPayload
 */
using DummyEvent = ActivationEvent<DummyPayload>;

}  // namespace

TEST(ActivationQueueTest, InsertAndFrontOrder) {
  ActivationQueue<DummyEvent> queue;
  queue.insert(DummyEvent(10, 100, 0));
  queue.insert(DummyEvent(5, 50, 0));
  queue.insert(DummyEvent(20, 200, 0));
  EXPECT_EQ(queue.size(), 3u);
  EXPECT_EQ(queue.front().timestamp(), 5u);
  EXPECT_EQ(queue.front().payload(), 50);
  EXPECT_EQ(queue.front().priority(), 0u);
}

TEST(ActivationQueueTest, PopRemovesFront) {
  ActivationQueue<DummyEvent> queue;
  queue.insert(DummyEvent(10, 100, 0));
  queue.insert(DummyEvent(5, 50, 0));
  queue.insert(DummyEvent(20, 200, 0));
  EXPECT_EQ(queue.front().timestamp(), 5u);
  EXPECT_EQ(queue.front().payload(), 50);
  queue.pop();
  EXPECT_EQ(queue.front().timestamp(), 10u);
  EXPECT_EQ(queue.front().payload(), 100);
  queue.pop();
  EXPECT_EQ(queue.front().timestamp(), 20u);
  EXPECT_EQ(queue.front().payload(), 200);
  queue.pop();
  EXPECT_TRUE(queue.empty());
}

TEST(ActivationQueueTest, EmptyThrowsOnFrontAndPop) {
  ActivationQueue<DummyEvent> queue;
  EXPECT_THROW({ [[maybe_unused]] const auto& tmp = queue.front(); }, std::runtime_error);
  EXPECT_THROW(queue.pop(), std::runtime_error);
}

TEST(ActivationQueueTest, InsertWithPayload) {
  ActivationQueue<DummyEvent> queue;
  queue.insert(DummyEvent(15, 42, 5));
  const auto& event = queue.front();
  EXPECT_EQ(event.timestamp(), 15u);
  EXPECT_EQ(event.payload(), 42);
  EXPECT_EQ(event.priority(), 5u);
}

TEST(ActivationQueueTest, InsertManyMaintainsOrder) {
  ActivationQueue<DummyEvent> queue;
  std::vector<DiscreteTime> times = {50, 10, 30, 40, 20};
  for (auto t : times) {
    queue.insert(DummyEvent(t, t.value(), 0));
  }
  std::sort(times.begin(), times.end());
  for (auto t : times) {
    ASSERT_FALSE(queue.empty());
    EXPECT_EQ(queue.front().timestamp(), t);
    EXPECT_EQ(queue.front().payload(), t.value());
    queue.pop();
  }
  EXPECT_TRUE(queue.empty());
}

TEST(ActivationQueueTest, PriorityOrderingSameTimestamp) {
  ActivationQueue<DummyEvent> queue;
  // Insert events with same timestamp but different priorities
  // Lower priority value = higher priority (executes first)
  queue.insert(DummyEvent(10, 103, 3));
  queue.insert(DummyEvent(10, 101, 1));
  queue.insert(DummyEvent(10, 102, 2));
  queue.insert(DummyEvent(10, 100, 0));

  EXPECT_EQ(queue.size(), 4u);

  // All have timestamp 10, should be ordered by priority (0, 1, 2, 3)
  EXPECT_EQ(queue.front().timestamp(), 10u);
  EXPECT_EQ(queue.front().priority(), 0u);
  EXPECT_EQ(queue.front().payload(), 100);
  queue.pop();

  EXPECT_EQ(queue.front().priority(), 1u);
  EXPECT_EQ(queue.front().payload(), 101);
  queue.pop();

  EXPECT_EQ(queue.front().priority(), 2u);
  EXPECT_EQ(queue.front().payload(), 102);
  queue.pop();

  EXPECT_EQ(queue.front().priority(), 3u);
  EXPECT_EQ(queue.front().payload(), 103);
  queue.pop();

  EXPECT_TRUE(queue.empty());
}

TEST(ActivationQueueTest, PriorityOrderingMixedTimestamps) {
  ActivationQueue<DummyEvent> queue;
  // Insert events with different timestamps and priorities
  queue.insert(DummyEvent(20, 200, 0));  // Later timestamp, high priority
  queue.insert(DummyEvent(10, 102, 2));  // Earlier timestamp, low priority
  queue.insert(DummyEvent(10, 100, 0));  // Earlier timestamp, high priority
  queue.insert(DummyEvent(15, 150, 1));  // Middle timestamp
  queue.insert(DummyEvent(10, 101, 1));  // Earlier timestamp, medium priority

  // Should be ordered first by timestamp, then by priority
  EXPECT_EQ(queue.front().timestamp(), 10u);
  EXPECT_EQ(queue.front().priority(), 0u);
  EXPECT_EQ(queue.front().payload(), 100);
  queue.pop();

  EXPECT_EQ(queue.front().timestamp(), 10u);
  EXPECT_EQ(queue.front().priority(), 1u);
  EXPECT_EQ(queue.front().payload(), 101);
  queue.pop();

  EXPECT_EQ(queue.front().timestamp(), 10u);
  EXPECT_EQ(queue.front().priority(), 2u);
  EXPECT_EQ(queue.front().payload(), 102);
  queue.pop();

  EXPECT_EQ(queue.front().timestamp(), 15u);
  EXPECT_EQ(queue.front().payload(), 150);
  queue.pop();

  EXPECT_EQ(queue.front().timestamp(), 20u);
  EXPECT_EQ(queue.front().payload(), 200);
  queue.pop();

  EXPECT_TRUE(queue.empty());
}

TEST(ActivationQueueTest, DefaultPriorityIsZero) {
  // Test that default priority is 0 when not specified
  const DummyEvent event(10, 100);
  EXPECT_EQ(event.priority(), 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
