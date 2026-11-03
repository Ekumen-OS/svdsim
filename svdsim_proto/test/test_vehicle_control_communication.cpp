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

#include <atomic>
#include <chrono>
#include <thread>

#include "svdsim_proto/vehicle_control_client.hpp"
#include "svdsim_proto/vehicle_control_server.hpp"

using svdsim_proto::VehicleControlClient;
using svdsim_proto::VehicleControlServer;
using svdsim_server::SimDataMessage;

constexpr int TEST_PORT = 4444;
constexpr int NUM_MESSAGES = 10;
constexpr int MESSAGE_INTERVAL_MS = 20;

TEST(VehicleControlCommunicationTest, PingPongMessages) {
  // Counters for received messages
  std::atomic<int> server_ping_count{0};
  std::atomic<int> server_pong_count{0};
  std::atomic<int> client_ping_count{0};
  std::atomic<int> client_pong_count{0};

  // Create server and client
  VehicleControlServer server(TEST_PORT);

  // Give server time to bind
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  VehicleControlClient client("localhost", TEST_PORT);

  // Give client time to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Set up server callback
  server.set_callback([&server_ping_count, &server_pong_count](const SimDataMessage& msg) {
    if (msg.has_server_inbound()) {
      const auto& inbound = msg.server_inbound();
      if (inbound.has_ping()) {
        server_ping_count++;
      }
    }
    if (msg.has_server_outbound()) {
      const auto& outbound = msg.server_outbound();
      if (outbound.has_pong()) {
        server_pong_count++;
      }
    }
  });

  // Set up client callback
  client.set_callback([&client_ping_count, &client_pong_count](const SimDataMessage& msg) {
    if (msg.has_server_inbound()) {
      const auto& inbound = msg.server_inbound();
      if (inbound.has_ping()) {
        client_ping_count++;
      }
    }
    if (msg.has_server_outbound()) {
      const auto& outbound = msg.server_outbound();
      if (outbound.has_pong()) {
        client_pong_count++;
      }
    }
  });

  // Send messages in alternating pattern
  for (int i = 0; i < NUM_MESSAGES; ++i) {
    // Client sends ping to server (server receives via ServerInbound)
    SimDataMessage ping_msg;
    auto* timestamp = ping_msg.mutable_timestamp();
    timestamp->set_seconds(0);
    timestamp->set_fraction(0.0);
    auto* inbound = ping_msg.mutable_server_inbound();
    auto* ping = inbound->mutable_ping();
    ping->set_sequence_number(i);
    ASSERT_TRUE(client.publish(ping_msg));
    ASSERT_TRUE(client.poll_and_receive(10));  // required for inproc forwarding
    ASSERT_FALSE(client.poll_and_receive());   // no more messages

    // Give time for message to arrive and be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(MESSAGE_INTERVAL_MS));
    ASSERT_TRUE(server.poll_and_receive(10));
    ASSERT_FALSE(server.poll_and_receive());  // no more messages

    // Server sends pong to client (client receives via ServerOutbound)
    SimDataMessage pong_msg;
    auto* timestamp2 = pong_msg.mutable_timestamp();
    timestamp2->set_seconds(0);
    timestamp2->set_fraction(0.0);
    auto* outbound = pong_msg.mutable_server_outbound();
    auto* pong = outbound->mutable_pong();
    pong->set_sequence_number(i);
    ASSERT_TRUE(server.publish(pong_msg));
    ASSERT_TRUE(server.poll_and_receive(10));  // required for inproc forwarding
    ASSERT_FALSE(server.poll_and_receive());   // no more messages

    // Give time for message to arrive and be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(MESSAGE_INTERVAL_MS));
    ASSERT_TRUE(client.poll_and_receive(10));
    ASSERT_FALSE(client.poll_and_receive());  // no more messages
  }

  // Final poll to ensure all messages are received
  for (int i = 0; i < 10; ++i) {
    (void)server.poll_and_receive(10);
    (void)client.poll_and_receive(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Verify all messages were received
  EXPECT_EQ(server_ping_count.load(), NUM_MESSAGES);
  EXPECT_EQ(server_pong_count.load(), 0);  // Server doesn't receive pong anymore
  EXPECT_EQ(client_ping_count.load(), 0);  // Client doesn't receive ping anymore
  EXPECT_EQ(client_pong_count.load(), NUM_MESSAGES);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
