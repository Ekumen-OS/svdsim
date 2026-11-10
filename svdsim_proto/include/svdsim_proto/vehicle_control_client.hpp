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

#ifndef SVDSIM_PROTO_VEHICLE_CONTROL_CLIENT_HPP_
#define SVDSIM_PROTO_VEHICLE_CONTROL_CLIENT_HPP_

// Standard library
#include <functional>
#include <memory>
#include <string>

// Third-party libraries
#include <zmq.hpp>

// Generated protobuf headers
#include "sim_messages.pb.h"

namespace svdsim_proto {

/**
 * @brief ZeroMQ-based client for vehicle control communication
 *
 * This class implements a bidirectional P2P communication client using ZeroMQ PAIR sockets.
 * It can send and receive SimDataMessage protobuf messages.
 */
class VehicleControlClient {
 public:
  using MessageCallback = std::function<void(const svdsim_server::SimDataMessage&)>;

  /**
   * @brief Construct a new Vehicle Control Client
   *
   * @param hostname The hostname or IP address of the server
   * @param port The port number to connect to
   */
  explicit VehicleControlClient(const std::string& hostname, int port);

  /**
   * @brief Destructor
   */
  ~VehicleControlClient();

  /**
   * @brief Publish a message to the server
   *
   * @param message The SimDataMessage to send
   * @return bool True if the message was sent successfully, false otherwise
   */
  [[nodiscard]] bool publish(const svdsim_server::SimDataMessage& message);

  /**
   * @brief Set the callback function for received messages
   *
   * @param callback Function to call when a message is received
   */
  void set_callback(MessageCallback callback);

  /**
   * @brief Poll for incoming messages and invoke callback if available
   *
   * @param timeout_ms Timeout in milliseconds (0 for non-blocking)
   * @return bool True if a message was received, false otherwise
   */
  [[nodiscard]] bool poll_and_receive(int timeout_ms = 0);

 private:
  static constexpr int kSendHighWaterMark = 100;
  static constexpr char kInprocAddress[] = "inproc://vehicle_control_client";

  zmq::context_t context_;
  zmq::socket_t socket_;
  zmq::socket_t inproc_socket_;
  zmq::socket_t inproc_sender_;
  MessageCallback callback_;
};

}  // namespace svdsim_proto

#endif  // SVDSIM_PROTO_VEHICLE_CONTROL_CLIENT_HPP_
