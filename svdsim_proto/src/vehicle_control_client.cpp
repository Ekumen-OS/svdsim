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

#include "svdsim_proto/vehicle_control_client.hpp"

#include <iostream>
#include <stdexcept>

namespace svdsim_proto {

using svdsim_server::SimDataMessage;

VehicleControlClient::VehicleControlClient(const std::string& hostname, const int port)
    : context_(1),
      socket_(context_, zmq::socket_type::pair),
      inproc_socket_(context_, zmq::socket_type::pair),
      inproc_sender_(context_, zmq::socket_type::pair) {
  // Set send high water mark (only keep latest message)
  socket_.set(zmq::sockopt::sndhwm, kSendHighWaterMark);

  // Set up inproc socket for internal routing
  inproc_socket_.bind(kInprocAddress);
  inproc_sender_.connect(kInprocAddress);

  // Connect to the server
  const std::string address = "tcp://" + hostname + ":" + std::to_string(port);
  socket_.connect(address);
}

VehicleControlClient::~VehicleControlClient() {
  inproc_sender_.close();
  inproc_socket_.close();
  socket_.close();
  context_.close();
}

bool VehicleControlClient::publish(const SimDataMessage& message) {
  try {
    // Serialize the protobuf message
    const size_t size = message.ByteSizeLong();
    zmq::message_t zmq_msg(size);
    if (!message.SerializeToArray(zmq_msg.data(), size)) {
      std::cerr << "Failed to serialize message\n";
      return false;
    }

    // Send the message to inproc socket for internal routing
    const auto result = inproc_sender_.send(zmq_msg, zmq::send_flags::dontwait);

    return result.has_value();
  } catch (const zmq::error_t& e) {
    std::cerr << "ZMQ error in publish: " << e.what() << "\n";
    return false;
  }
}

void VehicleControlClient::set_callback(MessageCallback callback) {
  callback_ = std::move(callback);
}

bool VehicleControlClient::poll_and_receive(const int timeout_ms) {
  try {
    // Poll for incoming messages on both inproc and external sockets
    zmq::pollitem_t items[] = {
        {static_cast<void*>(inproc_socket_), 0, ZMQ_POLLIN, 0},  //
        {static_cast<void*>(socket_), 0, ZMQ_POLLIN, 0},         //
    };
    const int rc = zmq::poll(items, 2, std::chrono::milliseconds(timeout_ms));

    if (rc <= 0) {
      return false;
    }

    // Check if message received on inproc socket (outgoing messages to forward)
    if (items[0].revents & ZMQ_POLLIN) {
      zmq::message_t zmq_msg;
      const auto result = inproc_socket_.recv(zmq_msg, zmq::recv_flags::dontwait);
      if (result.has_value()) {
        // Forward the message to the external socket
        socket_.send(zmq_msg, zmq::send_flags::dontwait);
      }
    }

    // Check if message received on external socket (incoming messages)
    if (items[1].revents & ZMQ_POLLIN) {
      // Receive the message
      zmq::message_t zmq_msg;
      const auto result = socket_.recv(zmq_msg, zmq::recv_flags::dontwait);
      if (result.has_value()) {
        // Deserialize the protobuf message and forward to callback
        SimDataMessage message;
        const bool success = message.ParseFromArray(zmq_msg.data(), zmq_msg.size());
        if (success && callback_) {
          callback_(message);
        }
      }
    }

    return true;

  } catch (const zmq::error_t& e) {
    return false;
  }
}

}  // namespace svdsim_proto
