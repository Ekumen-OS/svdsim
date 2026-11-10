# GitHub Copilot Instructions for SVDSim

SVDSim is a standalone ROS 2 simulator for autonomous racing development. This
repository contains the simulation server, the ROS 2 bridge, the message
definitions, and the protobuf/ZMQ communication libraries.

## Packages

- `svdsim_server` - 2D physics simulation, vehicle dynamics, and sensor models
- `svdsim_bridge` - ROS 2 bridge between the ZMQ/protobuf server and ROS 2 topics
- `svdsim_msgs` - ROS 2 message definitions
- `svdsim_proto` - Protocol buffer definitions and ZMQ vehicle control libraries

## Key Points for Copilot

- Target ROS 2 Jazzy on Ubuntu 24.04
- Follow the ROS 2 component-based architecture (`rclcpp_components`)
- Keep the simulation loop deterministic and real-time friendly
- Use modern C++17 with Google style, `clang-format`, and the `pre-commit` hooks
- Keep the simulator usable both standalone and as a library from downstream projects
- Always include proper error handling and parameter validation
