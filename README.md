# SVD Simulator

A lightweight 2D simulator for autonomous racing development, designed for the Monaco RoboRacer project. The simulator provides realistic vehicle dynamics, sensor models, and environment simulation for testing and developing autonomous racing algorithms.

## Packages

- **svdsim_server**: Core simulation server implementing vehicle dynamics models, sensor models (IMU, LiDAR, cameras), and utilities for trajectory management and path following. Uses Box2D for efficient collision detection and raytracing. Provides ROS 2 component-based architecture with protobuf-based communication.

- **svdsim_bridge**: ROS 2 bridge for connecting the simulation server to the ROS 2 ecosystem, enabling integration with perception, planning, and control nodes.

- **svdsim_msgs**: ROS 2 message definitions for simulator communication.

- **svdsim_proto**: Protocol buffer definitions for efficient inter-process communication and data serialization.

## Running the Simulator

TBD: usage instructions will be added once a standalone example launch package is available for this repository.

### Key Features

- **Efficient 2D Physics**: Box2D-based collision detection and raytracing for fast, deterministic simulation
- **Advanced Vehicle Dynamics**: Modified Nicolas-Comstock (MNC) tire model for realistic combined longitudinal and lateral forces
- **High-Fidelity Sensors**: IMU with Allan variance noise modeling, LiDAR raytracing, and camera simulation
- **Component Architecture**: ROS 2 component-based design with protobuf communication for modularity and performance

## Bibliographical References

### State Estimation and Robotics

- Barfoot, T. D. (2017). *State Estimation for Robotics*. Cambridge University Press.
  - ISBN: 978-1107159396
  - Provides the theoretical foundation for IMU sensor models and coordinate frame transformations used in the simulator.
  - [Publisher Link](https://www.cambridge.org/core/books/state-estimation-for-robotics/AC4B5BD23A11EDADE0C4513DE6552E4A)

### Vehicle Dynamics

- Guiggiani, M. (2018). *The Science of Vehicle Dynamics: Handling, Braking, and Ride of Road and Race Cars*. 2nd Edition. Springer.
  - Advanced treatment of vehicle dynamics with emphasis on racing applications, tire mechanics, and handling characteristics.

- Liniger, A. (2012). *Autonomous Drift Control*. Master's Thesis, ETH Zurich.
  - Presents drift control techniques and vehicle dynamics models for autonomous drifting, including single-track drift models.

- Althoff, M., Koschi, M., and Manzinger, S. (2017). "CommonRoad: Composable Benchmarks for Motion Planning on Roads." *Proceedings of the IEEE Intelligent Vehicles Symposium*, pp. 719-726.
  - Provides standardized vehicle models and benchmarking framework for motion planning, used as reference for vehicle dynamics modeling.
  - [Project Website](https://commonroad.in.tum.de/)

- Milani, S., Marzbani, H., Khazaei, A., Simic, M., and Jazar, R. N. (2019). "Elliptical Combined-Slip Tire Model in Vehicle Dynamics." In *Innovation in Medicine and Healthcare Systems, and Multimedia*, Smart Innovation, Systems and Technologies, Vol. 145, pp. 457-468. Springer.
  - Presents an elliptical combined-slip tire model for improved vehicle dynamics simulation.
  - [DOI: 10.1007/978-981-13-8566-6_44](https://doi.org/10.1007/978-981-13-8566-6_44)

- Brach, R. M., and Brach, R. M. (2000). "Modeling Combined Braking and Steering Tire Forces." SAE Technical Paper 2000-01-0357.
  - Presents the Modified Nicolas-Comstock (MNC) tire model for combined longitudinal and lateral tire forces.
  - Provides equations for computing tire forces under simultaneous slip and sideslip conditions.
  - [DOI: 10.4271/2000-01-0357](https://doi.org/10.4271/2000-01-0357)
  - [ResearchGate](https://www.researchgate.net/publication/267196202_Modeling_Combined_Braking_and_Steering_Tire_Forces)

### IMU Noise Modeling

- Kalibr IMU Noise Model
  - IMU noise characterization methodology used for accelerometer and gyroscope simulation.
  - [Documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
  - Based on the Allan variance method for IMU characterization.

### Software Libraries

- **Box2D**: 2D Physics Engine
  - Catto, E. Box2D: A 2D physics engine for games.
  - [Project Website](https://box2d.org/)
  - [GitHub Repository](https://github.com/erincatto/box2d)

- **EnTT**: Entity Component System (ECS) Library
  - Corsi, M. EnTT - A fast and reliable entity component system (ECS).
  - [GitHub Repository](https://github.com/skypjack/entt)
  - [Documentation](https://skypjack.github.io/entt/)

- **Eigen**: C++ Template Library for Linear Algebra
  - Guennebaud, G., Jacob, B., et al. Eigen v3.
  - [Project Website](https://eigen.tuxfamily.org/)

- **Sophus**: C++ Lie Group Library
  - Strasdat, H. Sophus - C++ implementation of Lie Groups using Eigen.
  - [GitHub Repository](https://github.com/strasdat/Sophus)

- **Protocol Buffers**: Google's data interchange format
  - [Documentation](https://protobuf.dev/)
  - [GitHub Repository](https://github.com/protocolbuffers/protobuf)

### Additional Resources

- ROS 2 (Robot Operating System 2)
  - [Documentation](https://docs.ros.org/)
  - Component-based architecture framework used throughout the simulator.

## License

This project is licensed under the [Apache License 2.0](LICENSE).

### Third-Party Dependency Licenses

The simulator links against or builds with the following third-party components. All of them are permissively licensed or, in Eigen's and ZeroMQ's case, weak-copyleft at the file level, and none impose additional conditions on this project's Apache-2.0 license:

| Dependency | License | How it's used |
|---|---|---|
| ROS 2 (`rclcpp`, `std_msgs`, etc.) | Apache License 2.0 | System package, dynamically linked |
| OpenCV | Apache License 2.0 | System package (`libopencv-dev`), dynamically linked |
| Box2D | MIT License | System package (`box2d-dev`), dynamically linked |
| Sophus | MIT License | System package, header-only |
| yaml-cpp | MIT License | System package, dynamically linked |
| EnTT | MIT License | Fetched from its upstream GitHub repository at build time (not vendored in this repository), header-only |
| Protocol Buffers | BSD 3-Clause License | System package (`protobuf-dev`), dynamically linked; also used at build time to generate C++ code from `.proto` definitions |
| Eigen3 | Mozilla Public License 2.0 | System package, header-only, used unmodified |
| ZeroMQ (`libzmq`) | Mozilla Public License 2.0 | System package (`libzmq3-dev`), dynamically linked via `pkg-config` |
| cppzmq | MIT License | System package, header-only C++ bindings for ZeroMQ |

No GPL- or AGPL-licensed dependencies are used.
