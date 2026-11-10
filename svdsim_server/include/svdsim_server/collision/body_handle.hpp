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

#ifndef SVDSIM_SERVER_COLLISION_BODY_HANDLE_HPP_
#define SVDSIM_SERVER_COLLISION_BODY_HANDLE_HPP_

#include <any>
#include <cstddef>
#include <memory>
#include <typeinfo>

#include <box2d/box2d.h>
#include <sophus/se2.hpp>

#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {

/// Categories of physics bodies in the simulation.
enum class BodyCategory { VEHICLE, OBSTACLE, WALL };

/// RAII handle for managing Box2D bodies with type-erased user data.
class BodyHandle {
 public:
  /// Private constructor only accessible by ColliderManager.
  template <typename UserDataType>
  BodyHandle(std::size_t id, BodyCategory category, UserDataType userdata, b2Body* body, std::weak_ptr<b2World> world)
      : id_(id), category_(category), userdata_(std::move(userdata)), body_(body), world_(std::move(world)) {}

  /// Destroys the BodyHandle and unregisters the body from the world if still valid.
  ~BodyHandle() {
    if (auto world = world_.lock()) {
      world->DestroyBody(body_);
    }
  }

  /// Deleted copy constructor to prevent copying.
  BodyHandle(const BodyHandle&) = delete;

  /// Deleted copy assignment operator to prevent copying.
  BodyHandle& operator=(const BodyHandle&) = delete;

  /// Move constructor.
  BodyHandle(BodyHandle&& other) noexcept
      : id_(other.id_),
        category_(other.category_),
        userdata_(std::move(other.userdata_)),
        body_(other.body_),
        world_(std::move(other.world_)) {
    other.body_ = nullptr;
  }

  /// Move assignment operator.
  BodyHandle& operator=(BodyHandle&& other) noexcept {
    if (this != &other) {
      if (auto world = world_.lock()) {
        world->DestroyBody(body_);
      }
      id_ = other.id_;
      category_ = other.category_;
      userdata_ = std::move(other.userdata_);
      body_ = other.body_;
      world_ = std::move(other.world_);
      other.body_ = nullptr;
    }
    return *this;
  }

  /// Returns the unique identifier of this body.
  std::size_t id() const { return id_; }

  /// Returns the category of this body.
  BodyCategory category() const { return category_; }

  /// Returns a pointer to the user data if the type matches, nullptr otherwise.
  /// @tparam T The expected type of the user data.
  /// @return Pointer to user data if type matches, nullptr otherwise.
  template <typename T>
  T* user_data() {
    try {
      return std::any_cast<T>(&userdata_);
    } catch (const std::bad_any_cast&) {
      return nullptr;
    }
  }

  /// Returns a const pointer to the user data if the type matches, nullptr otherwise.
  /// @tparam T The expected type of the user data.
  /// @return Const pointer to user data if type matches, nullptr otherwise.
  template <typename T>
  const T* user_data() const {
    try {
      return std::any_cast<T>(&userdata_);
    } catch (const std::bad_any_cast&) {
      return nullptr;
    }
  }

  /// Returns the current pose of the body.
  /// @return SE2 pose representing position and orientation.
  Sophus::SE2d pose() const {
    const b2Vec2 position = body_->GetPosition();
    const auto angle = body_->GetAngle();
    return Sophus::SE2d(Sophus::SO2d(angle), Eigen::Vector2d(position.x, position.y));
  }

  /// Sets the pose of the body.
  /// @param new_pose SE2 pose to set.
  void set_pose(const Sophus::SE2d& new_pose) {
    const auto translation = new_pose.translation();
    const auto angle = static_cast<float>(new_pose.so2().log());
    body_->SetTransform(b2Vec2(static_cast<float>(translation.x()), static_cast<float>(translation.y())), angle);
  }

  /// Returns the Box2D body pointer.
  /// @return Pointer to the underlying b2Body.
  b2Body* body() const { return body_; }

 private:
  std::size_t id_;
  BodyCategory category_;
  std::any userdata_;
  b2Body* body_;
  std::weak_ptr<b2World> world_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_COLLISION_BODY_HANDLE_HPP_
