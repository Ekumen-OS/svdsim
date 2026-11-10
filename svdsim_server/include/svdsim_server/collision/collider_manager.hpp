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

#ifndef SVDSIM_SERVER_COLLISION_COLLIDER_MANAGER_HPP_
#define SVDSIM_SERVER_COLLISION_COLLIDER_MANAGER_HPP_

#include <functional>
#include <memory>
#include <tuple>

#include <box2d/box2d.h>
#include <sophus/se2.hpp>

#include "svdsim_server/collision/body_handle.hpp"
#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {

class ColliderManager {
 public:
  /// Filter function type for raycast operations.
  using RaycastFilter = std::function<void(const BodyHandle&)>;

  /// Callback function type for collision detection.
  using CollisionCallback = std::function<void(const BodyHandle&)>;

  /// Constructs a ColliderManager with a Box2D world initialized with zero gravity.
  ColliderManager();

  /// Destroys the ColliderManager and releases all Box2D resources.
  ~ColliderManager();

  /// Deleted copy constructor to prevent copying.
  ColliderManager(const ColliderManager&) = delete;

  /// Deleted copy assignment operator to prevent copying.
  ColliderManager& operator=(const ColliderManager&) = delete;

  /// Creates a rectangular body with specified corners and userdata.
  /// @param initial_pose Initial pose of the body in world coordinates.
  /// @param category The category of the body being created.
  /// @param front_left Front left corner coordinates (x, y) relative to body origin.
  /// @param rear_right Rear right corner coordinates (x, y) relative to body origin.
  /// @param userdata User-defined data to associate with the body.
  /// @return Shared pointer to BodyHandle managing the created body, or nullptr if invalid coordinates.
  template <typename UserDataType>
  std::shared_ptr<BodyHandle> add_rectangular_body(
      const Sophus::SE2d& initial_pose,
      BodyCategory category,
      const std::tuple<float, float>& front_left,
      const std::tuple<float, float>& rear_right,
      UserDataType userdata) {
    const auto [x1, y1] = front_left;
    const auto [x2, y2] = rear_right;

    // Validate that front_left is actually front and left of rear_right
    if (x1 <= x2 || y1 <= y2) {
      return nullptr;
    }

    const float center_x = (x1 + x2) / 2.0f;
    const float center_y = (y1 + y2) / 2.0f;
    const float half_width = std::abs(x2 - x1) / 2.0f;
    const float half_height = std::abs(y2 - y1) / 2.0f;

    b2BodyDef bodyDef;
    bodyDef.type = b2_staticBody;
    bodyDef.position.Set(
        static_cast<float>(initial_pose.translation().x()), static_cast<float>(initial_pose.translation().y()));
    bodyDef.angle = static_cast<float>(initial_pose.so2().log());

    b2Body* body = world_->CreateBody(&bodyDef);

    b2PolygonShape box;
    box.SetAsBox(half_width, half_height, b2Vec2(center_x, center_y), 0.0f);

    body->CreateFixture(&box, 0.0f);

    const std::size_t id = next_id_++;

    auto handle = std::make_shared<BodyHandle>(id, category, std::move(userdata), body, world_);
    body->GetUserData().pointer = reinterpret_cast<uintptr_t>(handle.get());

    return handle;
  }

  /// Performs a raycast from the given pose along its x-axis direction. Notice that if the ray starts from
  /// within a body, it will not register a hit.
  /// @param pose Starting pose (SE2) for the raycast.
  /// @param max_distance Maximum distance to raycast.
  /// @return Distance to the closest hit, or max_distance if no hit occurred.
  double raycast(const Sophus::SE2d& pose, float max_distance);

  /// Checks for collisions with other bodies in the world.
  /// @param body The body to check collisions for.
  /// @param callback Callback invoked for each colliding body.
  void check_collisions(const BodyHandle& body, const CollisionCallback& callback);

 private:
  std::size_t next_id_{0};
  std::shared_ptr<b2World> world_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_COLLISION_COLLIDER_MANAGER_HPP_
