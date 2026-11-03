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

#include "svdsim_server/collision/collider_manager.hpp"

#include <iostream>

namespace svdsim_server {

ColliderManager::ColliderManager() {
  b2Vec2 gravity(0.0f, 0.0f);
  world_ = std::make_shared<b2World>(gravity);
}

ColliderManager::~ColliderManager() = default;

double ColliderManager::raycast(const Sophus::SE2d& pose, float max_distance) {
  const Eigen::Vector2d start = pose.translation();
  const Eigen::Vector2d direction = pose.so2() * Eigen::Vector2d::UnitX();
  const Eigen::Vector2d end = start + direction * max_distance;

  const b2Vec2 p1(start.x(), start.y());
  const b2Vec2 p2(end.x(), end.y());

  float closest_fraction = 1.0f;

  class RayCastCallback : public b2RayCastCallback {
   public:
    explicit RayCastCallback(float& closest_fraction) : closest_fraction_(closest_fraction) {}

    float ReportFixture(
        [[maybe_unused]] b2Fixture* fixture,
        [[maybe_unused]] const b2Vec2& point,
        [[maybe_unused]] const b2Vec2& normal,
        float fraction) override {
      closest_fraction_ = fraction;
      return fraction;  // Clip ray to this hit for subsequent tests
    }

   private:
    float& closest_fraction_;
  };

  RayCastCallback ray_callback(closest_fraction);
  world_->RayCast(&ray_callback, p1, p2);

  return static_cast<double>(closest_fraction) * static_cast<double>(max_distance);
}

void ColliderManager::check_collisions(const BodyHandle& body, const CollisionCallback& callback) {
  // Get the AABB of the search body
  auto* search_body = body.body();
  b2AABB search_aabb;
  search_aabb.lowerBound = b2Vec2(FLT_MAX, FLT_MAX);
  search_aabb.upperBound = b2Vec2(-FLT_MAX, -FLT_MAX);

  // Combine AABBs from all fixtures on the body into a single encompassing bounding box
  for (b2Fixture* fixture = search_body->GetFixtureList(); fixture; fixture = fixture->GetNext()) {
    search_aabb.Combine(fixture->GetAABB(0));
  }

  // Query for bodies in the AABB region
  class AABBQueryCallback : public b2QueryCallback {
   public:
    AABBQueryCallback(b2Body* search_body, const ColliderManager::CollisionCallback& callback)
        : search_body_(search_body), callback_(callback) {}

    bool ReportFixture(b2Fixture* fixture) override {
      auto* other_body = fixture->GetBody();

      // Skip the search body itself
      if (other_body == search_body_) {
        return true;  // Continue query
      }

      // Test for collision between fixtures
      bool collision_found = false;
      for (b2Fixture* search_fixture = search_body_->GetFixtureList(); search_fixture && !collision_found;
           search_fixture = search_fixture->GetNext()) {
        for (b2Fixture* other_fixture = other_body->GetFixtureList(); other_fixture;
             other_fixture = other_fixture->GetNext()) {
          const auto xfA = search_body_->GetTransform();
          const auto xfB = other_body->GetTransform();

          if (b2TestOverlap(search_fixture->GetShape(), 0, other_fixture->GetShape(), 0, xfA, xfB)) {
            collision_found = true;
            break;
          }
        }
      }

      // Get the body handle from user data
      auto* other_handle = reinterpret_cast<BodyHandle*>(other_body->GetUserData().pointer);

      if (other_handle && collision_found) {
        callback_(*other_handle);
      }

      return true;  // Continue query
    }

   private:
    b2Body* search_body_;
    const ColliderManager::CollisionCallback& callback_;
  };

  AABBQueryCallback query_callback(search_body, callback);
  world_->QueryAABB(&query_callback, search_aabb);
}

}  // namespace svdsim_server
