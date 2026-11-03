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

#include <box2d/box2d.h>
#include <gtest/gtest.h>

TEST(Box2DLinkingTest, CreateWorld) {
  // Define the gravity vector.
  b2Vec2 gravity(0.0f, -10.0f);

  // Construct a world object, which will hold and simulate the rigid bodies.
  b2World world(gravity);

  // Define the ground body.
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(0.0f, -10.0f);

  // Call the body factory which allocates memory for the ground body
  // from a pool and creates the ground box shape (also from a pool).
  // The body is also added to the world.
  b2Body* groundBody = world.CreateBody(&groundBodyDef);

  // Define the ground box shape.
  b2PolygonShape groundBox;

  // The extents are the half-widths of the box.
  groundBox.SetAsBox(50.0f, 10.0f);

  // Add the ground fixture to the ground body.
  groundBody->CreateFixture(&groundBox, 0.0f);

  // Prepare for simulation. Typically we use a time step of 1/60 of a
  // second (60Hz) and 10 iterations. This provides a high quality simulation
  // in most game scenarios.
  float timeStep = 1.0f / 60.0f;
  int32 velocityIterations = 6;
  int32 positionIterations = 2;

  // Instruct the world to perform a single step of simulation.
  // It is generally best to keep the time step and iterations fixed.
  world.Step(timeStep, velocityIterations, positionIterations);

  // Check that the world exists and stepped without crashing
  EXPECT_NE(groundBody, nullptr);
}
