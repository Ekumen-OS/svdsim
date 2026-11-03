// Copyright 2026 Ekumen
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

#include "svdsim_server/utils/trajectory_reader.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>

namespace svdsim_server {
namespace {

TEST(TrajectoryReaderTest, ReadValidTrajectory) {
  std::stringstream ss;
  ss << "x,y\n";
  ss << "0.0,0.0\n";
  ss << "1.0,0.0\n";
  ss << "2.0,0.0\n";
  ss << "3.0,0.0\n";

  TrajectoryReader reader;
  auto trajectory = reader.read_from_stream(ss);

  ASSERT_EQ(trajectory.size(), 4);
  EXPECT_DOUBLE_EQ(trajectory[0].x, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].y, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[3].x, 3.0);
  EXPECT_DOUBLE_EQ(trajectory[3].y, 0.0);
}

TEST(TrajectoryReaderTest, ReadTrajectoryWithExtraColumns) {
  std::stringstream ss;
  ss << "x,y,z,velocity\n";
  ss << "0.0,0.0,0.0,1.0\n";
  ss << "1.0,1.0,0.0,1.5\n";
  ss << "2.0,2.0,0.0,2.0\n";

  TrajectoryReader reader;
  auto trajectory = reader.read_from_stream(ss);

  ASSERT_EQ(trajectory.size(), 3);
  EXPECT_DOUBLE_EQ(trajectory[0].x, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].y, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[2].x, 2.0);
  EXPECT_DOUBLE_EQ(trajectory[2].y, 2.0);
}

TEST(TrajectoryReaderTest, ReadTrajectorySkipsEmptyLines) {
  std::stringstream ss;
  ss << "x,y\n";
  ss << "0.0,0.0\n";
  ss << "\n";
  ss << "1.0,1.0\n";
  ss << "  \n";
  ss << "2.0,2.0\n";

  TrajectoryReader reader;
  auto trajectory = reader.read_from_stream(ss);

  ASSERT_EQ(trajectory.size(), 3);
}

TEST(TrajectoryReaderTest, ReadTrajectorySkipsCommentLines) {
  std::stringstream ss;
  ss << "x,y\n";
  ss << "0.0,0.0\n";
  ss << "# This is a comment\n";
  ss << "1.0,1.0\n";
  ss << "2.0,2.0\n";

  TrajectoryReader reader;
  auto trajectory = reader.read_from_stream(ss);

  ASSERT_EQ(trajectory.size(), 3);
}

TEST(TrajectoryReaderTest, EmptyHeader) {
  std::stringstream ss;
  ss << "\n";
  ss << "0.0,0.0\n";

  TrajectoryReader reader;
  EXPECT_THROW(reader.read_from_stream(ss), std::runtime_error);
}

TEST(TrajectoryReaderTest, MissingXColumn) {
  std::stringstream ss;
  ss << "y,z\n";
  ss << "0.0,0.0\n";

  TrajectoryReader reader;
  EXPECT_THROW(reader.read_from_stream(ss), std::runtime_error);
}

TEST(TrajectoryReaderTest, MissingYColumn) {
  std::stringstream ss;
  ss << "x,z\n";
  ss << "0.0,0.0\n";

  TrajectoryReader reader;
  EXPECT_THROW(reader.read_from_stream(ss), std::runtime_error);
}

TEST(TrajectoryReaderTest, TooFewPoints) {
  std::stringstream ss;
  ss << "x,y\n";
  ss << "0.0,0.0\n";
  ss << "1.0,1.0\n";

  TrajectoryReader reader;
  EXPECT_THROW(reader.read_from_stream(ss), std::runtime_error);
}

TEST(TrajectoryReaderTest, InvalidNumberFormat) {
  std::stringstream ss;
  ss << "x,y\n";
  ss << "0.0,0.0\n";
  ss << "abc,1.0\n";
  ss << "2.0,2.0\n";

  TrajectoryReader reader;
  EXPECT_THROW(reader.read_from_stream(ss), std::runtime_error);
}

TEST(TrajectoryReaderTest, InfiniteValue) {
  std::stringstream ss;
  ss << "x,y\n";
  ss << "0.0,0.0\n";
  ss << "inf,1.0\n";
  ss << "2.0,2.0\n";

  TrajectoryReader reader;
  EXPECT_THROW(reader.read_from_stream(ss), std::runtime_error);
}

TEST(TrajectoryReaderTest, TrajectoryPointToVector) {
  TrajectoryPoint point(1.5, 2.5);
  Eigen::Vector2d vec = point.to_vector();

  EXPECT_DOUBLE_EQ(vec.x(), 1.5);
  EXPECT_DOUBLE_EQ(vec.y(), 2.5);
}

}  // namespace
}  // namespace svdsim_server
