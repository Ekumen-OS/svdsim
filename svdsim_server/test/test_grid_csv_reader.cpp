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

#include "svdsim_server/configuration/grid_csv_reader.hpp"

#include <cmath>
#include <sstream>

#include <gtest/gtest.h>

namespace svdsim_server {
namespace {

// Test fixture for GridCsvReader tests
class GridCsvReaderTest : public ::testing::Test {
 protected:
  GridCsvReader reader_;
};

// Test successful parsing of valid CSV with one vehicle
TEST_F(GridCsvReaderTest, ParseValidCsvSingleVehicle) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_EQ(entries[0].id, "vehicle1");
  EXPECT_DOUBLE_EQ(entries[0].init_x, 1.5);
  EXPECT_DOUBLE_EQ(entries[0].init_y, 2.5);
  EXPECT_DOUBLE_EQ(entries[0].init_yaw, 0.785);
  EXPECT_EQ(entries[0].port, 9000);
  EXPECT_EQ(entries[0].type, VehicleType::REMOTE);
}

// Test successful parsing of valid CSV with multiple vehicles
TEST_F(GridCsvReaderTest, ParseValidCsvMultipleVehicles) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.0,2.0,0.0,9000,remote\n";
  ss << "vehicle2,3.0,4.0,1.57,9001,sparring\n";
  ss << "vehicle3,-1.0,-2.0,-3.14,9002,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 3);

  EXPECT_EQ(entries[0].id, "vehicle1");
  EXPECT_DOUBLE_EQ(entries[0].init_x, 1.0);
  EXPECT_DOUBLE_EQ(entries[0].init_y, 2.0);
  EXPECT_DOUBLE_EQ(entries[0].init_yaw, 0.0);
  EXPECT_EQ(entries[0].port, 9000);
  EXPECT_EQ(entries[0].type, VehicleType::REMOTE);

  EXPECT_EQ(entries[1].id, "vehicle2");
  EXPECT_DOUBLE_EQ(entries[1].init_x, 3.0);
  EXPECT_DOUBLE_EQ(entries[1].init_y, 4.0);
  EXPECT_DOUBLE_EQ(entries[1].init_yaw, 1.57);
  EXPECT_EQ(entries[1].port, 9001);
  EXPECT_EQ(entries[1].type, VehicleType::SPARRING);

  EXPECT_EQ(entries[2].id, "vehicle3");
  EXPECT_DOUBLE_EQ(entries[2].init_x, -1.0);
  EXPECT_DOUBLE_EQ(entries[2].init_y, -2.0);
  EXPECT_DOUBLE_EQ(entries[2].init_yaw, -3.14);
  EXPECT_EQ(entries[2].port, 9002);
  EXPECT_EQ(entries[2].type, VehicleType::REMOTE);
}

// Test parsing CSV with whitespace
TEST_F(GridCsvReaderTest, ParseCsvWithWhitespace) {
  // Arrange
  std::stringstream ss;
  ss << "id, initx, inity, inityaw, port, type\n";
  ss << "  vehicle1  ,  1.5  ,  2.5  ,  0.785  ,  9000  ,  remote  \n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_EQ(entries[0].id, "vehicle1");
  EXPECT_DOUBLE_EQ(entries[0].init_x, 1.5);
  EXPECT_DOUBLE_EQ(entries[0].init_y, 2.5);
  EXPECT_DOUBLE_EQ(entries[0].init_yaw, 0.785);
  EXPECT_EQ(entries[0].port, 9000);
  EXPECT_EQ(entries[0].type, VehicleType::REMOTE);
}

// Test parsing CSV with empty lines
TEST_F(GridCsvReaderTest, ParseCsvWithEmptyLines) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";
  ss << "  \n";
  ss << "vehicle2,3.0,4.0,1.57,9001,sparring\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 2);
  EXPECT_EQ(entries[0].id, "vehicle1");
  EXPECT_EQ(entries[1].id, "vehicle2");
}

// Test parsing CSV with comments
TEST_F(GridCsvReaderTest, ParseCsvWithComments) {
  // Arrange
  std::stringstream ss;
  ss << "# This is a comment\n";
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "# Another comment\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_EQ(entries[0].id, "vehicle1");
}

// Test parsing CSV with quoted fields
TEST_F(GridCsvReaderTest, ParseCsvWithQuotedFields) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "\"vehicle1\",\"1.5\",\"2.5\",\"0.785\",\"9000\",\"remote\"\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_EQ(entries[0].id, "vehicle1");
  EXPECT_DOUBLE_EQ(entries[0].init_x, 1.5);
}

// Test parsing vehicle IDs with special characters
TEST_F(GridCsvReaderTest, ParseVehicleIdWithSpecialCharacters) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle_1,1.5,2.5,0.785,9000,remote\n";
  ss << "vehicle-2,3.0,4.0,1.57,9001,sparring\n";
  ss << "Vehicle123,5.0,6.0,0.0,9002,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 3);
  EXPECT_EQ(entries[0].id, "vehicle_1");
  EXPECT_EQ(entries[1].id, "vehicle-2");
  EXPECT_EQ(entries[2].id, "Vehicle123");
}

// Test parsing negative numbers
TEST_F(GridCsvReaderTest, ParseNegativeNumbers) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,-1.5,-2.5,-0.785,9000,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_DOUBLE_EQ(entries[0].init_x, -1.5);
  EXPECT_DOUBLE_EQ(entries[0].init_y, -2.5);
  EXPECT_DOUBLE_EQ(entries[0].init_yaw, -0.785);
}

// Test parsing scientific notation
TEST_F(GridCsvReaderTest, ParseScientificNotation) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5e2,2.5e-1,7.85e-1,9000,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_DOUBLE_EQ(entries[0].init_x, 150.0);
  EXPECT_DOUBLE_EQ(entries[0].init_y, 0.25);
  EXPECT_DOUBLE_EQ(entries[0].init_yaw, 0.785);
}

// Test case-insensitive header validation
TEST_F(GridCsvReaderTest, ParseCaseInsensitiveHeader) {
  // Arrange
  std::stringstream ss;
  ss << "ID,INITX,INITY,INITYAW,PORT,TYPE\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_EQ(entries[0].id, "vehicle1");
}

// Error tests

// Test empty CSV throws error
TEST_F(GridCsvReaderTest, EmptyCsvThrowsError) {
  // Arrange
  std::stringstream ss;

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test CSV with only header throws error
TEST_F(GridCsvReaderTest, OnlyHeaderThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test wrong number of header columns throws error
TEST_F(GridCsvReaderTest, WrongHeaderColumnCountThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test wrong header column names throws error
TEST_F(GridCsvReaderTest, WrongHeaderNamesThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "name,x,y,yaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test wrong number of data columns throws error
TEST_F(GridCsvReaderTest, WrongDataColumnCountThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test empty vehicle ID throws error
TEST_F(GridCsvReaderTest, EmptyVehicleIdThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << ",1.5,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test invalid vehicle ID characters throws error
TEST_F(GridCsvReaderTest, InvalidVehicleIdCharactersThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle 1,1.5,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test duplicate vehicle ID throws error
TEST_F(GridCsvReaderTest, DuplicateVehicleIdThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";
  ss << "vehicle1,3.0,4.0,1.57,9001,sparring\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test duplicate port throws error
TEST_F(GridCsvReaderTest, DuplicatePortThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";
  ss << "vehicle2,3.0,4.0,1.57,9000,sparring\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test empty field throws error
TEST_F(GridCsvReaderTest, EmptyInitXThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test invalid double throws error
TEST_F(GridCsvReaderTest, InvalidDoubleThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,abc,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test invalid integer throws error
TEST_F(GridCsvReaderTest, InvalidIntegerThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,abc,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test port out of range (too low) throws error
TEST_F(GridCsvReaderTest, PortTooLowThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,0,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test port out of range (too high) throws error
TEST_F(GridCsvReaderTest, PortTooHighThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,65536,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test partial number parsing throws error
TEST_F(GridCsvReaderTest, PartialNumberParsingThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5abc,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test infinity throws error
TEST_F(GridCsvReaderTest, InfinityThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,inf,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test NaN throws error
TEST_F(GridCsvReaderTest, NaNThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,nan,2.5,0.785,9000,remote\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// VehicleType column tests

// Test explicit remote type
TEST_F(GridCsvReaderTest, ParseExplicitRemoteType) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,remote\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_EQ(entries[0].type, VehicleType::REMOTE);
}

// Test explicit sparring type
TEST_F(GridCsvReaderTest, ParseExplicitSparringType) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "ghost,2.0,0.0,0.0,9998,sparring\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 1);
  EXPECT_EQ(entries[0].id, "ghost");
  EXPECT_EQ(entries[0].type, VehicleType::SPARRING);
}

// Test mixed remote and sparring vehicles
TEST_F(GridCsvReaderTest, ParseMixedVehicleTypes) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,0.0,0.0,0.0,9999,remote\n";
  ss << "ghost,2.0,0.0,0.0,9998,sparring\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 2);
  EXPECT_EQ(entries[0].id, "vehicle1");
  EXPECT_EQ(entries[0].type, VehicleType::REMOTE);
  EXPECT_EQ(entries[1].id, "ghost");
  EXPECT_EQ(entries[1].type, VehicleType::SPARRING);
}

// Test case-insensitive type parsing
TEST_F(GridCsvReaderTest, ParseCaseInsensitiveVehicleType) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,REMOTE\n";
  ss << "vehicle2,3.0,4.0,1.57,9001,Sparring\n";

  // Act
  auto entries = reader_ << ss;

  // Assert
  ASSERT_EQ(entries.size(), 2);
  EXPECT_EQ(entries[0].type, VehicleType::REMOTE);
  EXPECT_EQ(entries[1].type, VehicleType::SPARRING);
}

// Test invalid vehicle type throws error
TEST_F(GridCsvReaderTest, InvalidVehicleTypeThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,invalid\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

// Test empty type field throws error
TEST_F(GridCsvReaderTest, EmptyVehicleTypeThrowsError) {
  // Arrange
  std::stringstream ss;
  ss << "id,initx,inity,inityaw,port,type\n";
  ss << "vehicle1,1.5,2.5,0.785,9000,\n";

  // Act & Assert
  EXPECT_THROW(reader_ << ss, std::runtime_error);
}

}  // namespace
}  // namespace svdsim_server

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
