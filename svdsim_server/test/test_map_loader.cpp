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

// Non-ROS dependencies
#include <gtest/gtest.h>

// Standard library
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

// Project headers
#include "svdsim_server/utils/map_loader.hpp"

namespace svdsim_server {
namespace utils {
namespace {

// Helper function to create a test map YAML file
void create_test_map_yaml(
    const std::string& yaml_path,
    const std::string& image_name,
    double resolution,
    double origin_x,
    double origin_y,
    double origin_theta,
    const std::string& negate_value = "false") {
  std::cerr << "Creating test map YAML at: " << yaml_path << std::endl;
  std::ofstream yaml_file(yaml_path);
  yaml_file << std::fixed << std::setprecision(6);
  yaml_file << "image: " << image_name << "\n";
  yaml_file << "resolution: " << resolution << "\n";
  yaml_file << "origin: [" << origin_x << ", " << origin_y << ", " << origin_theta << "]\n";
  yaml_file << "negate: " << negate_value << "\n";
  yaml_file << "occupied_thresh: 0.65\n";
  yaml_file << "free_thresh: 0.196\n";
  yaml_file << "mode: trinary\n";
  yaml_file.close();
}

// Helper function to create a simple test PGM image
void create_test_pgm(const std::string& pgm_path, int width, int height) {
  std::ofstream pgm_file(pgm_path);
  pgm_file << "P5\n";
  pgm_file << width << " " << height << "\n";
  pgm_file << "255\n";

  // Create a simple pattern: top half of the image is occupied (0)
  // and bottom half is free (255).
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      uint8_t value = (y >= height / 2) ? 0 : 255;
      pgm_file.write(reinterpret_cast<const char*>(&value), 1);
    }
  }
  pgm_file.close();
}

class MapLoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create temporary directory for test files
    test_dir_ = std::filesystem::temp_directory_path() / "svdsim_server_test";
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test files
    if (std::filesystem::exists(test_dir_)) {
      std::filesystem::remove_all(test_dir_);
    }
  }

  std::filesystem::path test_dir_;
};

TEST_F(MapLoaderTest, LoadValidMapYaml) {
  // Create test files
  const std::string yaml_path = (test_dir_ / "test_map.yaml").string();
  const std::string pgm_path = (test_dir_ / "test_map.pgm").string();

  const int map_width = 10;
  const int map_height = 10;
  const double resolution = 0.05;

  create_test_pgm(pgm_path, map_width, map_height);
  create_test_map_yaml(yaml_path, "test_map.pgm", resolution, 0.0, 0.0, 0.0);

  // Load the map
  StaticMap map = load_map_from_yaml(yaml_path);

  // Verify map properties
  EXPECT_EQ(map.width, static_cast<std::size_t>(map_width));
  EXPECT_EQ(map.height, static_cast<std::size_t>(map_height));
  EXPECT_DOUBLE_EQ(map.resolution, resolution);
  EXPECT_EQ(map.occupancy_grid.size(), static_cast<std::size_t>(map_width * map_height));

  // Verify origin
  EXPECT_DOUBLE_EQ(map.origin.translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(map.origin.translation().y(), 0.0);
  EXPECT_NEAR(map.origin.so2().log(), 0.0, 1e-6);

  // Verify occupancy grid pattern (top half free, bottom half occupied)
  for (std::size_t y = 0; y < map.height; ++y) {
    for (std::size_t x = 0; x < map.width; ++x) {
      const std::size_t index = y * map.width + x;
      if (y < map.height / 2) {
        EXPECT_TRUE(map.occupancy_grid[index]) << "Cell (" << x << ", " << y << ") should be occupied";
      } else {
        EXPECT_FALSE(map.occupancy_grid[index]) << "Cell (" << x << ", " << y << ") should be free";
      }
    }
  }
}

TEST_F(MapLoaderTest, LoadMapWithNonZeroOrigin) {
  // Create test files with non-zero origin
  const std::string yaml_path = (test_dir_ / "test_map_origin.yaml").string();
  const std::string pgm_path = (test_dir_ / "test_map_origin.pgm").string();

  const double origin_x = -1.5;
  const double origin_y = 2.3;
  const double origin_theta = M_PI / 4.0;

  create_test_pgm(pgm_path, 5, 5);
  create_test_map_yaml(yaml_path, "test_map_origin.pgm", 0.1, origin_x, origin_y, origin_theta);

  // Load the map
  StaticMap map = load_map_from_yaml(yaml_path);

  // Verify origin
  EXPECT_DOUBLE_EQ(map.origin.translation().x(), origin_x);
  EXPECT_DOUBLE_EQ(map.origin.translation().y(), origin_y);
  EXPECT_NEAR(map.origin.so2().log(), origin_theta, 1e-6);
}

TEST_F(MapLoaderTest, MissingYamlFileThrowsException) {
  const std::string yaml_path = (test_dir_ / "nonexistent.yaml").string();

  EXPECT_THROW(
      {
        try {
          load_map_from_yaml(yaml_path);
        } catch (const std::runtime_error& e) {
          EXPECT_NE(std::string(e.what()).find("does not exist"), std::string::npos);
          throw;
        }
      },
      std::runtime_error);
}

TEST_F(MapLoaderTest, MissingImageFileThrowsException) {
  const std::string yaml_path = (test_dir_ / "test_map_no_image.yaml").string();

  create_test_map_yaml(yaml_path, "nonexistent.pgm", 0.05, 0.0, 0.0, 0.0);

  EXPECT_THROW(
      {
        try {
          load_map_from_yaml(yaml_path);
        } catch (const std::runtime_error& e) {
          EXPECT_NE(std::string(e.what()).find("does not exist"), std::string::npos);
          throw;
        }
      },
      std::runtime_error);
}

TEST_F(MapLoaderTest, InvalidYamlFormatThrowsException) {
  const std::string yaml_path = (test_dir_ / "invalid.yaml").string();

  std::ofstream yaml_file(yaml_path);
  yaml_file << "invalid: yaml: content:\n";
  yaml_file.close();

  EXPECT_THROW(load_map_from_yaml(yaml_path), std::runtime_error);
}

TEST_F(MapLoaderTest, NegateAsIntegerZero) {
  const std::string yaml_path = (test_dir_ / "test_map_negate_int0.yaml").string();
  const std::string pgm_path = (test_dir_ / "test_map_negate_int0.pgm").string();

  create_test_pgm(pgm_path, 5, 5);
  create_test_map_yaml(yaml_path, "test_map_negate_int0.pgm", 0.05, 0.0, 0.0, 0.0, "0");

  // Should load successfully with negate=false
  // Cell (0,0) has pixel value 255 (white), which without negate means occupied
  StaticMap map = load_map_from_yaml(yaml_path);
  EXPECT_TRUE(map.occupancy_grid[0]) << "Cell (0,0) should be occupied when negate=0";
}

TEST_F(MapLoaderTest, NegateAsIntegerOne) {
  const std::string yaml_path = (test_dir_ / "test_map_negate_int1.yaml").string();
  const std::string pgm_path = (test_dir_ / "test_map_negate_int1.pgm").string();

  create_test_pgm(pgm_path, 5, 5);
  create_test_map_yaml(yaml_path, "test_map_negate_int1.pgm", 0.05, 0.0, 0.0, 0.0, "1");

  // Should load successfully with negate=true
  // Cell (0,0) has pixel value 255 (white), which with negate means free
  StaticMap map = load_map_from_yaml(yaml_path);
  EXPECT_FALSE(map.occupancy_grid[0]) << "Cell (0,0) should be free when negate=1";
}

TEST_F(MapLoaderTest, NegateAsBooleanTrue) {
  const std::string yaml_path = (test_dir_ / "test_map_negate_true.yaml").string();
  const std::string pgm_path = (test_dir_ / "test_map_negate_true.pgm").string();

  create_test_pgm(pgm_path, 5, 5);
  create_test_map_yaml(yaml_path, "test_map_negate_true.pgm", 0.05, 0.0, 0.0, 0.0, "true");

  // Should load successfully with negate=true
  // Cell (0,0) has pixel value 255 (white), which with negate means free
  StaticMap map = load_map_from_yaml(yaml_path);
  EXPECT_FALSE(map.occupancy_grid[0]) << "Cell (0,0) should be free when negate=true";
}

TEST_F(MapLoaderTest, NegateAsBooleanFalse) {
  const std::string yaml_path = (test_dir_ / "test_map_negate_false.yaml").string();
  const std::string pgm_path = (test_dir_ / "test_map_negate_false.pgm").string();

  create_test_pgm(pgm_path, 5, 5);
  create_test_map_yaml(yaml_path, "test_map_negate_false.pgm", 0.05, 0.0, 0.0, 0.0, "false");

  // Should load successfully with negate=false
  // Cell (0,0) has pixel value 255 (white), which without negate means occupied
  StaticMap map = load_map_from_yaml(yaml_path);
  EXPECT_TRUE(map.occupancy_grid[0]) << "Cell (0,0) should be occupied when negate=false";
}

TEST_F(MapLoaderTest, NegateInvalidValueThrowsException) {
  const std::string yaml_path = (test_dir_ / "test_map_negate_invalid.yaml").string();
  const std::string pgm_path = (test_dir_ / "test_map_negate_invalid.pgm").string();

  create_test_pgm(pgm_path, 5, 5);
  create_test_map_yaml(yaml_path, "test_map_negate_invalid.pgm", 0.05, 0.0, 0.0, 0.0, "invalid_string");

  EXPECT_THROW(
      {
        try {
          load_map_from_yaml(yaml_path);
        } catch (const std::runtime_error& e) {
          EXPECT_NE(std::string(e.what()).find("negate"), std::string::npos);
          throw;
        }
      },
      std::runtime_error);
}

TEST_F(MapLoaderTest, EmptyImageFileThrowsException) {
  const std::string yaml_path = (test_dir_ / "test_map_empty.yaml").string();
  const std::string image_path = (test_dir_ / "test_map_empty.pgm").string();

  // Create an empty/invalid image file
  std::ofstream image_file(image_path);
  image_file.close();

  create_test_map_yaml(yaml_path, "test_map_empty.pgm", 0.05, 0.0, 0.0, 0.0);

  EXPECT_THROW(
      {
        try {
          load_map_from_yaml(yaml_path);
        } catch (const std::runtime_error& e) {
          EXPECT_NE(std::string(e.what()).find("Failed to load map image"), std::string::npos);
          throw;
        }
      },
      std::runtime_error);
}

TEST_F(MapLoaderTest, CorruptedImageFileThrowsException) {
  const std::string yaml_path = (test_dir_ / "test_map_corrupted.yaml").string();
  const std::string image_path = (test_dir_ / "test_map_corrupted.pgm").string();

  // Create a corrupted PGM file with invalid header
  std::ofstream image_file(image_path);
  image_file << "P5\n";
  image_file << "10 10\n";
  image_file << "255\n";
  // Write insufficient data
  for (int i = 0; i < 10; ++i) {
    uint8_t value = 128;
    image_file.write(reinterpret_cast<const char*>(&value), 1);
  }
  image_file.close();

  create_test_map_yaml(yaml_path, "test_map_corrupted.pgm", 0.05, 0.0, 0.0, 0.0);

  // OpenCV may still load this but with invalid dimensions, or fail to load entirely
  EXPECT_THROW(
      {
        try {
          load_map_from_yaml(yaml_path);
        } catch (const std::runtime_error& e) {
          // Should fail either due to loading failure or invalid dimensions
          const std::string error_msg = e.what();
          const bool has_expected_error = (error_msg.find("Failed to load map image") != std::string::npos) ||
                                          (error_msg.find("Invalid image dimensions") != std::string::npos);
          EXPECT_TRUE(has_expected_error) << "Unexpected error message: " << error_msg;
          throw;
        }
      },
      std::runtime_error);
}

}  // namespace
}  // namespace utils
}  // namespace svdsim_server

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
