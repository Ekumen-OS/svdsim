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

#include <gtest/gtest.h>

#include "svdsim_server/wall_time.hpp"

namespace svdsim_server {

class WallTimeTest : public ::testing::Test {
 protected:
  void SetUp() override { tolerance_ = 1e-9; }

  double tolerance_;
};

TEST_F(WallTimeTest, FromSeconds) {
  // Test positive time
  auto ts1 = WallTime::from_seconds(5.75);
  EXPECT_EQ(ts1.seconds, 5);
  EXPECT_NEAR(ts1.fractional, 0.75, tolerance_);

  // Test zero
  auto ts2 = WallTime::from_seconds(0.0);
  EXPECT_EQ(ts2.seconds, 0);
  EXPECT_NEAR(ts2.fractional, 0.0, tolerance_);

  // Test negative time
  auto ts3 = WallTime::from_seconds(-3.25);
  EXPECT_EQ(ts3.seconds, -4);
  EXPECT_NEAR(ts3.fractional, 0.75, tolerance_);

  // Test very small fractional part
  auto ts4 = WallTime::from_seconds(10.0001);
  EXPECT_EQ(ts4.seconds, 10);
  EXPECT_NEAR(ts4.fractional, 0.0001, tolerance_);

  // Test integer seconds
  auto ts5 = WallTime::from_seconds(42.0);
  EXPECT_EQ(ts5.seconds, 42);
  EXPECT_NEAR(ts5.fractional, 0.0, tolerance_);
}

TEST_F(WallTimeTest, ToSeconds) {
  WallTime ts;

  ts.seconds = 5;
  ts.fractional = 0.75;
  EXPECT_NEAR(ts.to_seconds(), 5.75, tolerance_);

  ts.seconds = 0;
  ts.fractional = 0.0;
  EXPECT_NEAR(ts.to_seconds(), 0.0, tolerance_);

  ts.seconds = -4;
  ts.fractional = 0.75;
  EXPECT_NEAR(ts.to_seconds(), -3.25, tolerance_);

  ts.seconds = 100;
  ts.fractional = 0.123456;
  EXPECT_NEAR(ts.to_seconds(), 100.123456, tolerance_);
}

TEST_F(WallTimeTest, RoundTrip) {
  std::vector<double> test_values = {0.0, 1.5, 10.0, 123.456, -5.25, 0.0001};

  for (double value : test_values) {
    auto ts = WallTime::from_seconds(value);
    double result = ts.to_seconds();
    EXPECT_NEAR(result, value, tolerance_);
  }
}

TEST_F(WallTimeTest, Equality) {
  WallTime ts1{5, 0.75};
  WallTime ts2{5, 0.75};
  WallTime ts3{5, 0.76};
  WallTime ts4{6, 0.75};

  EXPECT_TRUE(ts1 == ts2);
  EXPECT_FALSE(ts1 == ts3);
  EXPECT_FALSE(ts1 == ts4);

  EXPECT_FALSE(ts1 != ts2);
  EXPECT_TRUE(ts1 != ts3);
  EXPECT_TRUE(ts1 != ts4);
}

TEST_F(WallTimeTest, Comparison) {
  WallTime ts1{5, 0.75};
  WallTime ts2{5, 0.75};
  WallTime ts3{5, 0.76};
  WallTime ts4{6, 0.75};
  WallTime ts5{4, 0.9};
  WallTime ts6{5, 0.5};

  // Less than
  EXPECT_FALSE(ts1 < ts2);  // Equal
  EXPECT_TRUE(ts1 < ts3);   // Same second, smaller fractional
  EXPECT_TRUE(ts1 < ts4);   // Smaller second
  EXPECT_FALSE(ts1 < ts5);  // Larger second
  EXPECT_FALSE(ts1 < ts6);  // Same second, larger fractional

  // Less than or equal
  EXPECT_TRUE(ts1 <= ts2);   // Equal
  EXPECT_TRUE(ts1 <= ts3);   // Smaller
  EXPECT_TRUE(ts1 <= ts4);   // Smaller
  EXPECT_FALSE(ts1 <= ts5);  // Larger
  EXPECT_FALSE(ts1 <= ts6);  // Larger

  // Greater than
  EXPECT_FALSE(ts1 > ts2);  // Equal
  EXPECT_FALSE(ts1 > ts3);  // Smaller
  EXPECT_FALSE(ts1 > ts4);  // Smaller
  EXPECT_TRUE(ts1 > ts5);   // Larger
  EXPECT_TRUE(ts1 > ts6);   // Larger

  // Greater than or equal
  EXPECT_TRUE(ts1 >= ts2);   // Equal
  EXPECT_FALSE(ts1 >= ts3);  // Smaller
  EXPECT_FALSE(ts1 >= ts4);  // Smaller
  EXPECT_TRUE(ts1 >= ts5);   // Larger
  EXPECT_TRUE(ts1 >= ts6);   // Larger
}

TEST_F(WallTimeTest, ComparisonNegative) {
  WallTime ts1{-5, 0.25};
  WallTime ts2{-5, 0.75};
  WallTime ts3{-4, 0.5};
  WallTime ts4{-6, 0.9};

  // With negative seconds
  EXPECT_TRUE(ts1 < ts2);   // Same negative second, smaller fractional
  EXPECT_TRUE(ts1 < ts3);   // Less negative (larger)
  EXPECT_FALSE(ts1 < ts4);  // More negative (smaller)

  EXPECT_FALSE(ts1 > ts2);  // Same negative second, smaller fractional
  EXPECT_FALSE(ts1 > ts3);  // Less negative (larger)
  EXPECT_TRUE(ts1 > ts4);   // More negative (smaller)
}

TEST_F(WallTimeTest, ComparisonZero) {
  WallTime ts_zero{0, 0.0};
  WallTime ts_positive{1, 0.0};
  WallTime ts_negative{-1, 0.0};
  WallTime ts_small_frac{0, 0.001};

  EXPECT_TRUE(ts_zero < ts_positive);
  EXPECT_FALSE(ts_zero < ts_negative);
  EXPECT_TRUE(ts_zero < ts_small_frac);

  EXPECT_FALSE(ts_zero > ts_positive);
  EXPECT_TRUE(ts_zero > ts_negative);
  EXPECT_FALSE(ts_zero > ts_small_frac);

  EXPECT_TRUE(ts_zero <= ts_zero);
  EXPECT_TRUE(ts_zero >= ts_zero);
}

TEST_F(WallTimeTest, Addition) {
  WallTime ts1{5, 0.75};
  WallTime ts2{3, 0.5};
  WallTime result = ts1 + ts2;

  EXPECT_NEAR(result.to_seconds(), 9.25, tolerance_);
  EXPECT_EQ(result.seconds, 9);
  EXPECT_NEAR(result.fractional, 0.25, tolerance_);

  // Test with overflow of fractional part
  WallTime ts3{2, 0.7};
  WallTime ts4{1, 0.8};
  WallTime result2 = ts3 + ts4;

  EXPECT_NEAR(result2.to_seconds(), 4.5, tolerance_);
  EXPECT_EQ(result2.seconds, 4);
  EXPECT_NEAR(result2.fractional, 0.5, tolerance_);

  // Test with zero
  WallTime ts_zero{0, 0.0};
  WallTime result3 = ts1 + ts_zero;
  EXPECT_TRUE(result3 == ts1);
}

TEST_F(WallTimeTest, Subtraction) {
  WallTime ts1{5, 0.75};
  WallTime ts2{3, 0.5};
  WallTime result = ts1 - ts2;

  EXPECT_NEAR(result.to_seconds(), 2.25, tolerance_);
  EXPECT_EQ(result.seconds, 2);
  EXPECT_NEAR(result.fractional, 0.25, tolerance_);

  // Test with underflow of fractional part
  WallTime ts3{5, 0.3};
  WallTime ts4{2, 0.8};
  WallTime result2 = ts3 - ts4;

  EXPECT_NEAR(result2.to_seconds(), 2.5, tolerance_);
  EXPECT_EQ(result2.seconds, 2);
  EXPECT_NEAR(result2.fractional, 0.5, tolerance_);

  // Test resulting in zero
  WallTime result3 = ts1 - ts1;
  EXPECT_NEAR(result3.to_seconds(), 0.0, tolerance_);

  // Test resulting in negative
  WallTime result4 = ts2 - ts1;
  EXPECT_NEAR(result4.to_seconds(), -2.25, tolerance_);
}

TEST_F(WallTimeTest, AdditionNegative) {
  WallTime ts1{-3, 0.25};
  WallTime ts2{2, 0.5};
  WallTime result = ts1 + ts2;

  EXPECT_NEAR(result.to_seconds(), -0.25, tolerance_);

  // Test negative + negative
  WallTime ts3{-5, 0.75};
  WallTime ts4{-2, 0.5};
  WallTime result2 = ts3 + ts4;

  EXPECT_NEAR(result2.to_seconds(), -5.75, tolerance_);
}

TEST_F(WallTimeTest, SubtractionNegative) {
  WallTime ts1{3, 0.5};
  WallTime ts2{-2, 0.25};
  WallTime result = ts1 - ts2;

  EXPECT_NEAR(result.to_seconds(), 5.25, tolerance_);

  // Test negative - positive
  WallTime ts3{-3, 0.5};
  WallTime ts4{2, 0.25};
  WallTime result2 = ts3 - ts4;

  EXPECT_NEAR(result2.to_seconds(), -4.75, tolerance_);
}

TEST_F(WallTimeTest, AdditionAssignment) {
  WallTime ts1{5, 0.75};
  WallTime ts2{3, 0.5};
  WallTime original = ts1;

  ts1 += ts2;

  EXPECT_NEAR(ts1.to_seconds(), 9.25, tolerance_);
  EXPECT_EQ(ts1.seconds, 9);
  EXPECT_NEAR(ts1.fractional, 0.25, tolerance_);

  // Verify original hasn't changed
  EXPECT_NEAR(original.to_seconds(), 5.75, tolerance_);

  // Chain operations
  WallTime ts3{1, 0.0};
  ts3 += ts1 += ts2;
  EXPECT_NEAR(ts3.to_seconds(), 13.75, tolerance_);
  EXPECT_NEAR(ts1.to_seconds(), 12.75, tolerance_);
}

TEST_F(WallTimeTest, SubtractionAssignment) {
  WallTime ts1{5, 0.75};
  WallTime ts2{3, 0.5};
  WallTime original = ts1;

  ts1 -= ts2;

  EXPECT_NEAR(ts1.to_seconds(), 2.25, tolerance_);
  EXPECT_EQ(ts1.seconds, 2);
  EXPECT_NEAR(ts1.fractional, 0.25, tolerance_);

  // Verify original hasn't changed
  EXPECT_NEAR(original.to_seconds(), 5.75, tolerance_);

  // Test with negative result
  WallTime ts3{1, 0.5};
  ts3 -= ts1;
  EXPECT_NEAR(ts3.to_seconds(), -0.75, tolerance_);
}

TEST_F(WallTimeTest, ArithmeticOperatorsReturnValue) {
  WallTime ts1{5, 0.75};
  WallTime ts2{3, 0.5};

  // Test that operators return proper values
  WallTime& ref1 = (ts1 += ts2);
  EXPECT_TRUE(&ref1 == &ts1);  // += returns reference to this

  WallTime ts3{2, 0.25};
  WallTime& ref2 = (ts3 -= ts2);
  EXPECT_TRUE(&ref2 == &ts3);  // -= returns reference to this
}

}  // namespace svdsim_server
