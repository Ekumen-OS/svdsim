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
#include <sstream>

#include "svdsim_server/discrete_time.hpp"

namespace svdsim_server {

class DiscreteTimeTest : public ::testing::Test {};

TEST_F(DiscreteTimeTest, DefaultConstruction) {
  DiscreteTime ts;
  EXPECT_EQ(ts.value(), 0);
}

TEST_F(DiscreteTimeTest, ValueConstruction) {
  DiscreteTime ts(42);
  EXPECT_EQ(ts.value(), 42);
}

TEST_F(DiscreteTimeTest, ImplicitConversionFromSizeT) {
  DiscreteTime ts = 100;
  EXPECT_EQ(ts.value(), 100);
}

TEST_F(DiscreteTimeTest, EqualityComparison) {
  DiscreteTime ts1(10);
  DiscreteTime ts2(10);
  DiscreteTime ts3(20);

  EXPECT_TRUE(ts1 == ts2);
  EXPECT_FALSE(ts1 == ts3);
  EXPECT_TRUE(ts1 != ts3);
  EXPECT_FALSE(ts1 != ts2);
}

TEST_F(DiscreteTimeTest, RelationalComparison) {
  DiscreteTime ts1(10);
  DiscreteTime ts2(20);
  DiscreteTime ts3(10);

  // Less than
  EXPECT_TRUE(ts1 < ts2);
  EXPECT_FALSE(ts2 < ts1);
  EXPECT_FALSE(ts1 < ts3);

  // Less than or equal
  EXPECT_TRUE(ts1 <= ts2);
  EXPECT_TRUE(ts1 <= ts3);
  EXPECT_FALSE(ts2 <= ts1);

  // Greater than
  EXPECT_TRUE(ts2 > ts1);
  EXPECT_FALSE(ts1 > ts2);
  EXPECT_FALSE(ts1 > ts3);

  // Greater than or equal
  EXPECT_TRUE(ts2 >= ts1);
  EXPECT_TRUE(ts1 >= ts3);
  EXPECT_FALSE(ts1 >= ts2);
}

TEST_F(DiscreteTimeTest, PreIncrement) {
  DiscreteTime ts(10);
  DiscreteTime& result = ++ts;
  EXPECT_EQ(ts.value(), 11);
  EXPECT_EQ(result.value(), 11);
  EXPECT_EQ(&result, &ts);  // Returns reference to self
}

TEST_F(DiscreteTimeTest, PostIncrement) {
  DiscreteTime ts(10);
  DiscreteTime result = ts++;
  EXPECT_EQ(ts.value(), 11);
  EXPECT_EQ(result.value(), 10);  // Returns old value
}

TEST_F(DiscreteTimeTest, PreDecrement) {
  DiscreteTime ts(10);
  DiscreteTime& result = --ts;
  EXPECT_EQ(ts.value(), 9);
  EXPECT_EQ(result.value(), 9);
  EXPECT_EQ(&result, &ts);  // Returns reference to self
}

TEST_F(DiscreteTimeTest, PostDecrement) {
  DiscreteTime ts(10);
  DiscreteTime result = ts--;
  EXPECT_EQ(ts.value(), 9);
  EXPECT_EQ(result.value(), 10);  // Returns old value
}

TEST_F(DiscreteTimeTest, Addition) {
  DiscreteTime ts(10);
  DiscreteTime result = ts + 5;
  EXPECT_EQ(result.value(), 15);
  EXPECT_EQ(ts.value(), 10);  // Original unchanged
}

TEST_F(DiscreteTimeTest, Subtraction) {
  DiscreteTime ts(10);
  DiscreteTime result = ts - 3;
  EXPECT_EQ(result.value(), 7);
  EXPECT_EQ(ts.value(), 10);  // Original unchanged
}

TEST_F(DiscreteTimeTest, SubtractionBetweenTimes) {
  DiscreteTime ts1(10);
  DiscreteTime ts2(3);
  std::size_t difference = ts1 - ts2;
  EXPECT_EQ(difference, 7);
  EXPECT_EQ(ts1.value(), 10);  // Original unchanged
  EXPECT_EQ(ts2.value(), 3);   // Original unchanged
}

TEST_F(DiscreteTimeTest, CompoundAddition) {
  DiscreteTime ts(10);
  ts += 5;
  EXPECT_EQ(ts.value(), 15);
}

TEST_F(DiscreteTimeTest, CompoundSubtraction) {
  DiscreteTime ts(10);
  ts -= 3;
  EXPECT_EQ(ts.value(), 7);
}

TEST_F(DiscreteTimeTest, StreamOutput) {
  DiscreteTime ts(42);
  std::ostringstream oss;
  oss << ts;
  EXPECT_EQ(oss.str(), "42");
}

TEST_F(DiscreteTimeTest, ChainedOperations) {
  DiscreteTime ts(10);
  ts += 5;
  ts -= 3;
  EXPECT_EQ(ts.value(), 12);
}

TEST_F(DiscreteTimeTest, ConstexprConstruction) {
  constexpr DiscreteTime ts1;
  constexpr DiscreteTime ts2(42);
  static_assert(ts1.value() == 0, "Default construction should be zero");
  static_assert(ts2.value() == 42, "Value construction should work");
}

TEST_F(DiscreteTimeTest, ConstexprOperations) {
  constexpr DiscreteTime ts(10);
  constexpr DiscreteTime sum = ts + 5;
  constexpr DiscreteTime diff = ts - 5;
  constexpr std::size_t time_diff = ts - DiscreteTime(5);

  static_assert(sum.value() == 15, "Addition should work");
  static_assert(diff.value() == 5, "Subtraction should work");
  static_assert(time_diff == 5, "Time difference should work");
}

TEST_F(DiscreteTimeTest, ConstexprComparisons) {
  constexpr DiscreteTime ts1(10);
  constexpr DiscreteTime ts2(20);

  static_assert(ts1 == ts1, "Equality should work");
  static_assert(ts1 != ts2, "Inequality should work");
  static_assert(ts1 < ts2, "Less than should work");
  static_assert(ts1 <= ts2, "Less than or equal should work");
  static_assert(ts2 > ts1, "Greater than should work");
  static_assert(ts2 >= ts1, "Greater than or equal should work");
}

TEST_F(DiscreteTimeTest, ZeroValue) {
  DiscreteTime ts(0);
  EXPECT_EQ(ts.value(), 0);
  EXPECT_TRUE(ts == DiscreteTime(0));
}

TEST_F(DiscreteTimeTest, LargeValue) {
  constexpr std::size_t large = 1000000000UL;
  DiscreteTime ts(large);
  EXPECT_EQ(ts.value(), large);
}

TEST_F(DiscreteTimeTest, ToStringConversion) {
  DiscreteTime ts(42);
  std::string str = std::to_string(ts);
  EXPECT_EQ(str, "42");
}

TEST_F(DiscreteTimeTest, ToStringZeroValue) {
  DiscreteTime ts(0);
  std::string str = std::to_string(ts);
  EXPECT_EQ(str, "0");
}

TEST_F(DiscreteTimeTest, ToStringLargeValue) {
  DiscreteTime ts(123456789);
  std::string str = std::to_string(ts);
  EXPECT_EQ(str, "123456789");
}

TEST_F(DiscreteTimeTest, ToStringUsableWithStdToString) {
  DiscreteTime ts(999);
  std::string str = std::to_string(ts);  // ADL finds svdsim_server::std::to_string
  EXPECT_EQ(str, "999");
}

}  // namespace svdsim_server
