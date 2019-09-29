/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Masaya Kataoka
 */
#include <autoware_health_checker/health_checker/health_checker.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class AutowareHealthCheckerTestClass {
public:
  AutowareHealthCheckerTestClass() : pnh("~") {}
  std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr;
  ros::NodeHandle pnh;
  ros::NodeHandle nh;
  ~AutowareHealthCheckerTestClass(){};
  void init() {
    health_checker_ptr =
        std::make_shared<autoware_health_checker::HealthChecker>(nh, pnh);
  }
};

class AutowareHealthCheckerTestSuite : public ::testing::Test {
public:
  AutowareHealthCheckerTestSuite() {}

  ~AutowareHealthCheckerTestSuite() {}
  AutowareHealthCheckerTestClass test_obj_;

protected:
  virtual void SetUp() { test_obj_.init(); }
  virtual void TearDown() {}
};

uint8_t test_function(double value) {
  if (value == 0.0) {
    return autoware_health_checker::LEVEL_FATAL;
  }
  if (value == 1.0) {
    return autoware_health_checker::LEVEL_ERROR;
  }
  if (value == 2.0) {
    return autoware_health_checker::LEVEL_WARN;
  }
  return autoware_health_checker::LEVEL_OK;
};

boost::property_tree::ptree test_value_json_func(double value) {
  boost::property_tree::ptree tree;
  tree.put("value", value);
  return tree;
};

/*
  test for value check function
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_VALUE) {
  std::function<uint8_t(double value)> check_func = test_function;
  std::function<boost::property_tree::ptree(double value)>
      check_value_json_func = test_value_json_func;
  uint8_t ret_fatal_value =
      test_obj_.health_checker_ptr->CHECK_VALUE(
          "test", 0.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_fatal_value, autoware_health_checker::LEVEL_FATAL)
      << "CHECK_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_FATAL);
  uint8_t ret_error_value =
      test_obj_.health_checker_ptr->CHECK_VALUE(
          "test", 1.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_error_value, autoware_health_checker::LEVEL_ERROR)
      << "CHECK_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_ERROR);
  uint8_t ret_warn_value =
      test_obj_.health_checker_ptr->CHECK_VALUE(
          "test", 2.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_warn_value, autoware_health_checker::LEVEL_WARN)
      << "CHECK_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_WARN);
  uint8_t ret_ok_value =
      test_obj_.health_checker_ptr->CHECK_VALUE(
          "test", -1.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_ok_value, autoware_health_checker::LEVEL_OK)
      << "CHECK_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_OK);
  boost::optional<double> value =
      check_value_json_func(0.0).get_optional<double>("value");
  ASSERT_EQ(value.get(), 0.0)
      << "Failed to get json value."
      << "It should be 0.0";
}

/*
  test for minimum value check function
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_MIN_VALUE) {
  uint8_t ret_fatal_min =
      test_obj_.health_checker_ptr->CHECK_MIN_VALUE(
          "test", 1, 6, 4, 2, "test");
  ASSERT_EQ(ret_fatal_min, autoware_health_checker::LEVEL_FATAL)
      << "CHECK_MIN_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_FATAL);
  uint8_t ret_error_min =
      test_obj_.health_checker_ptr->CHECK_MIN_VALUE(
          "test", 3, 6, 4, 2, "test");
  ASSERT_EQ(ret_error_min, autoware_health_checker::LEVEL_ERROR)
      << "CHECK_MIN_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_ERROR);
  uint8_t ret_warn_min =
      test_obj_.health_checker_ptr->CHECK_MIN_VALUE(
          "test", 5, 6, 4, 2, "test");
  ASSERT_EQ(ret_warn_min, autoware_health_checker::LEVEL_WARN)
      << "CHECK_MIN_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_WARN);
  uint8_t ret_ok_min =
      test_obj_.health_checker_ptr->CHECK_MIN_VALUE(
          "test", 7, 6, 4, 2, "test");
  ASSERT_EQ(ret_ok_min, autoware_health_checker::LEVEL_OK)
      << "CHECK_MIN_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_OK);
}

/*
  test for maximum value check function
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_MAX_VALUE) {
  uint8_t ret_fatal_max =
      test_obj_.health_checker_ptr->CHECK_MAX_VALUE(
          "test", 7, 2, 4, 6, "test");
  ASSERT_EQ(ret_fatal_max, autoware_health_checker::LEVEL_FATAL)
      << "CHECK_MAX_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_FATAL);
  uint8_t ret_error_max =
      test_obj_.health_checker_ptr->CHECK_MAX_VALUE(
          "test", 5, 2, 4, 6, "test");
  ASSERT_EQ(ret_error_max, autoware_health_checker::LEVEL_ERROR)
      << "CHECK_MAX_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_ERROR);
  uint8_t ret_warn_max =
      test_obj_.health_checker_ptr->CHECK_MAX_VALUE(
          "test", 3, 2, 4, 6, "test");
  ASSERT_EQ(ret_warn_max, autoware_health_checker::LEVEL_WARN)
      << "CHECK_MAX_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_WARN);
  uint8_t ret_ok_max =
      test_obj_.health_checker_ptr->CHECK_MAX_VALUE(
          "test", 1, 2, 4, 6, "test");
  ASSERT_EQ(ret_ok_max, autoware_health_checker::LEVEL_OK)
      << "CHECK_MAX_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_OK);
}

/*
  test for range check functions
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_RANGE) {
  uint8_t ret_fatal_range =
      test_obj_.health_checker_ptr->CHECK_RANGE(
          "test", 7.0, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_fatal_range, autoware_health_checker::LEVEL_FATAL)
      << "CHECK_RANGE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_FATAL);
  uint8_t ret_error_range =
      test_obj_.health_checker_ptr->CHECK_RANGE(
          "test", 5.5, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_error_range, autoware_health_checker::LEVEL_ERROR)
      << "CHECK_RANGE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_ERROR);
  uint8_t ret_warn_range =
      test_obj_.health_checker_ptr->CHECK_RANGE(
          "test", 4.5, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_warn_range, autoware_health_checker::LEVEL_WARN)
      << "CHECK_RANGE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_WARN);
  uint8_t ret_ok_range =
      test_obj_.health_checker_ptr->CHECK_RANGE(
          "test", 3.0, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_ok_range, autoware_health_checker::LEVEL_OK)
      << "CHECK_RANGE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_OK);
}

/*
  test for bool check functions
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_TRUE) {
  uint8_t ret_fatal_bool =
      test_obj_.health_checker_ptr->CHECK_TRUE(
          "test", true, autoware_system_msgs::DiagnosticStatus::FATAL, "test");
  ASSERT_EQ(ret_fatal_bool, autoware_health_checker::LEVEL_FATAL)
      << "CHECK_TRUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_FATAL);
  uint8_t ret_error_bool =
      test_obj_.health_checker_ptr->CHECK_TRUE(
          "test", false, autoware_system_msgs::DiagnosticStatus::ERROR, "test");
  ASSERT_EQ(ret_error_bool, autoware_health_checker::LEVEL_ERROR)
      << "CHECK_TRUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_ERROR);
  uint8_t ret_warn_bool =
      test_obj_.health_checker_ptr->CHECK_TRUE(
          "test", true, autoware_system_msgs::DiagnosticStatus::WARN, "test");
  ASSERT_EQ(ret_warn_bool, autoware_health_checker::LEVEL_WARN)
      << "CHECK_TRUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_WARN);
  uint8_t ret_ok_bool =
      test_obj_.health_checker_ptr->CHECK_TRUE(
          "test", false, autoware_system_msgs::DiagnosticStatus::OK, "test");
  ASSERT_EQ(ret_ok_bool, autoware_health_checker::LEVEL_OK)
      << "CHECK_TRUE function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_OK);
}

/*
  test for set diag function
*/
TEST_F(AutowareHealthCheckerTestSuite, SET_DIAG_STATUS) {
  autoware_system_msgs::DiagnosticStatus status;
  status.level = status.FATAL;
  uint8_t ret_diag_fatal =
      test_obj_.health_checker_ptr->SET_DIAG_STATUS(status);
  ASSERT_EQ(ret_diag_fatal, autoware_health_checker::LEVEL_FATAL)
      << "SET_DIAG_STATUS function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_FATAL);
  status.level = status.ERROR;
  uint8_t ret_diag_error =
      test_obj_.health_checker_ptr->SET_DIAG_STATUS(status);
  ASSERT_EQ(ret_diag_error, autoware_health_checker::LEVEL_ERROR)
      << "SET_DIAG_STATUS function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_ERROR);
  status.level = status.WARN;
  uint8_t ret_diag_warn =
      test_obj_.health_checker_ptr->SET_DIAG_STATUS(status);
  ASSERT_EQ(ret_diag_warn, autoware_health_checker::LEVEL_WARN)
      << "SET_DIAG_STATUS function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_WARN);
  status.level = status.OK;
  uint8_t ret_diag_ok =
      test_obj_.health_checker_ptr->SET_DIAG_STATUS(status);
  ASSERT_EQ(ret_diag_ok, autoware_health_checker::LEVEL_OK)
      << "SET_DIAG_STATUS function returns invalid value."
      << "It should be " << static_cast<int>(autoware_health_checker::LEVEL_OK);
}

/*
  test for node status
*/
TEST_F(AutowareHealthCheckerTestSuite, NODE_STATUS) {
  test_obj_.health_checker_ptr->NODE_ACTIVATE();
  uint8_t ret_active =
      test_obj_.health_checker_ptr->getNodeStatus();
  ASSERT_EQ(ret_active, true) << "The value must be true";
  test_obj_.health_checker_ptr->NODE_DEACTIVATE();
  uint8_t ret_inactive =
      test_obj_.health_checker_ptr->getNodeStatus();
  ASSERT_EQ(ret_inactive, false) << "The value must be true";
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "AutowareHealthCheckerTestNode");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
