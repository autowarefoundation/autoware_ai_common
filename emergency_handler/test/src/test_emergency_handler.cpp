/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <gtest/gtest.h>
#include <emergency_handler/emergency_handler.h>
#include <emergency_handler/emergency_stop_planner.h>
#include <emergency_handler/system_status_filter.h>
#include "autoware_msgs/VehicleCmd.h"

class MyEmergencyHandler : public EmergencyHandler
{
  friend class EmergencyHandlerTestSuite;

public:
  MyEmergencyHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : EmergencyHandler(nh, pnh) {}
};

class EmergencyHandlerTestSuite : public ::testing::Test
{
protected:
  std::unique_ptr<MyEmergencyHandler> myobj_;

  void SetUp(void)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    myobj_ = std::unique_ptr<MyEmergencyHandler>(new MyEmergencyHandler(nh, pnh));
    SystemStatusFilter::vital_monitor_.initMonitoredNodeList(pnh);
    myobj_->registerEmergencyPlanners();

    vehicle_cmd_pub = nh.advertise<autoware_msgs::VehicleCmd>("vehicle_cmd", 1);
    system_status_pub = nh.advertise<autoware_system_msgs::SystemStatus>("system_status", 1);
  }

public:
  ros::Publisher vehicle_cmd_pub, system_status_pub;
  ros::Subscriber system_status_sub;

  void publish_system_status(int level)
  {
    autoware_system_msgs::SystemStatus status;
    status.node_status.emplace_back(SystemStatusFilter::vital_monitor_.createNodeStatus("Test", &status.header, level));
    system_status_pub.publish(status);
  }

  void publish_vehicle_cmd(int linear_vel, int linear_acc, int steer_angle, int emergency)
  {
    autoware_msgs::VehicleCmd vehicle_cmd;
    vehicle_cmd.ctrl_cmd.linear_velocity = linear_vel;
    vehicle_cmd.ctrl_cmd.linear_acceleration = linear_acc;
    vehicle_cmd.ctrl_cmd.steering_angle = steer_angle;
    vehicle_cmd.emergency = emergency;
    vehicle_cmd_pub.publish(vehicle_cmd);
  }

  void loadParamTest(void)
  {
    ASSERT_TRUE(myobj_->is_emergency_planner_enabled_);
    ASSERT_EQ(myobj_->priority_table.node_error, 0);
    ASSERT_EQ(myobj_->priority_table.hardware_error, 1);
    ASSERT_EQ(myobj_->priority_table.emergency_handler_error, 2);
  }

  void registerEmergencyPlannersTest(void)
  {
    ASSERT_EQ(myobj_->emergency_planner_map_.size(), 2);
  }

  void findTargetEmergencyPlannerTest(void)
  {
    std::shared_ptr<EmergencyPlanner> target = myobj_->find_target_emergency_planner(0);
    ASSERT_STREQ(target->planner_name().c_str(), "emergency_stop_planner");

    target = myobj_->find_target_emergency_planner(1);
    ASSERT_STREQ(target->planner_name().c_str(), "semi-emergency_stop_planner");

    target = myobj_->find_target_emergency_planner(2);
    ASSERT_STREQ(target->planner_name().c_str(), "semi-emergency_stop_planner");
  }

  void updatePriorityTest(void)
  {
    myobj_->priority_ = myobj_->priority_table.no_error;
    myobj_->updatePriority(false);
    ASSERT_EQ(myobj_->priority_, myobj_->priority_table.emergency_handler_error);

    myobj_->priority_ = myobj_->priority_table.hardware_error;
    myobj_->updatePriority(false);
    ASSERT_EQ(myobj_->priority_, myobj_->priority_table.hardware_error);

    myobj_->priority_ = myobj_->priority_table.node_error;
    myobj_->updatePriority(false);
    ASSERT_EQ(myobj_->priority_, myobj_->priority_table.node_error);
  }

  void feedbackTest1(void)
  {
    std::shared_ptr<EmergencyPlanner> target = myobj_->find_target_emergency_planner(myobj_->priority_table.node_error);

    autoware_msgs::ControlCommand cmd;
    cmd.linear_velocity = 2.0;
    cmd.steering_angle = 10.0;

    double expected_spd = cmd.linear_velocity;
    double expected_str = cmd.steering_angle;

    for (int i = 0; i < 200; i++)
    {
      EmergencyPlannerFeedback epf(cmd);
      target->get_feedback_from_emergency_planner(&epf);
      cmd = epf.vehicle_cmd.ctrl_cmd;

      expected_spd = 0;

      ASSERT_EQ(cmd.linear_velocity, expected_spd);
      ASSERT_EQ(cmd.steering_angle, expected_str);
    }
  }

  void feedbackTest2(void)
  {
    std::shared_ptr<EmergencyPlanner> target =
        myobj_->find_target_emergency_planner(myobj_->priority_table.emergency_handler_error);

    autoware_msgs::ControlCommand cmd;
    cmd.linear_velocity = 2.0;
    cmd.steering_angle = 10.0;

    double expected_spd = cmd.linear_velocity;
    double expected_str = cmd.steering_angle;
    double max_dec;

    myobj_->pnh_.param<double>("emergency_spdctl_max_dec", max_dec, 1.0);
    double dec_spd_each = max_dec / EMERGENCY_PLANNER_RATE;

    for (int i = 0; i < 200; i++)
    {
      EmergencyPlannerFeedback epf(cmd);
      target->get_feedback_from_emergency_planner(&epf);
      cmd = epf.vehicle_cmd.ctrl_cmd;

      expected_spd = std::max((expected_spd - dec_spd_each), 0.0);

      ASSERT_EQ(cmd.linear_velocity, expected_spd);
      ASSERT_EQ(cmd.steering_angle, expected_str);
    }
  }

  void feedbackTest3(void)
  {
    std::shared_ptr<EmergencyPlanner> target =
        myobj_->find_target_emergency_planner(myobj_->priority_table.emergency_handler_error);

    autoware_msgs::ControlCommand cmd;
    cmd.linear_velocity = 2.0;
    cmd.steering_angle = 10.0;

    double expected_spd = cmd.linear_velocity;
    double expected_str = cmd.steering_angle;
    double max_dec;

    myobj_->pnh_.param<double>("emergency_spdctl_max_dec", max_dec, 1.0);
    double dec_spd_each = max_dec / EMERGENCY_PLANNER_RATE;

    for (int i = 0; i < 200; i++)
    {
      if (i == 50)
      {
        target = myobj_->find_target_emergency_planner(myobj_->priority_table.node_error);
      }

      EmergencyPlannerFeedback epf(cmd);
      target->get_feedback_from_emergency_planner(&epf);
      cmd = epf.vehicle_cmd.ctrl_cmd;

      if (i < 50)
      {
        expected_spd = std::max((expected_spd - dec_spd_each), 0.0);
      }
      else
      {
        expected_spd = 0;
      }

      ASSERT_EQ(cmd.linear_velocity, expected_spd);
      ASSERT_EQ(cmd.steering_angle, expected_str);
    }
  }

  void vehicleCmdCallback(void)
  {
    int linear_vel = 3.0;
    int linear_acc = 3.0;
    int steering_angle = 3.0;
    int emergency = 1;

    publish_vehicle_cmd(linear_vel, linear_acc, steering_angle, emergency);

    for (int i = 0; i < 3; i++)
    {
      ros::WallDuration(0.1).sleep();
      ros::spinOnce();
    }

    ASSERT_EQ(myobj_->ctrl_cmd_.linear_velocity, 3.0);
    ASSERT_EQ(myobj_->ctrl_cmd_.linear_acceleration, 3.0);
    ASSERT_EQ(myobj_->ctrl_cmd_.steering_angle, 3.0);
  }

  void nodeStatusCallback(void)
  {
    SimpleHardwareFilter hw_filter;
    SimpleNodeFilter node_filter;

    myobj_->addFilter(static_cast<SystemStatusFilter>(hw_filter));
    myobj_->addFilter(static_cast<SystemStatusFilter>(node_filter));

    myobj_->status_sub_.enable();
    ros::Duration wait_for_subscriber(0.5);
    wait_for_subscriber.sleep();

    int level = 3;
    publish_system_status(level);

    for (int i = 0; i < 3; i++)
    {
      ros::WallDuration(0.1).sleep();
      ros::spinOnce();
    }

    ASSERT_EQ(myobj_->priority_, 0.0);
  }
};

TEST_F(EmergencyHandlerTestSuite, LoadParamTest)
{
  loadParamTest();
}

TEST_F(EmergencyHandlerTestSuite, RegisterEmergencyPlannersTest1)
{
  registerEmergencyPlannersTest();
}

TEST_F(EmergencyHandlerTestSuite, FindTargetEmergencyPlannerTest)
{
  findTargetEmergencyPlannerTest();
}

TEST_F(EmergencyHandlerTestSuite, UpdatePriorityTest)
{
  updatePriorityTest();
}

TEST_F(EmergencyHandlerTestSuite, FeedbackTest1)
{
  feedbackTest1(); /* Emergency planner */
}

TEST_F(EmergencyHandlerTestSuite, FeedbackTest2)
{
  feedbackTest2(); /* Semi-emergency planner */
}

TEST_F(EmergencyHandlerTestSuite, FeedbackTest3)
{
  feedbackTest3(); /* Semi-emergency -> emergency planner */
}

TEST_F(EmergencyHandlerTestSuite, VehicleCmdCallback)
{
  vehicleCmdCallback();
}

TEST_F(EmergencyHandlerTestSuite, NodeStatusCallback)
{
  nodeStatusCallback();
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "EmergencyHanderTestNode");

  return RUN_ALL_TESTS();
}
