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

#include <string>
#include <map>
#include <algorithm>
#include <functional>
#include <utility>
#include <emergency_handler/emergency_handler.h>
#include <emergency_handler/emergency_stop_planner.h>
#include <emergency_handler/system_status_filter.h>

EmergencyHandlingPriority EmergencyHandler::priority_table;

// Constructor
EmergencyHandler::EmergencyHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh), pnh_(pnh), status_sub_(nh, pnh)
{
  pnh_.param<bool>("emergency_planner_enabled", is_emergency_planner_enabled_, false);

  std::map<std::string, int> emergency_handling_priority;
  pnh_.getParam("emergency_handling_priority", emergency_handling_priority);

  for (auto itr = emergency_handling_priority.begin(); itr != emergency_handling_priority.end(); itr++)
  {
    if (itr->first == "node_error")
    {
      priority_table.node_error = itr->second;
    }
    else if (itr->first == "hardware_error")
    {
      priority_table.hardware_error = itr->second;
    }
    else if (itr->first == "emergency_handler_error")
    {
      priority_table.emergency_handler_error = itr->second;
    }
  }

  vehicle_cmd_sub_ = nh_.subscribe("vehicle_cmd", 1, &EmergencyHandler::vehicleCmdCallback, this);
  EmergencyHandler::setupPublisher();

  priority_ = priority_table.no_error;
  callback_time_ = ros::Time::now();
  is_system_status_received_ = false;

  ctrl_cmd_.linear_velocity = 0.0;
  ctrl_cmd_.steering_angle = 0.0;
}

void EmergencyHandler::setupPublisher(void)
{
  statecmd_pub_ = nh_.advertise<std_msgs::String>("state_cmd", 1, true);
  recordcmd_pub_ = nh_.advertise<std_msgs::Header>("record_cmd", 1, true);
  estatus_pub_ = nh_.advertise<autoware_system_msgs::DiagnosticStatusArray>("error_status", 1, false);
  emlane_pub_ = nh_.advertise<autoware_msgs::Lane>("emergency_waypoints", 1, false);
  emvel_pub_ = nh_.advertise<autoware_msgs::VehicleCmd>("emergency_velocity", 1, false);
}

// Regiser EmergencyPlans
void EmergencyHandler::registerEmergencyPlanners(void)
{
  double espdctl_max_dec;
  pnh_.param<double>("emergency_spdctl_max_dec", espdctl_max_dec, 1.0);

  std::map<std::string, int> emergency_planner_param;
  pnh_.getParam("emergency_planner", emergency_planner_param);

  for (auto itr = emergency_planner_param.begin(); itr != emergency_planner_param.end(); itr++)
  {
    if (itr->first == "semi-emergency_stop_planner")
    {
      emergency_planner_map_.emplace(itr->second, std::shared_ptr<EmergencyPlanner>
        (new SemiEmergencyStopPlanner((*itr), espdctl_max_dec)));
    }
    else if (itr->first == "emergency_stop_planner")
    {
      emergency_planner_map_.emplace(itr->second, std::shared_ptr<EmergencyPlanner>
        (new EmergencyStopPlanner((*itr), espdctl_max_dec)));
    }
  }

  // emergency planner with priority=0 is registered if there exsts no emergency planner
  if (emergency_planner_map_.empty())
  {
    emergency_planner_map_.emplace(0, std::shared_ptr<EmergencyPlanner>
      (new EmergencyStopPlanner(std::pair<std::string, int>("emergency_stop_planner", 0), espdctl_max_dec)));
  }
}

// Find Appropriate emergency planner based upon the priority required by system
std::shared_ptr<EmergencyPlanner> EmergencyHandler::find_target_emergency_planner(int req_priority)
{
  std::shared_ptr<EmergencyPlanner> target_emergency_planner(nullptr);

  if (emergency_planner_map_.empty())
  {
    ROS_ERROR("No Emergency Planner Found");
  }
  else
  {
    std::map<int, std::shared_ptr<EmergencyPlanner>, std::greater<int>> available_planner;

    for (auto itr = emergency_planner_map_.begin(); itr != emergency_planner_map_.end(); itr++)
    {
      if (itr->first <= req_priority)
      {
        available_planner.emplace(itr->first, itr->second);
      }
    }

    if (available_planner.empty())
    {
      ROS_ERROR("No Available Planner Found");
      target_emergency_planner = emergency_planner_map_.begin()->second;
    }
    else
    {
      target_emergency_planner = available_planner.begin()->second;
    }
  }

  return target_emergency_planner;
}

void EmergencyHandler::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& vehicle_cmd)
{
  ctrl_cmd_ = vehicle_cmd->ctrl_cmd;
}

// Add Filter
void EmergencyHandler::addFilter(const SystemStatusFilter& filter)
{
  status_sub_.addCallback(boost::bind(&EmergencyHandler::wrapFunc, this, filter.getFunc(), _1));
}

// Run
void EmergencyHandler::run(void)
{
  status_sub_.enable();
  ros::Duration wait_for_subscriber(0.5);  // TBC
  wait_for_subscriber.sleep();

  bool is_urgent = false;
  ros::Time start_time = ros::Time::now();
  autoware_msgs::ControlCommand emergency_ctrl_cmd;

  double loop_rate = autoware_health_checker::SYSTEM_UPDATE_RATE;

  ros::Rate rate(loop_rate);
  while (ros::ok())
  {
    ros::spinOnce();

    double dt = (ros::Time::now() - callback_time_).toSec();
    const bool status_ok = !(is_system_status_received_ && (dt > 0.1));
    updatePriority(status_ok);

    if (!is_urgent)
    {
      if (priority_ != priority_table.no_error)
      {
        start_time = ros::Time::now();
        emergency_ctrl_cmd = ctrl_cmd_;
        is_urgent = true;

        // decision maker
        if (is_emergency_planner_enabled_)
        {
          std_msgs::String str_msg;
          str_msg.data = "emergency";
          statecmd_pub_.publish(str_msg);
        }

        // drive recorder request
        std_msgs::Header header_msg;
        header_msg.stamp = start_time;
        recordcmd_pub_.publish(header_msg);
        {
          std::lock_guard<std::mutex> lock(error_status_mutex_);
          estatus_pub_.publish(error_status_);
          error_status_.status.clear();
        }
      }
    }
    else
    {
      if ((ros::Time::now() - start_time).toSec() >= 10.0)
      {
        priority_ = priority_table.no_error;
        is_urgent = false;
      }
    }

    if (is_urgent && is_emergency_planner_enabled_)
    {
      std::shared_ptr<EmergencyPlanner> target = find_target_emergency_planner(priority_);
      EmergencyPlannerFeedback epf(emergency_ctrl_cmd);
      target->get_feedback_from_emergency_planner(&epf);
      emergency_ctrl_cmd = epf.vehicle_cmd.ctrl_cmd;

      // Publish Emergency Command
      if (epf.is_vehicle_cmd_updated) emvel_pub_.publish(epf.vehicle_cmd);
      if (epf.is_lane_updated) emlane_pub_.publish(epf.lane);
    }
    rate.sleep();
  }
}

// Wrap Function
void EmergencyHandler::wrapFunc(FilterFunc func, std::shared_ptr<SystemStatus> const status)
{
  std::lock_guard<std::mutex> lock(priority_mutex_);
  priority_ = std::min(func(status), priority_);
  callback_time_ = ros::Time::now();
  is_system_status_received_ = true;
}

void EmergencyHandler::updatePriority(const bool status_ok)
{
  autoware_system_msgs::DiagnosticStatusArray err_status;

  std::lock_guard<std::mutex> lock(priority_mutex_);
  if ((!status_ok) && (priority_ > priority_table.emergency_handler_error))
  {
    priority_ = priority_table.emergency_handler_error;
    err_status = genNoStatusMsg();
  }
  else
  {
    err_status = SystemStatusFilter::getFactorStatusArray();
  }

  std::lock_guard<std::mutex> lock_mut(error_status_mutex_);
  error_status_ = err_status;

  SystemStatusFilter::resetFactorStatusArray();
}

autoware_system_msgs::DiagnosticStatusArray EmergencyHandler::genNoStatusMsg(void)
{
  ros::Time current = ros::Time::now();
  autoware_system_msgs::DiagnosticStatus status;
  status.header.stamp = current;
  status.key = "topic_rate_system_status_slow";
  status.value = "{\n    \"value\": \"0\"\n}\n";
  status.description = "topic rate system_status is slow.";
  status.type = autoware_system_msgs::DiagnosticStatus::UNEXPECTED_RATE;
  status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
  autoware_system_msgs::DiagnosticStatusArray status_array;
  status_array.status.emplace_back(status);
  return std::move(status_array);
}
