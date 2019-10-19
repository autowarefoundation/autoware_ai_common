#ifndef EMERGENCY_HANDLER_EMERGENCY_PLANNER_H
#define EMERGENCY_HANDLER_EMERGENCY_PLANNER_H
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
#include <std_msgs/String.h>
#include <string>
#include <utility>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/ControlCommand.h>
#include <autoware_system_msgs/DiagnosticStatusArray.h>

static constexpr double EMERGENCY_PLANNER_RATE = 50.0;

class EmergencyPlannerFeedback
{
public:
  explicit EmergencyPlannerFeedback(autoware_msgs::ControlCommand cmd) :
  is_vehicle_cmd_updated(false), is_lane_updated(false)
  {
    vehicle_cmd.ctrl_cmd = cmd;
  }

  bool is_vehicle_cmd_updated;
  autoware_msgs::VehicleCmd vehicle_cmd;

  bool is_lane_updated;
  autoware_msgs::Lane lane;
};

class EmergencyPlanner
{
public:
  EmergencyPlanner(const std::pair<std::string, int> param, double max_dec) :
  required_priority_(param.second), name_(param.first) { max_dec_for_spdctl_ = max_dec / EMERGENCY_PLANNER_RATE ;}

  virtual ~EmergencyPlanner() {}

  virtual void get_feedback_from_emergency_planner(EmergencyPlannerFeedback* const epf) = 0;
  const std::string planner_name(void) {return name_;}

protected:
  const std::string name_;
  const int required_priority_;
  double max_dec_for_spdctl_;
};

#endif  //  EMERGENCY_HANDLER_EMERGENCY_PLANNER_H
