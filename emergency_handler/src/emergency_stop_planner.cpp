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
#include <utility>
#include <algorithm>
#include <emergency_handler/emergency_stop_planner.h>

SemiEmergencyStopPlanner::SemiEmergencyStopPlanner(const std::pair<std::string, int> param, double max_dec) :
EmergencyPlanner(param, max_dec)
{
}

void SemiEmergencyStopPlanner::get_feedback_from_emergency_planner(EmergencyPlannerFeedback* const epf)
{
  epf->is_vehicle_cmd_updated = true;
  epf->vehicle_cmd.header.stamp = ros::Time::now();
  epf->vehicle_cmd.ctrl_cmd.linear_velocity =
  std::max(epf->vehicle_cmd.ctrl_cmd.linear_velocity - max_dec_for_spdctl_, 0.0);
  epf->vehicle_cmd.emergency = (epf->vehicle_cmd.ctrl_cmd.linear_velocity == 0.0) ? 1 : 0;
}

EmergencyStopPlanner::EmergencyStopPlanner(const std::pair<std::string, int> param, double max_dec) :
EmergencyPlanner(param, max_dec)
{
}

void EmergencyStopPlanner::get_feedback_from_emergency_planner(EmergencyPlannerFeedback* const epf)
{
  epf->is_vehicle_cmd_updated = true;
  epf->vehicle_cmd.header.stamp = ros::Time::now();
  epf->vehicle_cmd.emergency = 1;
  epf->vehicle_cmd.ctrl_cmd.linear_velocity = 0;
  //  epf->vehicle_cmd.ctrl_cmd.steering_angle // No update use the previous steering angle
}
