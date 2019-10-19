#ifndef EMERGENCY_HANDLER_EMERGENCY_STOP_PLANNER_H
#define EMERGENCY_HANDLER_EMERGENCY_STOP_PLANNER_H

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
#include <emergency_handler/emergency_planner.h>

class SemiEmergencyStopPlanner : public EmergencyPlanner
{
public:
  SemiEmergencyStopPlanner(const std::pair<std::string, int> param, double max_dec);

  void get_feedback_from_emergency_planner(EmergencyPlannerFeedback* const epf) override;
};

class EmergencyStopPlanner : public EmergencyPlanner
{
public:
  EmergencyStopPlanner(const std::pair<std::string, int> param, double max_dec);

  void get_feedback_from_emergency_planner(EmergencyPlannerFeedback* const epf) override;
};

#endif  // EMERGENCY_HANDLER_EMERGENCY_STOP_PLANNER_H
