#ifndef EMERGENCY_HANDLER_EMERGENCY_HANDLER_H
#define EMERGENCY_HANDLER_EMERGENCY_HANDLER_H
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
#include <map>
#include <memory>
#include <boost/thread.hpp>
#include <autoware_health_checker/constants.h>
#include <autoware_health_checker/system_status_subscriber/system_status_subscriber.h>
#include <autoware_system_msgs/SystemStatus.h>
#include <emergency_handler/libsystem_status_filter.h>
#include <emergency_handler/emergency_planner.h>

typedef std::function<int(std::shared_ptr<SystemStatus> const)> FilterFunc;

struct EmergencyHandlingPriority
{
  unsigned int node_error;
  unsigned int hardware_error;
  unsigned int emergency_handler_error;
  unsigned int no_error;

  EmergencyHandlingPriority() : node_error(0), hardware_error(0), emergency_handler_error(0), no_error(INT_MAX) {}
};

class EmergencyHandler
{
public:
  EmergencyHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void registerEmergencyPlanners(void);
  void addFilter(const SystemStatusFilter& filter);
  void run(void);

  static EmergencyHandlingPriority priority_table;

protected:
  ros::NodeHandle nh_, pnh_;

  ros::Publisher statecmd_pub_, recordcmd_pub_, estatus_pub_, emlane_pub_, emvel_pub_;
  void setupPublisher(void);

  autoware_health_checker::SystemStatusSubscriber status_sub_;
  ros::Subscriber vehicle_cmd_sub_;
  void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& vehicle_cmd);

  autoware_system_msgs::DiagnosticStatusArray error_status_;
  autoware_system_msgs::DiagnosticStatusArray genNoStatusMsg(void);

  std::map<int, std::shared_ptr<EmergencyPlanner>> emergency_planner_map_;
  std::shared_ptr<EmergencyPlanner> find_target_emergency_planner(int req_priority);
  bool is_emergency_planner_enabled_;

  std::mutex priority_mutex_;
  std::mutex error_status_mutex_;

  int priority_;
  void updatePriority(const bool status_ok);
  ros::Time callback_time_;

  autoware_msgs::ControlCommand ctrl_cmd_;

  void wrapFunc(FilterFunc func, std::shared_ptr<SystemStatus> const status);

  bool is_system_status_received_;
};

#endif  // EMERGENCY_HANDLER_EMERGENCY_HANDLER_H
