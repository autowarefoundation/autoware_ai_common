#ifndef EMERGENCY_HANDLER_LIBSYSTEM_STATUS_FILTER_H
#define EMERGENCY_HANDLER_LIBSYSTEM_STATUS_FILTER_H
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
#include <string>
#include <vector>
#include <set>
#include <mutex>
#include <autoware_system_msgs/SystemStatus.h>
#include <emergency_handler/libvital_monitor.h>

using SystemStatus =  autoware_system_msgs::SystemStatus;
using DiagnosticStatusArray =  autoware_system_msgs::DiagnosticStatusArray;
using DiagnosticStatus = autoware_system_msgs::DiagnosticStatus;
using NodeStatus = autoware_system_msgs::NodeStatus;
using HardwareStatus = autoware_system_msgs::HardwareStatus;

enum class StatusType
{
  NONE,
  NOT_READY,
  OK,
  ERROR
};

struct FactorStatusArray
{
  std::set<std::string> keyset_;
  DiagnosticStatusArray dataset_;
  void add(const DiagnosticStatus& status);
  void reset();
};

class SystemStatusFilter
{
public:
  SystemStatusFilter();
  virtual int selectPriority(std::shared_ptr<SystemStatus> const status);
  static const DiagnosticStatusArray& getFactorStatusArray();
  static void resetFactorStatusArray();
  const std::function<int(std::shared_ptr<SystemStatus> const)>& getFunc() const;
  static VitalMonitor vital_monitor_;

protected:
  std::function<int(std::shared_ptr<SystemStatus> const)> callback_;
  static FactorStatusArray factor_status_array_;
  static constexpr int normal_behavior_ = INT_MAX;

  StatusType calcStatus(const DiagnosticStatus& status, int level_th);
  StatusType calcStatus(const DiagnosticStatusArray& st_array, int level_th);
  StatusType calcStatus(const NodeStatus& node_status, int level_th);
  StatusType calcStatus(const HardwareStatus& hw_status, int level_th);
  StatusType calcStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th);

  bool checkAllNodeSimply(const std::vector<NodeStatus>& array, int level_th);
  bool checkAllHardwareSimply(const std::vector<HardwareStatus>& array, int level_th);
  template<typename T> bool checkAllSimply(const std::vector<T>& array, int level_th);

private:
  static std::mutex factor_status_mutex_;
};


#endif  // EMERGENCY_HANDLER_LIBSYSTEM_STATUS_FILTER_H
