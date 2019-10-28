#ifndef EMERGENCY_HANDLER_LIBVITAL_MONITOR_H
#define EMERGENCY_HANDLER_LIBVITAL_MONITOR_H
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
#include <map>
#include <algorithm>
#include <autoware_system_msgs/SystemStatus.h>

struct LifeTime
{
  LifeTime(double secs, int level) :
    life_time_(secs), default_life_time_(secs), level_(level), activated_(false) {}

  bool activated_;
  int level_;
  const double default_life_time_;
  double life_time_;
  const bool isDead() const
  {
    return (activated_ && (life_time_ == 0.0));
  }
  void spend(double secs)
  {
    if (activated_) life_time_ = std::max(life_time_ - secs, 0.0);
  }
  void activate(void)
  {
    activated_ = true;
  }
  void reset()
  {
    life_time_ = default_life_time_;
  }
};

class VitalMonitor
{
public:
  void initMonitoredNodeList(const ros::NodeHandle& pnh);
  void updateNodeStatus(const std::vector<std::string>& available_nodes);
  void addDeadNodes(std::shared_ptr<autoware_system_msgs::SystemStatus> const status) const;
  const std::map<std::string, int>& getDeadNodes();
  autoware_system_msgs::DiagnosticStatusArray
    createDiagnosticStatusArray(std::string dead_node_name, std_msgs::Header* const header, int level) const;
  autoware_system_msgs::NodeStatus
    createNodeStatus(std::string dead_node_name, std_msgs::Header* const header, int level) const;

private:
  std::map<std::string, LifeTime> required_nodes_;
  std::map<std::string, int> dead_nodes_;
};

#endif  // EMERGENCY_HANDLER_LIBVITAL_MONITOR_H
