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
#include <vector>
#include <map>
#include <emergency_handler/libvital_monitor.h>

const std::map<std::string, int>& VitalMonitor::getDeadNodes()
{
  return dead_nodes_;
}

void VitalMonitor::updateNodeStatus(const std::vector<std::string>& available_nodes)
{
  const ros::Time current = ros::Time::now();
  static ros::Time previous = current;
  const double diff = (current - previous).toSec();
  previous = current;

  for (const auto& node : available_nodes)
  {
    if (dead_nodes_.count(node) != 0)
    {
      ROS_INFO("%s is switched to be available", node.c_str());
      required_nodes_.at(node).reset();
      dead_nodes_.erase(node);
    }
  }

  for (auto& node : required_nodes_)
  {
    const std::string node_name = node.first;
    const auto& found = std::find(available_nodes.begin(), available_nodes.end(), node_name);
    if (found == available_nodes.end())
    {
      node.second.spend(diff);
    }
    else
    {
      node.second.activate();
    }
  }

  for (const auto& node : required_nodes_)
  {
    const std::string node_name = node.first;
    if (dead_nodes_.count(node_name) == 0 && node.second.isDead())
    {
      ROS_INFO("%s is not available", node_name.c_str());
      dead_nodes_.emplace(node_name, node.second.level_);
    }
  }
}

void VitalMonitor::initMonitoredNodeList(const ros::NodeHandle& pnh)
{
  XmlRpc::XmlRpcValue params;
  pnh.getParam("vital_monitor", params);
  for (const auto& param : params)
  {
    std::string node_name = "/" + param.first;
    auto val = param.second;
    const double timeout_sec = val.hasMember("timeout") ? static_cast<double>(val["timeout"]) : 0.1;
    const int level = val.hasMember("level") ? static_cast<int>(val["level"]) : 1;
    required_nodes_.emplace(node_name, LifeTime(timeout_sec, level));
  }
  dead_nodes_.clear();
}

autoware_system_msgs::DiagnosticStatusArray VitalMonitor::createDiagnosticStatusArray(std::string dead_node_name,
                                                                                      std_msgs::Header* const header,
                                                                                      int level) const
{
  autoware_system_msgs::DiagnosticStatus ds;
  std::string name(dead_node_name);
  if (name[0] == '/')
  {
    name.erase(name.begin());
  }
  ds.header = *header;
  ds.key = "node_" + name + "_dead";
  ds.description = "node " + name + " is dead";
  ds.type = autoware_system_msgs::DiagnosticStatus::INTERNAL;
  ds.level = level;
  autoware_system_msgs::DiagnosticStatusArray array;
  array.status.emplace_back(ds);
  return array;
}

autoware_system_msgs::NodeStatus VitalMonitor::createNodeStatus(std::string dead_node_name,
                                                                std_msgs::Header* const header, int level) const
{
  autoware_system_msgs::NodeStatus ns;
  ns.header = *header;
  ns.node_name = dead_node_name;
  ns.node_activated = true;
  ns.status.emplace_back(createDiagnosticStatusArray(dead_node_name, header, level));
  return ns;
}

void VitalMonitor::addDeadNodes(std::shared_ptr<autoware_system_msgs::SystemStatus> const status) const
{
  for (auto& node : dead_nodes_)
  {
    const std::string name = node.first;
    const int level = node.second;
    auto& array = status->node_status;
    auto found = std::find_if(array.begin(), array.end(),
    [&](autoware_system_msgs::NodeStatus& stat) {return (name == stat.node_name);});  // NOLINT
    if (found == array.end())
    {
      array.emplace_back(createNodeStatus(name, &status->header, level));
    }
    else
    {
      found->node_activated = true;
      found->status.emplace_back(createDiagnosticStatusArray(name, &status->header, level));
    }
  }
}
