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
#include <algorithm>
#include <emergency_handler/libsystem_status_filter.h>

void FactorStatusArray::add(const DiagnosticStatus& status)
{
  if (keyset_.count(status.key) != 0)
  {
    auto const& found = std::find_if(dataset_.status.begin(), dataset_.status.end(),
      [&](const DiagnosticStatus& st){ return (st.key == status.key);});  // NOLINT
    if (found != dataset_.status.end())
    {
      *found = status;
      return;
    }
  }
  keyset_.emplace(status.key);
  dataset_.status.emplace_back(status);
}

void FactorStatusArray::reset()
{
  keyset_.clear();
  dataset_.status.clear();
}

SystemStatusFilter::SystemStatusFilter() :
  callback_(std::bind(&SystemStatusFilter::selectPriority, this, std::placeholders::_1)) {}

int SystemStatusFilter::selectPriority(std::shared_ptr<SystemStatus> const status)
{
  return normal_behavior_;
}

const DiagnosticStatusArray& SystemStatusFilter::getFactorStatusArray()
{
  return factor_status_array_.dataset_;
}

void SystemStatusFilter::resetFactorStatusArray()
{
  std::lock_guard<std::mutex> lock(factor_status_mutex_);
  factor_status_array_.reset();
}

const std::function<int(std::shared_ptr<SystemStatus> const)>& SystemStatusFilter::getFunc() const
{
  return callback_;
}

VitalMonitor SystemStatusFilter::vital_monitor_;
FactorStatusArray SystemStatusFilter::factor_status_array_;
std::mutex SystemStatusFilter::factor_status_mutex_;

StatusType SystemStatusFilter::calcStatus(const DiagnosticStatus& status, int level_th)
{
  if (status.level >= level_th)
  {
    std::lock_guard<std::mutex> lock(factor_status_mutex_);
    factor_status_array_.add(status);
    return StatusType::ERROR;
  }
  return StatusType::OK;
}

StatusType SystemStatusFilter::calcStatus(const DiagnosticStatusArray& st_array, int level_th)
{
  StatusType type = StatusType::OK;
  for (const auto& status : st_array.status)
  {
    type = std::max(calcStatus(status, level_th), type);
  }
  return type;
}

StatusType SystemStatusFilter::calcStatus(const NodeStatus& node_status, int level_th)
{
  if (!node_status.node_activated)
  {
    return StatusType::NOT_READY;
  }
  StatusType type = StatusType::OK;
  for (const auto& st_array : node_status.status)
  {
    type = std::max(calcStatus(st_array, level_th), type);
  }
  return type;
}

StatusType SystemStatusFilter::calcStatus(const HardwareStatus& hw_status, int level_th)
{
  StatusType type = StatusType::OK;
  for (const auto& st_array : hw_status.status)
  {
    type = std::max(calcStatus(st_array, level_th), type);
  }
  return type;
}

StatusType SystemStatusFilter::calcStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th)
{
  const auto found = find_if(array.begin(), array.end(),
    [=](const NodeStatus& s) {return s.node_name == node_name;});  // NOLINT
  if (found != array.end())
  {
    return calcStatus(*found, level_th);
  }
  return StatusType::NONE;
}

bool SystemStatusFilter::checkAllNodeSimply(const std::vector<NodeStatus>& array, int level_th)
{
  return checkAllSimply<NodeStatus>(array, level_th);
}

bool SystemStatusFilter::checkAllHardwareSimply(const std::vector<HardwareStatus>& array, int level_th)
{
  return checkAllSimply<HardwareStatus>(array, level_th);
}

template<typename T> bool SystemStatusFilter::checkAllSimply(const std::vector<T>& array, int level_th)
{
  bool is_ok = true;
  for (const auto& st : array)
  {
    const StatusType status = calcStatus(st, level_th);
    is_ok = (status == StatusType::ERROR) ? false : is_ok;
  }
  return is_ok;
}
