/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Masaya Kataoka
 */

#include <string>
#include <vector>
#include <autoware_health_checker/health_checker/health_checker.h>

namespace autoware_health_checker
{
HealthChecker::HealthChecker(ros::NodeHandle nh, ros::NodeHandle pnh)
  : value_manager_(nh, pnh)
  , node_activated_(false)
  , nh_(nh)
  , pnh_(pnh)
{
  status_pub_ =
    nh_.advertise<autoware_system_msgs::NodeStatus>("node_status", 10);
}

void HealthChecker::publishStatus(const ros::TimerEvent& event)
{
  std::lock_guard<std::mutex> lock(mtx_);
  autoware_system_msgs::NodeStatus status;
  status.node_activated = node_activated_;
  ros::Time now = ros::Time::now();
  status.header.stamp = now;
  status.node_name = ros::this_node::getName();
  const auto checker_keys = getRateCheckerKeys();
  // iterate Rate checker and publish rate_check result
  for (const auto& key : checker_keys)
  {
    const auto result = rate_checkers_[key]->getErrorLevelAndRate();
    if (result)
    {
      AwDiagStatusArray diag_array;
      AwDiagStatus diag = setValueCommon(
        key, result->second, rate_checkers_.at(key)->description);
      diag.header.stamp = now;
      diag.level = result->first;
      diag.type = AwDiagStatus::UNEXPECTED_RATE;
      diag_array.status.push_back(diag);
      status.status.push_back(diag_array);
    }
  }
  // iterate Diagnostic Buffer and publish all diagnostic data
  const auto keys = getKeys();
  for (const auto& key : keys)
  {
    status.status.push_back(diag_buffers_[key]->getAndClearData());
  }
  status_pub_.publish(status);
}

ErrorLevel HealthChecker::SET_DIAG_STATUS(
  const autoware_system_msgs::DiagnosticStatus& status)
{
  value_manager_.addCandidate(status.key);
  if (value_manager_.isNotFound(status.key))
  {
    return AwDiagStatus::UNDEFINED;
  }
  auto identify = [&status](const ErrorLevel target_level)
  {
    return (status.level == target_level);
  };
  if (!identify(AwDiagStatus::OK) && !identify(AwDiagStatus::WARN) &&
    !identify(AwDiagStatus::ERROR) && !identify(AwDiagStatus::FATAL))
  {
    return AwDiagStatus::UNDEFINED;
  }
  addNewBuffer(status.key, status.type, status.description);
  return status.level;
}

ErrorLevel HealthChecker::CHECK_TRUE(
  const ErrorKey& key, const bool value,
  const ErrorLevel level, const std::string& description)
{
  value_manager_.addCandidate(key);
  if (value_manager_.isNotFound(key))
  {
    return AwDiagStatus::UNDEFINED;
  }
  AwDiagStatus status = setValueCommon(key, value, description);
  status.level = level;
  status.type = status.INVALID_VALUE;
  return SET_DIAG_STATUS(status);
}

void HealthChecker::ENABLE()
{
  ros::Duration duration(1.0 / autoware_health_checker::UPDATE_RATE);
  timer_ = nh_.createTimer(duration, &HealthChecker::publishStatus, this);
}

std::vector<ErrorKey> HealthChecker::getKeys()
{
  std::vector<ErrorKey> keys;
  const auto checked = getRateCheckerKeys();
  for (const auto& buf : diag_buffers_)
  {
    const auto res = std::find(checked.begin(), checked.end(), buf.first);
    if (res == checked.end())
    {
      keys.push_back(buf.first);
    }
  }
  return keys;
}

std::vector<ErrorKey> HealthChecker::getRateCheckerKeys()
{
  std::vector<ErrorKey> keys;
  for (const auto& checker : rate_checkers_)
  {
    keys.push_back(checker.first);
  }
  return keys;
}

bool HealthChecker::keyExist(const ErrorKey& key) const
{
  return (diag_buffers_.count(key) != 0);
}

// add New Diagnostic Buffer if the key does not exist
bool HealthChecker::addNewBuffer(
  const ErrorKey& key, const ErrorType type, const std::string& description)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (keyExist(key))
  {
    return false;
  }
  diag_buffers_[key] = std::make_unique<DiagBuffer>(key, type, description,
    autoware_health_checker::BUFFER_DURATION);
  return true;
}

ErrorLevel HealthChecker::CHECK_MIN_VALUE(const ErrorKey& key,
  const double value, const double warn_value, const double error_value,
  const double fatal_value, const std::string& description)
{
  value_manager_.addCandidate(key);
  if (value_manager_.isNotFound(key))
  {
    return AwDiagStatus::UNDEFINED;
  }
  static const ThreshType thresh_type = "min";
  value_manager_.setDefaultValue(
    key, thresh_type, warn_value, error_value, fatal_value);
  auto identify = [key, value, thresh_type, this](ErrorLevel level)
  {
    return (value < value_manager_.getValue(key, thresh_type, level).get());
  };
  AwDiagStatus new_status = setValueCommon(key, value, description);
  new_status.level = identify(AwDiagStatus::FATAL) ? AwDiagStatus::FATAL :
    identify(AwDiagStatus::ERROR) ? AwDiagStatus::ERROR :
    identify(AwDiagStatus::WARN) ? AwDiagStatus::WARN :
    AwDiagStatus::OK;
  new_status.type = AwDiagStatus::OUT_OF_RANGE;
  return SET_DIAG_STATUS(new_status);
}

ErrorLevel HealthChecker::CHECK_MAX_VALUE(const ErrorKey& key,
  const double value, const double warn_value, const double error_value,
  const double fatal_value, const std::string& description)
{
  value_manager_.addCandidate(key);
  if (value_manager_.isNotFound(key))
  {
    return AwDiagStatus::UNDEFINED;
  }
  static const ThreshType thresh_type = "max";
  value_manager_.setDefaultValue(
    key, thresh_type, warn_value, error_value, fatal_value);
  auto identify = [key, value, thresh_type, this](ErrorLevel level)
  {
    return (value > value_manager_.getValue(key, thresh_type, level).get());
  };
  AwDiagStatus new_status = setValueCommon(key, value, description);
  new_status.level = identify(AwDiagStatus::FATAL) ? AwDiagStatus::FATAL :
    identify(AwDiagStatus::ERROR) ? AwDiagStatus::ERROR :
    identify(AwDiagStatus::WARN) ? AwDiagStatus::WARN :
    AwDiagStatus::OK;
  new_status.type = AwDiagStatus::OUT_OF_RANGE;
  return SET_DIAG_STATUS(new_status);
}

ErrorLevel HealthChecker::CHECK_RANGE(const ErrorKey& key,
  const double value, const MinMax warn_value, const MinMax error_value,
  const MinMax fatal_value, const std::string& description)
{
  value_manager_.addCandidate(key);
  if (value_manager_.isNotFound(key))
  {
    return AwDiagStatus::UNDEFINED;
  }
  value_manager_.setDefaultValue(key, "min", warn_value.first,
    error_value.first, fatal_value.first);
  value_manager_.setDefaultValue(key, "max", warn_value.second,
    error_value.second, fatal_value.second);
  auto identify = [key, value, this](ErrorLevel level)
  {
    return (value < value_manager_.getValue(key, "min", level).get()) ||
      (value > value_manager_.getValue(key, "max", level).get());
  };
  AwDiagStatus new_status = setValueCommon(key, value, description);
  new_status.level = identify(AwDiagStatus::FATAL) ? AwDiagStatus::FATAL :
    identify(AwDiagStatus::ERROR) ? AwDiagStatus::ERROR :
    identify(AwDiagStatus::WARN) ? AwDiagStatus::WARN :
    AwDiagStatus::OK;
  new_status.type = AwDiagStatus::OUT_OF_RANGE;
  return SET_DIAG_STATUS(new_status);
}

void HealthChecker::CHECK_RATE(const ErrorKey& key,
  const double warn_rate, const double error_rate,
  const double fatal_rate, const std::string& description)
{
  value_manager_.addCandidate(key);
  if (value_manager_.isNotFound(key))
  {
    return;
  }
  if (!keyExist(key))
  {
    value_manager_.setDefaultValue(
      key, "rate", warn_rate, error_rate, fatal_rate);
    rate_checkers_[key] = std::make_unique<RateChecker>(
      autoware_health_checker::BUFFER_DURATION,
      warn_rate, error_rate, fatal_rate, description);
  }
  rate_checkers_[key]->setRate(
    value_manager_.getValue(key, "rate", AwDiagStatus::WARN).get(),
    value_manager_.getValue(key, "rate", AwDiagStatus::ERROR).get(),
    value_manager_.getValue(key, "rate", AwDiagStatus::FATAL).get());
  rate_checkers_[key]->check();
  addNewBuffer(key, AwDiagStatus::UNEXPECTED_RATE, description);
  return;
}

template <typename T>
  autoware_system_msgs::DiagnosticStatus HealthChecker::setValueCommon(
    const ErrorKey& key, const T& value, const std::string& desc)
{
  AwDiagStatus new_status;
  new_status.header.stamp = ros::Time::now();
  new_status.key = key;
  new_status.value = valueToJson(value);
  new_status.description = desc;
  return new_status;
}

}  // namespace autoware_health_checker
