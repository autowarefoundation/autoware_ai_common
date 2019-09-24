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

#include <autoware_health_checker/health_checker/health_checker.h>

namespace autoware_health_checker {
HealthChecker::HealthChecker(ros::NodeHandle nh,
                                         ros::NodeHandle pnh) {
  node_activated_ = false;
  ros_ok_ = true;
  nh_ = nh;
  pnh_ = pnh;
  status_pub_ =
      nh_.advertise<autoware_system_msgs::NodeStatus>("node_status", 10);
}

HealthChecker::~HealthChecker() {
  ros_ok_ = false;
  publish_thread_.join();
}

void HealthChecker::publishStatus() {
  ros::Rate rate = ros::Rate(autoware_health_checker::UPDATE_RATE);
  while (ros_ok_) {
    mtx_.lock();
    autoware_system_msgs::NodeStatus status;
    status.node_activated = node_activated_;
    ros::Time now = ros::Time::now();
    status.header.stamp = now;
    status.node_name = ros::this_node::getName();
    std::vector<std::string> checker_keys = getRateCheckerKeys();
    // iterate Rate checker and publish rate_check result
    for (auto key_itr = checker_keys.begin(); key_itr != checker_keys.end();
         key_itr++) {
      autoware_system_msgs::DiagnosticStatusArray diag_array;
      autoware_system_msgs::DiagnosticStatus diag;
      diag.type = autoware_system_msgs::DiagnosticStatus::UNEXPECTED_RATE;
      std::pair<uint8_t, double> result =
          rate_checkers_[*key_itr]->getErrorLevelAndRate();
      diag.level = result.first;
      diag.key = *key_itr;
      diag.value = doubeToJson(result.second);
      diag.description = rate_checkers_[*key_itr]->description;
      diag.header.stamp = now;
      diag_array.status.push_back(diag);
      status.status.push_back(diag_array);
    }
    // iterate Diagnostic Buffer and publish all diagnostic data
    std::vector<std::string> keys = getKeys();
    for (auto key_itr = keys.begin(); key_itr != keys.end(); key_itr++) {
      status.status.push_back(diag_buffers_[*key_itr]->getAndClearData());
    }
    status_pub_.publish(status);
    mtx_.unlock();
    rate.sleep();
  }
  return;
}

void HealthChecker::ENABLE() {
  publish_thread_ = boost::thread(boost::bind(&HealthChecker::publishStatus, this));
  return;
}

std::vector<std::string> HealthChecker::getKeys() {
  std::vector<std::string> keys;
  std::vector<std::string> checker_keys = getRateCheckerKeys();
  for (auto buf_itr = diag_buffers_.begin();
       buf_itr != diag_buffers_.end(); buf_itr++) {
    bool matched = false;
    for (auto checker_key_itr = checker_keys.begin();
         checker_key_itr != checker_keys.end(); checker_key_itr++) {
      if (*checker_key_itr == buf_itr->first) {
        matched = true;
      }
    }
    if (!matched) {
      keys.push_back(buf_itr->first);
    }
  }
  return keys;
}

std::vector<std::string> HealthChecker::getRateCheckerKeys() {
  std::vector<std::string> keys;
  for (auto itr = rate_checkers_.begin(); itr != rate_checkers_.end(); itr++) {
    keys.push_back(itr->first);
  }
  return keys;
}

bool HealthChecker::keyExist(std::string key) {
  if (diag_buffers_.count(key) == 0) {
    return false;
  }
  return true;
}

// add New Diagnostic Buffer if the key does not exist
bool HealthChecker::addNewBuffer(std::string key, uint8_t type, std::string description) {
  mtx_.lock();
  bool is_success = false;
  if (!keyExist(key)) {
    std::unique_ptr<DiagBuffer> buf_ptr(
      new DiagBuffer(key, type, description, autoware_health_checker::BUFFER_LENGTH));
    diag_buffers_[key] = std::move(buf_ptr);
    is_success = true;
  }
  mtx_.unlock();
  return is_success;
}

uint8_t HealthChecker::CHECK_MIN_VALUE(std::string key, double value,
                                             double warn_value,
                                             double error_value,
                                             double fatal_value,
                                             std::string description) {
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE,
               description);
  autoware_system_msgs::DiagnosticStatus new_status;
  new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
  if (value < fatal_value) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
  } else if (value < error_value) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
  } else if (value < warn_value) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
  } else {
    new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
  }
  new_status.description = description;
  new_status.value = doubeToJson(value);
  new_status.header.stamp = ros::Time::now();
  diag_buffers_[key]->addDiag(new_status);
  return new_status.level;
}

uint8_t HealthChecker::CHECK_MAX_VALUE(std::string key, double value,
                                             double warn_value,
                                             double error_value,
                                             double fatal_value,
                                             std::string description) {
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE,
               description);
  autoware_system_msgs::DiagnosticStatus new_status;
  new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
  if (value > fatal_value) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
  } else if (value > error_value) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
  } else if (value > warn_value) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
  } else {
    new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
  }
  new_status.description = description;
  new_status.value = doubeToJson(value);
  new_status.header.stamp = ros::Time::now();
  diag_buffers_[key]->addDiag(new_status);
  return new_status.level;
}

uint8_t HealthChecker::CHECK_RANGE(std::string key, double value,
                                         std::pair<double, double> warn_value,
                                         std::pair<double, double> error_value,
                                         std::pair<double, double> fatal_value,
                                         std::string description) {
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE,
               description);
  autoware_system_msgs::DiagnosticStatus new_status;
  new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
  if (value < fatal_value.first || value > fatal_value.second) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
  } else if (value < error_value.first || value > error_value.second) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
  } else if (value < warn_value.first || value > warn_value.second) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
  } else {
    new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
  }
  new_status.value = doubeToJson(value);
  new_status.description = description;
  new_status.header.stamp = ros::Time::now();
  diag_buffers_[key]->addDiag(new_status);
  return new_status.level;
}

void HealthChecker::CHECK_RATE(std::string key, double warn_rate,
                                     double error_rate, double fatal_rate,
                                     std::string description) {
  if (!keyExist(key)) {
    std::unique_ptr<RateChecker> checker_ptr(
      new RateChecker(autoware_health_checker::BUFFER_LENGTH, warn_rate,
        error_rate, fatal_rate, description));
    rate_checkers_[key] = std::move(checker_ptr);
  }
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::UNEXPECTED_RATE,
               description);
  rate_checkers_[key]->check();
  return;
}

std::string HealthChecker::doubeToJson(double value) {
  using namespace boost::property_tree;
  std::stringstream ss;
  ptree pt;
  pt.put("value.double", value);
  write_json(ss, pt);
  return ss.str();
}
}
