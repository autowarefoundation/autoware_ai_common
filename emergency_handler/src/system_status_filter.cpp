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
#include <emergency_handler/emergency_handler.h>
#include <emergency_handler/system_status_filter.h>

static constexpr int DIAG_ERROR = autoware_system_msgs::DiagnosticStatus::ERROR;

int SimpleHardwareFilter::selectPriority(std::shared_ptr<SystemStatus> const status)
{
  const bool is_hardware_error_detected = !(checkAllHardwareSimply(status->hardware_status, DIAG_ERROR));

  return is_hardware_error_detected ?
  EmergencyHandler::priority_table.hardware_error : EmergencyHandler::priority_table.no_error;
}

int SimpleNodeFilter::selectPriority(std::shared_ptr<SystemStatus> const status)
{
  vital_monitor_.updateNodeStatus(status->available_nodes);
  vital_monitor_.addDeadNodes(status);
  const bool is_node_error_detected = !(checkAllNodeSimply(status->node_status, DIAG_ERROR));

  return is_node_error_detected ?
  EmergencyHandler::priority_table.node_error : EmergencyHandler::priority_table.no_error;
}
