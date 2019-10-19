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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SimpleHardwareFilter hw_filter;
  SimpleNodeFilter node_filter;
  EmergencyHandler emergency_handler(nh, pnh);
  SystemStatusFilter::vital_monitor_.initMonitoredNodeList(pnh);
  emergency_handler.registerEmergencyPlanners();
  emergency_handler.addFilter(static_cast<SystemStatusFilter>(hw_filter));
  emergency_handler.addFilter(static_cast<SystemStatusFilter>(node_filter));
  emergency_handler.run();

  return 0;
}
