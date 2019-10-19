The format of the setting file for emergency handler is as follows:

[1.a] Parameters related to emergency handler

```
emergency_spdctl_max_dec: This is maximum deceleration rate applied for speed control at emergency situation.
Note that this is not applied for Emergency stop(braking request with no speed control)

emergency_planner_enabled: This is for enabling/disabling emergency planner.
If emergency planner is set to false, no emergency action is taken with warning only.

emergency_planner: Available emergency planners with priority are listed here.
Emergency stop planner(braking request with no speed control) and semi-emergency stop(emergency stop by means of speed control) are prepared as default. Below is an example. As zero is the highest priority, emergency stop is applied at highest emergency situation, whereas semi-emergency_stop_planner is applied for other emergency situation.

emergency_planner:
  emergency_stop_planner: 0
  semi-emergency_stop_planner: 1

emergency_handling_priority: This determines the priority of each error situation. Below is an example where all errors are considered as the highest priority.

emergency_handling_priority:
  node_error: 0
  hardware_error: 0
  emergency_handler_error: 0
```

[1.b] Nodes to be monitored

Nodes to be monitored are listed with the following format.

vital_monitor:
  node_name:
    timeout: timeout threshold
    level: output level upon detection of no response from the target node

Below is an example. As the list is dependent on application, only decision_maker is listed as default.

```
vital_monitor:
  waypoint_replanner:
    timeout: 0.1
    level: 4
  lane_rule:
    timeout: 0.1
    level: 4
  lane_stop:
    timeout: 0.1
    level: 4
  lane_select:
    timeout: 0.1
    level: 4
  astar_avoid:
    timeout: 0.1
    level: 4
  velocity_set:
    timeout: 0.1
    level: 4
  pure_pursuit:
    timeout: 0.1
    level: 4
  twist_filter:
    timeout: 0.1
    level: 4
  vel_relay:
    timeout: 0.1
    level: 4
  pose_relay:
    timeout: 0.1
    level: 4
  decision_maker:
    timeout: 0.1
    level: 4
  ndt_matching:
    timeout: 0.1
    level: 4
  voxel_grid_filter:
    timeout: 0.1
    level: 4
  ray_ground_filter:
    timeout: 0.1
    level: 4
  diagnostic_aggregator:
    timeout: 0.1
    level: 4
```

[2] you can load these threshold online.
`rosparam load this_file`
