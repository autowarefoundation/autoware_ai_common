# map_file package
## points_map_filter
### feature
points_map_filter_node subscribe pointcloud maps and current pose, the node extract pointcloud near to the current pose.

#### subscribed topics
/points_map (sensor_msgs/PointCloud2)  : Raw pointcloud map. This topic usually comes from points_map_loader.  
/current_pose (geometry_msgs/PoseStamped) : Current pose of the car. This topic usually comes from pose_relay node.  

#### published topics
/points_map/filtered (sensor_msgs/PointCloud2) : Filtered pointcloud submap.  

#### parameters
load_grid_size (double) : grid size of submap.  
load_trigger_distance (double) : if the car moves load_trigger_distance(m), the map filter publish filtered submap. 

### how it works
map_filter_node relay /points_map topic until it recieves /current_pose topic.  
Then, the /current_pose topic recieved, the map_filter_node publish submap.

## demonstration
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/LpKIuI5b4DU/0.jpg)](http://www.youtube.com/watch?v=LpKIuI5b4DU)

## lanelet2_map_loader
### Feature
lanelet2_map_loader loads Lanelet2 file and publish the map data as lanelet2_msgs/MapBin message.
The node projects lan/lon coordinates into MGRS coordinates. 

### How to run
Run from CLI:
`rosrun map_file lanelet2_map_loader path/to/map.osm`

### Published Topic
/lanelet_map_bin (lanelet2_msgs/MapBin) : Binary data of loaded Lanelet2 Map.

## lanelet2_map_visualization
### Feature
lanelet2_map_visualization visualizes lanelet2_msgs/MapBin messages into visualization_msgs/MarkerArray.

### How to run 
Run from CLI:
`rosrun map_file lanelet2_map_visualization`

### Subscribed Topics
/lanelet_map_bin (lanelet2_msg/MapBin) : binary data of Lanelet2 Map

### Published Topics
/lanelet2_map_viz (visualization_msgs/MarkerArray) : visualization messages for RVIZ
