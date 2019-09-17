/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include "test_libwaypoint_follower.hpp"

class LibWaypointFollowerTestSuite : public ::testing::Test {
public:
  LibWaypointFollowerTestSuite() {}
  ~LibWaypointFollowerTestSuite() {}
  LibWaypointFollowerTestClass test_obj_;

protected:
  virtual void SetUp(){}
  virtual void TearDown(){}
};

TEST_F(LibWaypointFollowerTestSuite, calcRelativeCoordinate) {
//  The member variable x of calcRelativeCoordinate must return following:
//  - if      target_point is in front of current_pose,  return positive value
//  - else if target_point is behind from current_pose,  return negative value
//  - else    (target_point == current_pose),            return 0
  auto point = [](double x, double y, double z)
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return std::move(p);
  };
  const geometry_msgs::PoseStamped cpos = test_obj_.generateCurrentPose(0, 0, 0);
  std::map<std::string, std::pair<geometry_msgs::Point, CoordinateResult>> dataset;
  dataset["(is_forward)"] = std::make_pair(point(1, 0, 0), CoordinateResult::Positive);
  dataset["(is_backward)"] = std::make_pair(point(-1, 0, 0), CoordinateResult::Negative);
  dataset["(is_equal)"] = std::make_pair(point(0, 0, 0), CoordinateResult::Equal);
  for (const auto& el : dataset)
  {
    const double x = calcRelativeCoordinate(el.second.first, cpos.pose).x;
    const CoordinateResult ret =
      (x < 0.0) ? CoordinateResult::Negative
      : (x > 0.0) ? CoordinateResult::Positive : CoordinateResult::Equal;
    ASSERT_EQ(ret, el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, getLaneDirectionByPosition) {
//  getLaneDirectionByPosition must return following:
//  - if      waypoint[i+1] is in front of waypoint[i],  return Forward
//  - else if waypoint[i+1] is behind from waypoint[i],  return Backward
//  - else if waypoint[i+1] == waypoint[i] in all i,     return Error
//  - else if waypoint size < 2,                         return Error
  std::map<std::string, std::pair<autoware_msgs::Lane, LaneDirection>> dataset;
  dataset["(pos>0)"] = std::make_pair(test_obj_.generateLane(1, 5.0), LaneDirection::Forward);
  dataset["(pos<0)"] = std::make_pair(test_obj_.generateLane(-1, -5.0), LaneDirection::Backward);
  dataset["(pos=0)"] = std::make_pair(test_obj_.generateLane(0, -5.0), LaneDirection::Error);
  dataset["(size<2)"] = std::make_pair(test_obj_.generateOffsetLane(0, -5.0, 0.0, 0), LaneDirection::Error);
  for (const auto& el : dataset)
  {
    ASSERT_EQ(getLaneDirectionByPosition(el.second.first), el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, getLaneDirectionByVelocity) {
//  getLaneDirectionByVelocity must return following:
//  - if waypoint[i+1] is in front of waypoint[i],      return Forward
//  - else if waypoint[i+1] is behind from waypoint[i], return Backward
//  - else (for all i: velocity[i] == 0),               return Error
  std::map<std::string, std::pair<autoware_msgs::Lane, LaneDirection>> dataset;
  dataset["(vel>0)"] = std::make_pair(test_obj_.generateLane(1, 5.0), LaneDirection::Forward);
  dataset["(vel<0)"] = std::make_pair(test_obj_.generateLane(-1, -5.0), LaneDirection::Backward);
  dataset["(vel=0)"] = std::make_pair(test_obj_.generateLane(-1, 0.0), LaneDirection::Error);
  for (const auto& el : dataset)
  {
    ASSERT_EQ(getLaneDirectionByVelocity(el.second.first), el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, getLaneDirection) {
//  getLaneDirection must return following:
//   Now, Rp means position identification result,
//    and Rv means velocity identification result.
//    Rp and Rv return Forward, Backward, or Error.
//  - if Rp != Rv and Rp != Error and Rv != Error,    return Error
//  - else if Rp != Error and Rv != Error,            return Rp(== Rv)
//    => if Rp == Forward and Rv == Forward(v > 0.0), return Forward
//  - else if Rp != Error and Rv == Error,            return Rp
//    => if Rp == Forward and Rv == Error,            return Forward
//  - else if Rp == Error and Rv != Error,            return Rv
//    => if Rp == Error and Rv == Forward(v > 0.0),   return Forward
//  - else (Rp == Error and Rv == Error),             return Rv(== ERROR)
  std::map<std::string, std::pair<autoware_msgs::Lane, LaneDirection>> dataset;
  dataset["(Rp!=Rv)"] = std::make_pair(test_obj_.generateLane(1, -5.0), LaneDirection::Error);
  dataset["(Rp!=Err,Rv!=Err)"] = std::make_pair(test_obj_.generateLane(1, 5.0), LaneDirection::Forward);
  dataset["(Rp!=Err,Rv==Err)"] = std::make_pair(test_obj_.generateLane(1, 0.0), LaneDirection::Forward);
  dataset["(Rp==Err,Rv!=Err)"] = std::make_pair(test_obj_.generateLane(0, 5.0), LaneDirection::Forward);
  dataset["(Rp==Err,Rv==Err)"] = std::make_pair(test_obj_.generateLane(0, 0.0), LaneDirection::Error);
  for (const auto& el : dataset)
  {
    ASSERT_EQ(getLaneDirection(el.second.first), el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, inDrivingDirection) {
//  inDrivingDirection must return following:
//   Now, Rd means lane direction identification result,
//    and Rc means relative Coordinate result.
//    Rd return Forward, Backward, or Error.
//    The menber variable x of Rc return double value.
//
//   case) current_pose == (0,0,0) && lane.x={-1, 0, 1, 2, ...}
//    Now, idx means the target lane index,
//      and vel means lane.velocity(constant value in all point)
//  - if Rd == Error,                   return false
//    =>if idx == Any and vel < 0.0,    return false
// - else if Rc.x > 0.0,                return (Rd == Forward)
//   => if idx > 1 and vel >=0.0,       return true
// - else if Rc.x = 0.0,                return (Rd == Forward)
//   => if idx == 1 and vel >= 0.0,     return true
// - else (Rc.x < 0.0),                 return (Rd == Backward)
//   => if idx < 1 and vel >= 0.0,      return false
  geometry_msgs::PoseStamped pose = test_obj_.generateCurrentPose(0, 0, 0);
  std::map<std::string, std::pair<DirectionCheckDataSet, bool>> dataset;
  dataset["(idx==Any,vel<0.0)"] = std::make_pair(DirectionCheckDataSet(0, -5.0), false);
  dataset["(idx>1,vel>=0.0)"] = std::make_pair(DirectionCheckDataSet(2, 5.0), true);
  dataset["(idx==1,vel>=0.0)"] = std::make_pair(DirectionCheckDataSet(1, 5.0), true);
  dataset["(idx<1,vel>=0.0)"] = std::make_pair(DirectionCheckDataSet(0, 5.0), false);
  for (const auto& el : dataset)
  {
    WayPoints wp;
    wp.setPath(test_obj_.generateOffsetLane(1, el.second.first.vel, -1.0, 100));
    bool ret = wp.inDrivingDirection(el.second.first.idx, pose.pose);
    ASSERT_EQ(ret, el.second.second)
      << "Failure in " + el.first << ", it must be " << el.second.second << ".";
  }
}


TEST_F(LibWaypointFollowerTestSuite, getClosestWaypoint) {
//
// getClosestWaypoint must return following:
//   case) current_pose == (0,0,0)
// - if conflict path input,                                           return -1
// - else if no points path input,                                     return -1
// - else if success to search valid front near points,  return nearest idx(>0)
//   ("valid" means within distance and angle threshold)
//   =>  if valid_forward: lane_x = {-0.5,0.5,1.5,...} and vel >= 0,   return 1
//       if valid backward:lane_x = {1.5,0.5,-0.5,...} and vel < 0,    return 2
// - else if success to search invalid front near points, return nearest idx(>0)
//   =>  if over distance: lane_x = {6,7,8,9,...} and vel >= 0,        return 1
//       if opposite lane:
//         case) current_pose == (0,0,pi/2)
//                         lane_x = {-0.5,0.5,1.5,...} and vel >= 0,   return 1
// - else (fail to search front points),                               return -1
//   =>  if pass endpoint: lane_x = {-100,...,-1} and vel >= 0,        return -1

  geometry_msgs::PoseStamped valid_pose = test_obj_.generateCurrentPose(0, 0, 0);
  geometry_msgs::PoseStamped invalid_pose = test_obj_.generateCurrentPose(0, 0, M_PI / 2.0);
  std::map<std::string, std::pair<ClosestCheckDataSet, int>> dataset;
  dataset["(conflict_path)"] = std::make_pair(ClosestCheckDataSet(1, -5.0, 0.0, 100, valid_pose), -1);
  dataset["(no_point_path)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, 0.0, 0, valid_pose), -1);
  dataset["(valid_forward)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, -0.5, 100, valid_pose), 1);
  dataset["(valid_backward)"] = std::make_pair(ClosestCheckDataSet(-1, -5.0, -1.5, 100, valid_pose), 2);
  dataset["(over_distance)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, 6.0, 100, valid_pose), 1);
  dataset["(opposite_lane)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, -0.5, 100, invalid_pose), 1);
  dataset["(pass_endpoint)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, -100.0, 100, valid_pose), -1);

  for (const auto& el : dataset)
  {
    const ClosestCheckDataSet& data = el.second.first;
    const auto& lane = test_obj_.generateOffsetLane(data.dir, data.vel, data.offset, data.num);
    int ret = getClosestWaypoint(lane, data.pose.pose);
    ASSERT_EQ(ret, el.second.second)
      << "Failure in " << el.first << ", it must be " << el.second.second << ".";
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LibWaypointFollowerTestNode");
  ros::NodeHandle n;
  return RUN_ALL_TESTS();
}
