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

#include "libwaypoint_follower/libwaypoint_follower.h"

int WayPoints::getSize() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;
  else
    return current_waypoints_.waypoints.size();
}

double WayPoints::getInterval() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;

  // interval between 2 waypoints
  tf::Vector3 v1(current_waypoints_.waypoints[0].pose.pose.position.x,
                 current_waypoints_.waypoints[0].pose.pose.position.y, 0);

  tf::Vector3 v2(current_waypoints_.waypoints[1].pose.pose.position.x,
                 current_waypoints_.waypoints[1].pose.pose.position.y, 0);
  return tf::tfDistance(v1, v2);
}

geometry_msgs::Point WayPoints::getWaypointPosition(int waypoint) const
{
  geometry_msgs::Point p;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return p;

  p = current_waypoints_.waypoints[waypoint].pose.pose.position;
  return p;
}

geometry_msgs::Quaternion WayPoints::getWaypointOrientation(int waypoint) const
{
  geometry_msgs::Quaternion q;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return q;

  q = current_waypoints_.waypoints[waypoint].pose.pose.orientation;
  return q;
}

geometry_msgs::Pose WayPoints::getWaypointPose(int waypoint) const
{
  geometry_msgs::Pose pose;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return pose;

  pose = current_waypoints_.waypoints[waypoint].pose.pose;
  return pose;
}

double WayPoints::getWaypointVelocityMPS(int waypoint) const
{
  if (waypoint > getSize() - 1 || waypoint < 0)
    return 0;

  return current_waypoints_.waypoints[waypoint].twist.twist.linear.x;
}

bool WayPoints::inDrivingDirection(int waypoint, geometry_msgs::Pose current_pose) const
{
  const LaneDirection dir = getLaneDirection(current_waypoints_);
  double x = calcRelativeCoordinate(current_waypoints_.waypoints[waypoint].pose.pose.position, current_pose).x;
  return (x < 0.0 && dir == LaneDirection::Backward) || (x >= 0.0 && dir == LaneDirection::Forward);
}

double DecelerateVelocity(double distance, double prev_velocity)
{
  double decel_ms = 1.0;  // m/s
  double decel_velocity_ms = sqrt(2 * decel_ms * distance);

  std::cout << "velocity/prev_velocity :" << decel_velocity_ms << "/" << prev_velocity << std::endl;
  if (decel_velocity_ms < prev_velocity)
  {
    return decel_velocity_ms;
  }
  else
  {
    return prev_velocity;
  }
}

// calculation relative coordinate of point from current_pose frame
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);

  return tf_point_msg;
}

// calculation absolute coordinate of point on current_pose frame
geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = inverse * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

// distance between target 1 and target2 in 2-D
double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}

double getRelativeAngle(geometry_msgs::Pose waypoint_pose, geometry_msgs::Pose vehicle_pose)
{
  geometry_msgs::Point relative_p1 = calcRelativeCoordinate(waypoint_pose.position, vehicle_pose);
  geometry_msgs::Point p2;
  p2.x = 1.0;
  geometry_msgs::Point relative_p2 = calcRelativeCoordinate(calcAbsoluteCoordinate(p2, waypoint_pose), vehicle_pose);
  tf::Vector3 relative_waypoint_v(relative_p2.x - relative_p1.x, relative_p2.y - relative_p1.y,
                                  relative_p2.z - relative_p1.z);
  relative_waypoint_v.normalize();
  tf::Vector3 relative_pose_v(1, 0, 0);
  double angle = relative_pose_v.angle(relative_waypoint_v) * 180 / M_PI;
  // ROS_INFO("angle : %lf",angle);

  return angle;
}

LaneDirection getLaneDirection(const autoware_msgs::Lane& current_path)
{
  const LaneDirection pos_ret = getLaneDirectionByPosition(current_path);
  const LaneDirection vel_ret = getLaneDirectionByVelocity(current_path);
  const bool is_conflict =
    (pos_ret != vel_ret) && (pos_ret != LaneDirection::Error) && (vel_ret != LaneDirection::Error);
  return is_conflict ? LaneDirection::Error : (pos_ret != LaneDirection::Error) ? pos_ret : vel_ret;
}

LaneDirection getLaneDirectionByPosition(const autoware_msgs::Lane& current_path)
{
  if (current_path.waypoints.size() < 2)
  {
    return LaneDirection::Error;
  }
  LaneDirection positional_direction = LaneDirection::Error;
  for (int i = 1; i < current_path.waypoints.size(); i++)
  {
    const geometry_msgs::Pose& prev_pose = current_path.waypoints[i - 1].pose.pose;
    const geometry_msgs::Pose& next_pose = current_path.waypoints[i].pose.pose;
    const double rlt_x = calcRelativeCoordinate(next_pose.position, prev_pose).x;
    if (fabs(rlt_x) < 1e-3)
    {
      continue;
    }
    positional_direction = (rlt_x < 0) ? LaneDirection::Backward : LaneDirection::Forward;
    break;
  }
  return positional_direction;
}

LaneDirection getLaneDirectionByVelocity(const autoware_msgs::Lane& current_path)
{
  LaneDirection velocity_direction = LaneDirection::Error;
  for (const auto waypoint : current_path.waypoints)
  {
    const double& vel = waypoint.twist.twist.linear.x;
    if (fabs(vel) < 0.01)
    {
      continue;
    }
    velocity_direction = (vel < 0) ? LaneDirection::Backward : LaneDirection::Forward;
    break;
  }
  return velocity_direction;
}

class MinIDSearch
{
private:
  double val_min_;
  int idx_min_;
public:
  MinIDSearch() : idx_min_(-1), val_min_(DBL_MAX){}
  void update(int index, double v)
  {
    if (v < val_min_)
    {
      idx_min_ = index;
      val_min_ = v;
    }
  }
  const double result() const
  {
    return idx_min_;
  }
  const bool isOK() const
  {
    return (idx_min_ != -1);
  }
};

// get closest waypoint from current pose
int getClosestWaypoint(const autoware_msgs::Lane &current_path, geometry_msgs::Pose current_pose)
{
  if (current_path.waypoints.size() < 2 || getLaneDirection(current_path) == LaneDirection::Error)
    return -1;

  WayPoints wp;
  wp.setPath(current_path);

  // search closest candidate within a certain meter
  double search_distance = 5.0;
  double angle_threshold = 90;
  MinIDSearch cand_idx, not_cand_idx;
  for (int i = 1; i < wp.getSize(); i++)
  {
    if (!wp.inDrivingDirection(i, current_pose))
      continue;
    double distance = getPlaneDistance(wp.getWaypointPosition(i), current_pose.position);
    not_cand_idx.update(i, distance);
    if (distance > search_distance)
      continue;
    if (getRelativeAngle(wp.getWaypointPose(i), current_pose) > angle_threshold)
      continue;
    cand_idx.update(i, distance);
  }
  if (!cand_idx.isOK())
  {
    ROS_INFO("no candidate. search closest waypoint from all waypoints...");
  }
  return (!cand_idx.isOK()) ? not_cand_idx.result() : cand_idx.result();
}

// let the linear equation be "ax + by + c = 0"
// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c)
{
  //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
  double sub_x = fabs(start.x - end.x);
  double sub_y = fabs(start.y - end.y);
  double error = pow(10, -5);  // 0.00001

  if (sub_x < error && sub_y < error)
  {
    ROS_INFO("two points are the same point!!");
    return false;
  }

  *a = end.y - start.y;
  *b = (-1) * (end.x - start.x);
  *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

  return true;
}
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
{
  double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

  return d;
}

tf::Vector3 point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x, point.y, point.z);
  return vector;
}

geometry_msgs::Point vector2point(tf::Vector3 vector)
{
  geometry_msgs::Point point;
  point.x = vector.getX();
  point.y = vector.getY();
  point.z = vector.getZ();
  return point;
}

tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree)
{
  tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
                 sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
  tf::Vector3 unit_w1 = w1.normalize();

  return unit_w1;
}

geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree)
{
  geometry_msgs::Point rotate;
  rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
  rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

  return rotate;
}
