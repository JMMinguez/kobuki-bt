// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

#include "collect_cpp/GetWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace collect_cpp
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::PoseStamped wp;

  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  wp.pose.position.x = -0.8920753598213196;
  wp.pose.position.y = -1.2171552181243896;
  myQuaternion.setRPY(0.0, 0.0, 3.1415);
  myQuaternion = myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];

  destination_point_ = wp;
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  setOutput("waypoint", destination_point_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace collect_cpp

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<collect_cpp::GetWaypoint>("GetWaypoint");
}
