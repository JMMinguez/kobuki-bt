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

#ifndef COLLECT_CPP__SEARCH_ENTITY_HPP_
#define COLLECT_CPP__SEARCH_ENTITY_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace collect_cpp
{

class SearchEntity : public BT::ConditionNode
{
public:
  SearchEntity(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  void callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort("entity")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;

  vision_msgs::msg::Detection3DArray::SharedPtr last_detections_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  static constexpr float SPEED_ANGULAR = 0.3f;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
};

}  // namespace collect_cpp

#endif  // COLLECT_CPP__SEARCH_ENTITY_HPP_
