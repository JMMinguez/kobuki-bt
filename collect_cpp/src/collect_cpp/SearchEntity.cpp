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
#include <utility>

#include "collect_cpp/SearchEntity.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

namespace collect_cpp
{


using namespace std::placeholders;
using namespace std::chrono_literals;

SearchEntity::SearchEntity(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration& conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  
  detection3d_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "detection_3d", 10,
    std::bind(&SearchEntity::callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  geometry_msgs::msg::Twist vel_;
  vel_.angular.z = SPEED_ANGULAR;
  vel_publisher_->publish(vel_);

}

void
SearchEntity::callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), " ENTRANDO EN CALLBACK ");
  last_detections_ = std::move(msg);
}

BT::NodeStatus
SearchEntity::tick()
{
  if (last_detections_ == nullptr) {
    RCLCPP_INFO(node_->get_logger(), " null ");
    return BT::NodeStatus::RUNNING;
  }
  int len = std::size(last_detections_->detections);

  for (int i = 0; i < len; i++) {
    if (last_detections_->detections[i].results[0].hypothesis.class_id == "person") {
      RCLCPP_INFO(node_->get_logger(), " persona detectada ");
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<collect_cpp::SearchEntity>("SearchEntity");
}

}  // namespace collect_cpp