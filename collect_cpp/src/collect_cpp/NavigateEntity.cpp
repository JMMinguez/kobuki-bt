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
#include <iostream>
#include <iomanip>
#include <numbers>
#include <memory>
#include <cmath>

#include "collect_cpp/NavigateEntity.hpp"
#include "collect_cpp/PIDController.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

namespace collect_cpp
{


using std::placeholders::_1;
using namespace std::chrono_literals;

NavigateEntity::NavigateEntity(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  lin_pid_(0.0, 5.0, 0.0, 0.5),
  ang_pid_(0.0, M_PI / 2, 0.0, 0.5),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  geometry_msgs::msg::Twist vel;
  
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  transform_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::SensorDataQoS().reliable(),
    std::bind(&NavigateEntity::callback, this, _1));
}

void
NavigateEntity::callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg)
{
  lin_pid_.set_pid(0.6, 0.05, 0.35);
  ang_pid_.set_pid(0.6, 0.08, 0.32);
  
  getInput("target", target);

  for (int i = 0; i < std::size(msg->transforms); i++) {
    fprintf(
      stderr, "Received transform: %f, %f, %f, %f",
      msg->transforms[i].transform.translation.x, msg->transforms[i].transform.translation.y,
      msg->transforms[i].transform.rotation.x, msg->transforms[i].transform.rotation.y);

    geometry_msgs::msg::TransformStamped odom2person_msg;

    try {
      odom2person_msg = tf_buffer_.lookupTransform(
        "base_link", target,
        tf2::timeFromSec(rclcpp::Time(odom2person_msg.header.stamp).seconds()));
    } catch (tf2::TransformException & ex) {
      return;
    }

    const auto & x = odom2person_msg.transform.translation.x;
    const auto & y = odom2person_msg.transform.translation.y;

    // Cálculo de distancia y ángulo
    distancia = sqrt(pow(x, 2) + pow(y, 2));
    angulo = atan2(y, x);;
  }
}

BT::NodeStatus
NavigateEntity::tick()
{
  if (distancia <= 0.9) {   //distancia menor o igual a un metro
      vel.linear.x = 0;
      vel.angular.z = 0;
      vel_pub_->publish(vel);
      return BT::NodeStatus::SUCCESS;
    }
  else if (distancia >= 1){
      vel.angular.z = ang_pid_.get_output(angulo);

      vel.linear.x = lin_pid_.get_output(distancia);

      // Se publican velocidades
      vel_pub_->publish(vel);
      return BT::NodeStatus::RUNNING;
  }
}

void
NavigateEntity::halt()
{
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<collect_cpp::NavigateEntity>("NavigateEntity");
}

}  // namespace collect_cpp