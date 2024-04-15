// Copyright 2021 Intelligent Robotics Lab
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
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Por favor, proporciona el número de entidades como argumento." << std::endl;
    return 1;
  }

  int num_entity = std::stoi(argv[1]);

  auto node = rclcpp::Node::make_shared("patrolling_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("getwp_bt_node"));
  factory.registerFromPlugin(loader.getOSName("sound_bt_node"));
  factory.registerFromPlugin(loader.getOSName("search_bt_node"));
  factory.registerFromPlugin(loader.getOSName("navigate_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("collect_cpp");
  std::string xml_file = pkgpath + "/behavior_tree_xml/collect_bt.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
