// Copyright 2019 ADLINK Technology Ltd. Advanced Robotic Platform Group        
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

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "dynamic_vino_lib/inferences/object_detection.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

class Intergrater : public rclcpp::Node
{
public:
  explicit Intergrater(const std::string & topic_name)
  : Node("integrater")
  {

    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const object_msgs::msg::ObjectsInBoxes::SharedPtr msg) -> void
      {
        int cnt = 0;
        for(auto & obj : msg->objects_vector) 
            std::cout << "["<< ++cnt <<"] I see a: " << obj.object.object_name.c_str() << std::endl;
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<object_msgs::msg::ObjectsInBoxes>(topic_name, 10, callback);
    std::cout << "sub topic: " << topic_name << std::endl;

  }

private:
  rclcpp::Subscription<object_msgs::msg::ObjectsInBoxes>::SharedPtr sub_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "Hello world !!" << std::endl;
    auto topic = std::string("/ros2_openvino_toolkit/detected_objects");
    char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
    if (nullptr != cli_option) {
        topic = std::string(cli_option);
    }

    // Create a node.
    auto node = std::make_shared<Intergrater>(topic);
    rclcpp::spin(node);
    std::cout << "shutdown!" << std::endl;
    rclcpp::shutdown();
    return 0;
}