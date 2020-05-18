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
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "object_msgs/msg/objects_in_boxes.hpp"
#include "std_msgs/msg/string.hpp"

class Transform : public rclcpp::Node
{
public:
    explicit Transform(const std::string & topic_name)
    : Node("Transform")
    {
        // Create a callback function for when messages are received.
        // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
        auto callback =
            [this](const object_msgs::msg::ObjectsInBoxes::SharedPtr msg) -> void
            {
                int cnt = 0;
                for(auto & obj : msg->objects_vector)
                {
                    std::cout << "["<< ++cnt <<"] I see a '" << obj.object.object_name << "'." << std::endl;
                    publish_result(obj.object.object_name);
                }
            };
  
        sub_ = create_subscription<object_msgs::msg::ObjectsInBoxes>(topic_name, 10, callback);
        std::cout << "subscriber topic: " << topic_name << std::endl;
  
        // Create a publisher with a custom Quality of Service profile.
        auto publish_topic = std::string("/openvino_objects");
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        pub_ = this->create_publisher<std_msgs::msg::String>(publish_topic, qos);
        std::cout << "publisher topic: " << publish_topic << std::endl;
    }
  
    void publish_result(std::string objname)
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = objname;
        pub_->publish(std::move(msg));
    }
  
private:
    rclcpp::Subscription<object_msgs::msg::ObjectsInBoxes>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "Start to transform detected_ojects to string!" << std::endl;
    auto topic = std::string("/ros2_openvino_toolkit/detected_objects");
    char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
    if (nullptr != cli_option) {
        topic = std::string(cli_option);
    }

    // Create a node.
    auto node = std::make_shared<Transform>(topic);
    rclcpp::spin(node);
    std::cout << "shutdown!" << std::endl;
    rclcpp::shutdown();
    return 0;
}
