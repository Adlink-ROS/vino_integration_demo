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

#include "geometry_msgs/msg/twist.hpp"
#include "object_msgs/msg/objects_in_boxes.hpp"

enum ACTION
{
    CMD_ROTATE_RIGHT,
    CMD_ROTATE_LEFT,
};

class Integrator : public rclcpp::Node
{
public:
  explicit Integrator(const std::string & topic_name)
  : Node("integrator")
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
            obj_2_cmdvel(obj.object.object_name);
        }
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<object_msgs::msg::ObjectsInBoxes>(topic_name, 10, callback);
    std::cout << "sub topic: " << topic_name << std::endl;

    // Create a publisher with a custom Quality of Service profile.
    auto topic_cmd_vel = std::string("/cmd_vel");
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel, qos);
    std::cout << "pub topic: " << topic_cmd_vel << std::endl;
  }
  void to_publish(geometry_msgs::msg::Twist cmd_vel)
  {
    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(cmd_vel));

  }

  void pub_cmdvel(int idx)
  {
    geometry_msgs::msg::Twist cmd_vel;
    switch(idx) {
       case CMD_ROTATE_RIGHT:
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = -0.3;
            std::cout << " Turn right !." << std::endl;
           break;
        case CMD_ROTATE_LEFT:
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0.3;
            std::cout << " Turn left !." << std::endl;
            break;
        default:
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0;
            std::cout << " Stop !." << std::endl;
            break;

    }
    to_publish(cmd_vel);
  }

  void obj_2_cmdvel(std::string obj_name)
  {
      auto relation = obj2act.find(obj_name);
      if(relation != obj2act.end())
      {
        pub_cmdvel(obj2act[obj_name]);
      }
      else
        pub_cmdvel(-1);
  }

private:
  rclcpp::Subscription<object_msgs::msg::ObjectsInBoxes>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  std::map<std::string, int> obj2act =
  {
      { "person", CMD_ROTATE_RIGHT },
      { "chair",  CMD_ROTATE_LEFT }
  };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "This is demo for OpenVINO with ROS 2." << std::endl;
    auto topic = std::string("/ros2_openvino_toolkit/detected_objects");
    char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
    if (nullptr != cli_option) {
        topic = std::string(cli_option);
    }

    // Create a node.
    auto node = std::make_shared<Integrator>(topic);
    rclcpp::spin(node);
    std::cout << "shutdown!" << std::endl;
    rclcpp::shutdown();
    return 0;
}
