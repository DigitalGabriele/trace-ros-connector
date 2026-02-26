/**
 * Copyright 2016-2025 California Institute of Technology
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

/**
 * A Simple Publisher that publishes integers. Starts at zero, and increments by 1
 * every 10 seconds
 * 
 * Adapted to ROS2 from the original ROS example
 * See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 * 
 */
 
int main(int argc, char **argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("incrementer");

  // Create a publisher with QoS settings
  auto increment_publisher = node->create_publisher<std_msgs::msg::Int32>("increment", 1000);
  rclcpp::Rate loop_rate(1); // 0.1 Hz -> Every 10 seconds

  int count = 0;
  while (rclcpp::ok())
  {
    std_msgs::msg::Int32 msg;
    msg.data = count;

    RCLCPP_INFO(node->get_logger(), "Publishing count: %d", msg.data);
    increment_publisher->publish(msg);

    loop_rate.sleep();
    count++;
  }

  rclcpp::shutdown();
  return 0;
}




