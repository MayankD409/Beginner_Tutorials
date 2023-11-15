/**
Copyright © 2023 <copyright holders>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

/**
 * @file subscriber_member_function.cpp
 * @author Mayank Deshpande
 * @brief ROS2 Node with a minimal subscriber using member function callbacks.
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @class MinimalSubscriber
 * @brief ROS2 Node with a minimal subscriber.
 *
 * This class represents a ROS2 Node with a minimal subscriber. It inherits from
 * rclcpp::Node and subscribes to messages on a specified topic, using a member
 * function as a callback to process received messages.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the MinimalSubscriber class.
   *
   * Initializes the ROS2 Node and creates a subscriber for
   * std_msgs::msg::String messages on the specified topic.
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    // Create a subscriber for std_msgs::msg::String messages on the "topic"
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function for processing received messages.
   *
   * This member function is invoked whenever a new message is received on the
   * subscribed topic.
   *
   * @param msg The received std_msgs::msg::String message.
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    // Log the received message
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  // Member variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  ///< Subscriber for messages
};

/**
 * @brief Main function to initialize and run the MinimalSubscriber node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer indicating the exit status.
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and run the MinimalSubscriber node
  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  // Shutdown ROS2
  rclcpp::shutdown();

  // Return success
  return 0;
}
