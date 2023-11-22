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
 * @file publisher_member_function.cpp
 * @author Mayank Deshpande
 * @brief ROS2 Node with a publisher and a service, demonstrating member
 * function callbacks.
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cpp_srvcli/srv/talkerr.hpp>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rosbag2_cpp/writer.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class ServicePublisherNode
 * @brief ROS2 Node with a publisher and a service.
 *
 * This class represents a ROS2 Node with a publisher and a service. It inherits
 * from rclcpp::Node and demonstrates the usage of member functions as callbacks
 * for the timer and service.
 */
class ServicePublisherNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the ServicePublisherNode class.
   *
   * Initializes the ROS2 Node, creates a publisher, sets a publish frequency
   * parameter, creates a service, and sets up a timer for periodic publishing.
   */
  ServicePublisherNode() : Node("service_publisher"), count_(0) {
    // Create publisher for std_msgs::msg::String messages on the "topic"
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Declare a parameter for the publishing frequency with a default value of
    // 500
    this->declare_parameter("publish_frequency", 500);

    this->declare_parameter("record_bag", 1);

    // Set default message content
    this->message.data = "Default service message";

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");

    // Initialize the static broadcaster
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Create a service for changing the message content
    service_ = this->create_service<cpp_srvcli::srv::Talkerr>(
        "change_string",
        std::bind(&ServicePublisherNode::talkerr, this, std::placeholders::_1,
                  std::placeholders::_2));

    // Check if the publish frequency parameter is set to the default value
    if (this->get_parameter("publish_frequency").as_int() == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Publisher frequency is same: "
              << this->get_parameter("publish_frequency").as_int());
    }

    // Create a timer for periodic publishing with a default period of 500
    // milliseconds
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ServicePublisherNode::timer_callback, this));
    this->broadcast_static_transform();
  }

 private:
  /**
   * @brief Timer callback function for periodic publishing.
   *
   * Checks the publishing frequency parameter and publishes a message if within
   * the acceptable range. Displays warnings or errors if the frequency is too
   * fast or too slow.
   */
  void timer_callback() {
    // Check if the publish frequency is too fast or too slow
    if (this->get_parameter("publish_frequency").as_int() < 100) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                                   "Publish frequency too slow");
    } else if (this->get_parameter("publish_frequency").as_int() > 1000) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publish frequency too fast");
    }

    // Get the current message and publish it
    auto message = this->message;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:" << message.data);
    publisher_->publish(message);
    if (this->get_parameter("record_bag").as_int() == 1) {
      rclcpp::Time time_stamp = this->now();
      writer_->write(message, "topic", time_stamp);
    }
  }

  /**
   * @brief Service callback function for handling service requests.
   *
   * Changes the message content based on the received service request.
   *
   * @param request The service request containing a new string.
   * @param resp The service response containing the status after processing the
   * request.
   */
  void talkerr(const std::shared_ptr<cpp_srvcli::srv::Talkerr::Request> request,
               std::shared_ptr<cpp_srvcli::srv::Talkerr::Response> resp) {
    // Update the message content and set the response status
    this->message.data = request->new_string;
    resp->status = request->new_string;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Service request received: " << request->new_string);
  }

  void broadcast_static_transform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "child";
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 1.0;
    tf2::Quaternion q;
    q.setRPY(1.0, 2.0, 1.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_static_broadcaster_->sendTransform(t);
    if (this->get_parameter("record_bag").as_int() == 1) {
      rclcpp::Time time_stamp = this->now();
      writer_->write(t, "topic", time_stamp);
    }
  }

  // Member variables
  std_msgs::msg::String message;        ///< The message to be published
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic publishing
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  ///< Publisher for messages
  rclcpp::Service<cpp_srvcli::srv::Talkerr>::SharedPtr
      service_;   ///< Service for message change
  size_t count_;  ///< Counter for tracking events
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

/**
 * @brief Main function to initialize and run the ServicePublisherNode.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer indicating the exit status.
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and run the ServicePublisherNode
  rclcpp::spin(std::make_shared<ServicePublisherNode>());

  // Shutdown ROS2
  rclcpp::shutdown();

  // Return success
  return 0;
}
