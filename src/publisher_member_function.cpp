/**
 * @file publisher_member_function.cpp
 * @author Mayank Deshpande
 * @brief ROS2 Node with a publisher and a service, demonstrating member function callbacks.
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cpp_srvcli/srv/talkerr.hpp>

#include "cpp_srvcli/srv/detail/talkerr__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class ServicePublisherNode
 * @brief ROS2 Node with a publisher and a service.
 *
 * This class represents a ROS2 Node with a publisher and a service. It inherits from
 * rclcpp::Node and demonstrates the usage of member functions as callbacks for the timer
 * and service.
 */
class ServicePublisherNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the ServicePublisherNode class.
   *
   * Initializes the ROS2 Node, creates a publisher, sets a publish frequency parameter,
   * creates a service, and sets up a timer for periodic publishing.
   */
  ServicePublisherNode() : Node("service_publisher"), count_(0) {
    // Create publisher for std_msgs::msg::String messages on the "topic"
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Declare a parameter for the publishing frequency with a default value of 500
    this->declare_parameter("publish_frequency", 500);

    // Set default message content
    this->message.data = "Default service message";

    // Create a service for changing the message content
    service_ = this->create_service<cpp_srvcli::srv::Talkerr>(
        "change_string",
        std::bind(&ServicePublisherNode::talkerr, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Check if the publish frequency parameter is set to the default value
    if (this->get_parameter("publish_frequency").as_int() == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Publisher frequency is same: "
              << this->get_parameter("publish_frequency").as_int());
    }

    // Create a timer for periodic publishing with a default period of 500 milliseconds
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ServicePublisherNode::timer_callback, this));
  }

 private:
  /**
   * @brief Timer callback function for periodic publishing.
   *
   * Checks the publishing frequency parameter and publishes a message if within the
   * acceptable range. Displays warnings or errors if the frequency is too fast or too slow.
   */
  void timer_callback() {
    // Check if the publish frequency is too fast or too slow
    if (this->get_parameter("publish_frequency").as_int() < 100) {
      RCLCPP_ERROR_STREAM_THROTTLE(
          this->get_logger(), *this->get_clock(), 100,
          "Publish frequency too fast");
    } else if (this->get_parameter("publish_frequency").as_int() > 1000) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "Publish frequency too fast");
    }

    // Get the current message and publish it
    auto message = this->message;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:" << message.data);
    publisher_->publish(message);
  }

  /**
   * @brief Service callback function for handling service requests.
   *
   * Changes the message content based on the received service request.
   *
   * @param request The service request containing a new string.
   * @param resp The service response containing the status after processing the request.
   */
  void talkerr(
      const std::shared_ptr<cpp_srvcli::srv::Talkerr::Request> request,
      std::shared_ptr<cpp_srvcli::srv::Talkerr::Response> resp) {
    // Update the message content and set the response status
    this->message.data = request->new_string;
    resp->status = request->new_string;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Service request received: " << request->new_string);
  }

  // Member variables
  std_msgs::msg::String message;  ///< The message to be published
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic publishing
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  ///< Publisher for messages
  rclcpp::Service<cpp_srvcli::srv::Talkerr>::SharedPtr service_;  ///< Service for message change
  size_t count_;  ///< Counter for tracking events
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
