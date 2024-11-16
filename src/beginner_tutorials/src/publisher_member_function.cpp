/**
* @file publisher_member_function.cpp
* @author Kshitij Aggarwal - 119211618
* @brief
* @version 0.1
* @date 2024-11-08
*
* @copyright Copyright (c) 2024 Kshitij Aggarwal
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*
*/

#include <chrono>   // NOLINT(build/c++11)
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/set_string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


using std::chrono_literals::operator""ms;

/**
* @brief Publisher Node that publishes to /topic, with parameterized publishing
* frequency and echoes it to RCLCPP_INFO logging level.
*/

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Declare the parameter
    this->declare_parameter("publish_frequency", 1.0);  // Default value 1 Hz

    // Retrieve the parameter value
    double publish_frequency;
    this->get_parameter("publish_frequency", publish_frequency);

    // Convert frequency to milliseconds
    auto timer_period = std::chrono::milliseconds
                                (static_cast<int>(1000 / publish_frequency));

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    timer_ = this->create_wall_timer(
        timer_period, std::bind(&MinimalPublisher::timer_callback, this));

    if (count_ > 10000)
      RCLCPP_FATAL(this->get_logger(),
                      "Node running for too long asynchronously!");

    // Initialize the static transform broadcaster
    static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Broadcast the static transform
    broadcast_static_transform();



 /**
* @brief Service to dynamically update the string message being published
*
*/

    service_ = this->create_service<beginner_tutorials::srv::SetString>(
        "set_message",
        std::bind(&MinimalPublisher::handle_set_message,
        this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_DEBUG_STREAM(this->get_logger(),
    "Service 'set_string' is ready to accept requests." << 4);
  }
  std::string current_message_ = "Hello ENPM700! 0";  // Default message

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();

    message.data = current_message_ + std::to_string(count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void broadcast_static_transform() {
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "talk";

    // Set translation
    transform_stamped.transform.translation.x = 1.0;
    transform_stamped.transform.translation.y = 2.0;
    transform_stamped.transform.translation.z = 3.0;

    // Set rotation (in quaternion form)
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    // Broadcast the transform
    static_broadcaster_->sendTransform(transform_stamped);
    RCLCPP_INFO(this->get_logger(),
    "Static transform broadcasted from 'world' to 'base_link'");
  }

/**
* @brief Updating the message with the request received via service
*/
  void handle_set_message(
      const std::shared_ptr<beginner_tutorials::srv::SetString::Request>
      request,
      std::shared_ptr<beginner_tutorials::srv::SetString::Response> response) {
    std::string new_message = request->data;

    if (new_message == "") {
      RCLCPP_ERROR(this->get_logger(), "EMPTY MESSAGE. Cannot update!");
    } else {
      current_message_ = new_message;
      response->success = true;
      response->message = "Output string changed to: " + current_message_;
      RCLCPP_WARN(this->get_logger(), "Output string updated to: '%s'",
      current_message_.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Service<beginner_tutorials::srv::SetString>::SharedPtr service_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
