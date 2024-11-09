#include <chrono>   // NOLINT(build/c++11)
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/set_string.hpp"


using std::chrono_literals::operator""ms;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {

    // Declare the parameter
    this->declare_parameter("publish_frequency", 1.0);  // Default value 1 Hz

    // Retrieve the parameter value
    double publish_frequency;
    this->get_parameter("publish_frequency", publish_frequency);

    // Convert frequency to milliseconds
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency));

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    timer_ = this->create_wall_timer(
        timer_period, std::bind(&MinimalPublisher::timer_callback, this));

    if(count_ > 10000)
      RCLCPP_FATAL(this->get_logger(), "Node running for too long asynchronously!");


    // Service
    service_ = this->create_service<beginner_tutorials::srv::SetString>(
        "set_message",
        std::bind(&MinimalPublisher::handle_set_message, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Service 'set_string' is ready to accept requests." << 4);
    // RCLCPP_INFO(this->get_logger(), "Service 'set_string' is ready to accept requests.");

  }
  std::string current_message_ = "Hello ENPM700! 0";  // Default message

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();

    // message.data = "Hello ENPM700! " + std::to_string(count_++);
    message.data = current_message_ + std::to_string(count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void handle_set_message(
      const std::shared_ptr<beginner_tutorials::srv::SetString::Request> request,
      std::shared_ptr<beginner_tutorials::srv::SetString::Response> response) {
    std::string new_message = request->data;
    // current_message_ = request->data;
    if(new_message == ""){
      RCLCPP_ERROR(this->get_logger(), "EMPTY MESSAGE. Cannot update!");
    }
    else{
      current_message_ = new_message;
      response->success = true;
      response->message = "Output string changed to: " + current_message_;
      RCLCPP_WARN(this->get_logger(), "Output string updated to: '%s'", current_message_.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::SetString>::SharedPtr service_;
  size_t count_;
  
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
