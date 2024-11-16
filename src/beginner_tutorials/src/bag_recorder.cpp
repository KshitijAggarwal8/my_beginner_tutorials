/**
* @file bag_recorder.cpp
* @author Kshitij Aggarwal - 119211618
* @brief
* @version 0.1
* @date 2024-11-15
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>


/**
* @brief Bag recorder Node that finds and records every topic that's 
* available.
*/
class DynamicBagRecorder : public rclcpp::Node{
 public:
  DynamicBagRecorder()
  : Node("bag_recorder")  {
    // Declare and get the "enable_rosbag" parameter
    this->declare_parameter<bool>("enable_rosbag", false);

    bool enable_rosbag = this->get_parameter("enable_rosbag").as_bool();

    // Check if recording is enabled. Exit if it's disabled.
    if (!enable_rosbag) {
      RCLCPP_INFO(this->get_logger(),
      "Bag recording is disabled. Exiting the bag recorder node.");
      return;
    }

    // Initialize the bag writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");

    // Timer to periodically check and subscribe to new topics
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DynamicBagRecorder::update_subscriptions, this));
  }

 private:
  void update_subscriptions() {
    // Get all available topics and their types
    auto topic_names_and_types = this->get_topic_names_and_types();

    for (const auto & topic : topic_names_and_types) {
      const auto & topic_name = topic.first;
      const auto & topic_types = topic.second;

      // Skip topics already subscribed
      if (active_subscriptions_.find(topic_name) !=
      active_subscriptions_.end()) {
        continue;
      }

      // For simplicity, use the first type listed for the topic
      const auto & topic_type = topic_types.front();

      RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s of type: %s",
                  topic_name.c_str(), topic_type.c_str());

      // Dynamically create a subscription for this topic
      auto subscription = this->create_generic_subscription(
        topic_name, topic_type, rclcpp::QoS(10),
        [this, topic_name, topic_type]
        (std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
          // Write the serialized message to the bag
          writer_->write(serialized_msg, topic_name,
          topic_type, this->now());
        });

      // Store the subscription to keep it active
      active_subscriptions_[topic_name] = subscription;
    }
  }

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string,
  rclcpp::GenericSubscription::SharedPtr> active_subscriptions_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
