#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>
// #include <rclcpp/topic_endpoint_info.hpp>

class DynamicBagRecorder : public rclcpp::Node
{
public:
  DynamicBagRecorder()
  : Node("bag_recorder")
  {

    // Declare and get the "enable_rosbag" parameter
    this->declare_parameter<bool>("enable_rosbag", true);

    bool enable_rosbag = this->get_parameter("enable_rosbag").as_bool();

    // Check if recording is enabled
    if (!enable_rosbag) {
      RCLCPP_INFO(this->get_logger(), "Bag recording is disabled. Exiting the bag recorder node.");
      return;
    }

    // Initialize the bag writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");

    // Timer to periodically check and subscribe to new topics
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&DynamicBagRecorder::update_subscriptions, this));
  }

private:
  void update_subscriptions()
  {
    // Get all available topics and their types
    auto topic_names_and_types = this->get_topic_names_and_types();

    for (const auto & topic : topic_names_and_types)
    {
      const auto & topic_name = topic.first;
      const auto & topic_types = topic.second;

      // Skip topics already subscribed
      if (active_subscriptions_.find(topic_name) != active_subscriptions_.end()) {
        continue;
      }

      // For simplicity, use the first type listed for the topic
      const auto & topic_type = topic_types.front();

      RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s of type: %s", 
                  topic_name.c_str(), topic_type.c_str());

      // Dynamically create a subscription for this topic
      auto subscription = this->create_generic_subscription(
        topic_name, topic_type, rclcpp::QoS(10),
        [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
          // Write the serialized message to the bag
          writer_->write(serialized_msg, topic_name, topic_type, this->now());
        });

      // Store the subscription to keep it active
      active_subscriptions_[topic_name] = subscription;
    }
  }

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> active_subscriptions_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
