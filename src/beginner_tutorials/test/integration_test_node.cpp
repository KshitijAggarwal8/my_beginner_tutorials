/**
* @file integration_test_node.cpp
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

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;


////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger (""); // create an initial Logger

class MyTestsFixture {
public:
  MyTestsFixture () 
  {
    // Create the node that performs the test. (aka Integration test node):

    testerNode = rclcpp::Node::make_shared ("IntegrationTestNode1");
    Logger = testerNode->get_logger(); // make sure message will appear in rqt_console

    testerNode->declare_parameter<double> ("test_duration");

    TEST_DURATION = testerNode->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM (Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture ()
  {
  }

protected:
  double                  TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

/* A simple test case to test if the node 'talker' 
   is able to publish to the topic '/topic'     */

TEST_CASE_METHOD (MyTestsFixture, "test topic talker", "[topic]") {

  //subscribe to a specific topic we're looking for:
  bool got_topic = false;

  // Define a callback that captures the additional parameter
  struct ListenerCallback {
    ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic)
    {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM (Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  auto subscriber = testerNode->create_subscription<String> ("topic", 10, ListenerCallback (got_topic));

  rclcpp::Rate rate(10.0);       // 10hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration   = rclcpp::Clock().now() - start_time;
  auto timeout    = rclcpp::Duration::from_seconds (TEST_DURATION);
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " timeout=" << timeout.seconds());
  while (!got_topic && (duration < timeout))
    {
      rclcpp::spin_some (testerNode);
      rate.sleep();
      duration = (rclcpp::Clock().now() - start_time);
    }
  
  RCLCPP_INFO_STREAM (Logger, "Test completed. Received message: " << std::boolalpha << " got_topic=" << got_topic);

  // Using "CHECK" macro which is similar to "EXPECT_*"  macro in gtest
  CHECK (got_topic); // Test assertions - check that the topic was received
 }
