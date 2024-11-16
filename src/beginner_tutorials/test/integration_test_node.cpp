// Copyright 2023 Nick Morales.
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
//

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
    /**
     * 1.) Create the node that performs the test. (aka Integration test node):
     */
    testerNode = rclcpp::Node::make_shared ("IntegrationTestNode1");
    Logger = testerNode->get_logger(); // make sure message will appear in rqt_console

    /**
     * 2.) Declare a parameter for the duration of the test:
     */
    testerNode->declare_parameter<double> ("test_duration");

    /**
     * 3.) Get the test duration value:
     */
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


////////////////////////////////////////////////
// Test Case 2
////////////////////////////////////////////////

/* In this test case, the node under test (aka Auxiliary test node)
   is a topic talker, which got launched by the launcher.
   
   We will ceate a topic listener as part of the node performing the
   test (aka Integration test node).  And the test simply checks if
   the topic is recieved within the duration of the test. */

TEST_CASE_METHOD (MyTestsFixture, "test topic talker", "[topic]") {

  /**
   * 4.) Now, subscribe to a specific topic we're looking for:
   */
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

  /**
   * 5.) Finally do the actual test:
   */
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
