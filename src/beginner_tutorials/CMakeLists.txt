cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(catch_ros2 REQUIRED )
find_package(rosbag2_cpp REQUIRED)

# # Enable testing
# enable_testing()

if(BUILD_TESTING)
  # include(CTest)

  # disable these built-in tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ###############
  ## UNIT TEST ##
  ###############
  add_executable(integration_test_node
    test/integration_test_node.cpp
  )
  # Linking a unit test executable with catch_ros2::catch_ros2_with_main
  # provides the default Catch2 main function to the executable.
  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main
  )

  ament_target_dependencies(integration_test_node
    rclcpp std_srvs std_msgs
    )
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ######################################
  ## 2.) ADD INTEGRATION TEST SUPPORT ##
  ######################################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test (ExampleIntegration_TestYAML
    LAUNCH_FILE integration_test.launch.yaml  # example in YAML
    )
  
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetString.srv"
  DEPENDENCIES std_msgs
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker 
                          rclcpp 
                          std_msgs 
                          geometry_msgs
                          tf2
                          tf2_ros)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(bag_recorder src/bag_recorder.cpp)
ament_target_dependencies(bag_recorder rclcpp rosbag2_cpp std_msgs)

install(TARGETS
  bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(integration_test_node
  rclcpp std_srvs std_msgs
  )

ament_export_dependencies(rosidl_generate_interfaces)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(talker "${cpp_typesupport_target}")
# target_link_libraries(listener "${cpp_typesupport_target}")


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch test
  DESTINATION share/${PROJECT_NAME})

ament_package()
