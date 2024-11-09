# my_beginner_tutorials
Kshitij Aggarwal

119211618

# Overview:

A basic ROS2 package that has a publisher with a custom message "Hello ENPM700!".

# Build instructions

1. Create a workspace, with the package in the src/
2. Move to the base of the workspace.
3. Check for dependencies: rosdep install -i --from-path src --rosdistro humble -y
4. Build the package: colcon build --packages-select beginner_tutorials

# Run instructions:
1. Source setup files: . install/setup.bash
2. Run the publisher node: ros2 run beginner_tutorials talker

# Launch instruction:
1. Run the command: ros2 launch beginner_tutorials chitchat.launch.py publish_freq:=1.0

NOTE: The default publishing frequency is 1. However, by passing the parameter `publish_freq`, that can be changed. 
This launch file will launch both the talker and listener node together.

# Service call instructions:
1. Open another terminal in the root of the workspace
2. Source local setup: source install/setup.bash
3. Run the command:  ros2 service call /set_message beginner_tutorials/srv/SetString "{data: 'New message here.'}"

Note: If an empty message is passed, the node will generate an ERROR log message and it will revert back to the current running message.

# RESULTS:

![Publisher Output](.results/publisher_output.png)

![Launch file Output](.results/launch_output.png)

![RQT Output](.results/rqt_output.png)