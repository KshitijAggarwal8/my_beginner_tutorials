# my_beginner_tutorials
Kshitij Aggarwal
119211618

# Overview:

A basic ROS2 package that has a publisher with a custom message "Hello ENPM700!".

# Build instructions

1. Create a workspace, with the package in the sec/
2. Move to the base of the workspace.
3. Check for dependencies: rosdep install -i --from-path src --rosdistro humble -y
4. Build the package: colcon build --packages-select beginner_tutorials

# Run instructions:
1. Source setup files: . install/setup.bash
2. Run the publisher node: ros2 run beginner_tutorials talker
