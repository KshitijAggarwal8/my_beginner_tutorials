from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
 
def generate_launch_description():

    publish_freq_arg = DeclareLaunchArgument(
    'publish_freq',
    default_value='1.0',  # Default value if not provided
    description='Publish frequency for the talker node'
    )

    # Argument to enable or disable the ROS bag recorder node
    enable_rosbag_arg = DeclareLaunchArgument(
        'enable_rosbag',
        default_value='true',  # Default value is not-enabled
        description='Flag to enable or disable the ROS bag recorder node'
    )

    talker_node = Node(
        package="beginner_tutorials",
        executable="talker",
        parameters=[{"publish_frequency": LaunchConfiguration('publish_freq')}]
    )

    listener_node = Node(
        package="beginner_tutorials",
        executable="listener",
    )

    rosbag_node = Node(
        package="beginner_tutorials",
        executable="bag_recorder",
        parameters=[{"enable_rosbag": LaunchConfiguration('enable_rosbag')}]
    )

    return LaunchDescription([publish_freq_arg, 
                              enable_rosbag_arg,
                              talker_node, 
                              listener_node, 
                              rosbag_node])