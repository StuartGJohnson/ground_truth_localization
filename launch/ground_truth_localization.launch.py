"""Launch the system ROS 2 ground-truth odometry and TF republisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "source_topic": "/ground_truth/odom",
        "odom_topic": "/odom",
        "map_frame_id": "map",
        "odom_frame_id": "odom",
        "base_frame_id": "base_link",
        "publish_tf": "true",
        "publish_map_to_odom": "true",
        "use_sim_time": "true",
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [Node(
            package="ground_truth_localization",
            executable="ground_truth_localization_node",
            name="ground_truth_localization_node",
            output="screen",
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                if name in {"publish_tf", "publish_map_to_odom", "use_sim_time"}
                else LaunchConfiguration(name)
                for name in defaults
            }],
        )]
    )
