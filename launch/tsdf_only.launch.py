from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("nvblox_ros"),
            "config",
            "galbot_home_head_front_left_tsdf.yaml",
        ]
    )
    default_rviz = PathJoinSubstitution(
        [
            FindPackageShare("nvblox_ros"),
            "rviz",
            "tsdf_only.rviz",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config,
                description="Path to the TSDF-only nvblox parameter file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start rviz2 with the TSDF surface cloud display.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config for the TSDF surface cloud.",
            ),
            Node(
                package="nvblox_ros",
                executable="tsdf_only_node",
                name="tsdf_only_node",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="nvblox_tsdf_rviz",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
