from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def source_package_path(directory, filename):
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "src" / "nvblox_ros" / directory / filename
        if candidate.exists():
            return str(candidate)
    for parent in launch_path.parents:
        candidate = parent / directory / filename
        if candidate.exists():
            return str(candidate)
    return str(launch_path.parents[1] / directory / filename)


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    default_config = source_package_path(
        "config", "galbot_home_head_front_left_color_tsdf.yaml"
    )
    default_rviz = source_package_path("rviz", "color_tsdf.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config,
                description="Path to the color TSDF nvblox parameter file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start rviz2 with the color TSDF surface cloud display.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config for the TSDF surface cloud.",
            ),
            Node(
                package="nvblox_ros",
                executable="color_tsdf_node",
                name="color_tsdf_node",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="nvblox_color_tsdf_rviz",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
