import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory("main_bot")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_share, "maps", "warehouse.yaml"),
        description="Full path to the map yaml file saved from slam_toolbox",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "nav2.yaml"),
        description="Full path to the Nav2 parameters file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo /clock as the ROS time source",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "slam": "False",
            # Composed (single-process) bringup crashes with a SIGSEGV inside
            # ImageMagick while loading the map image; run isolated processes instead.
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            map_arg,
            params_file_arg,
            use_sim_time_arg,
            nav2_bringup,
        ]
    )
