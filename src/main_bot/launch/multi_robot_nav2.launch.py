import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# teb_local_planner (controller_server's FollowPath plugin) links against
# g2o. There's no ros-jazzy-libg2o installed system-wide on this machine
# (no root access when this was set up), so it was extracted from the .deb
# into this user-local prefix instead - controller_server needs this on
# LD_LIBRARY_PATH to dlopen libg2o_csparse_extension.so at plugin-load time
# (the other g2o libs resolve via the rpath baked in at link time, this one
# doesn't, for reasons not worth chasing further). If ros-jazzy-libg2o is
# ever installed properly via apt, this line becomes a no-op and can be
# removed.
_G2O_LOCAL_LIB = "/home/dvt/.local/ros-extra-deps/opt/ros/jazzy/lib/x86_64-linux-gnu"


def robot_nav2_bringup(pkg_share, robot_name, params_file, map_yaml, use_sim_time):
    """Namespaced Nav2 stack for one robot.

    Each robot has its own static params file (config/nav2_robot1.yaml,
    nav2_robot2.yaml) - identical except AMCL's initial_pose, which is baked
    in per robot since each dock sits at a different map-frame pose (map is
    anchored at robot1's dock, world 3.75/-4.175/yaw=90deg, since that's
    where slam_toolbox started mapping). Two earlier approaches to this both
    had real failure modes and were dropped in favor of a plain static file
    per robot - see the comment in nav2_robot1.yaml for why.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py"
            )
        ),
        launch_arguments={
            "namespace": robot_name,
            "use_namespace": "True",
            "map": map_yaml,
            "params_file": os.path.join(pkg_share, "config", params_file),
            "use_sim_time": use_sim_time,
            "slam": "False",
            # Composed bringup crashes with a SIGSEGV inside ImageMagick while
            # loading the map image (same issue as single-robot nav2.launch.py).
            "use_composition": "False",
        }.items(),
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("main_bot")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_share, "maps", "warehouse.yaml"),
        description="Full path to the map yaml file saved from slam_toolbox",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo /clock as the ROS time source",
    )

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot1_bringup = robot_nav2_bringup(
        pkg_share, "robot1", "nav2_robot1.yaml", map_yaml, use_sim_time
    )
    robot2_bringup = robot_nav2_bringup(
        pkg_share, "robot2", "nav2_robot2.yaml", map_yaml, use_sim_time
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                _G2O_LOCAL_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", ""),
            ),
            map_arg,
            use_sim_time_arg,
            robot1_bringup,
            robot2_bringup,
        ]
    )
