import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# nav2_bringup's own rviz_launch.py only remaps /map, /tf, /tf_static,
# /goal_pose, /clicked_point, /initialpose for namespace:=/use_namespace:=True
# - nav2_default_view.rviz's other displays (RobotModel, LaserScan, costmaps,
# plans, particle cloud, footprints) hardcode plain absolute topic names, so
# under a namespace push they silently keep listening on the global
# (unpublished) topic instead of e.g. /robot1/scan. This is the full list
# those displays actually reference.
_REMAPPINGS = [
    ("/tf", "tf"),
    ("/tf_static", "tf_static"),
    ("/map", "map"),
    ("/map_updates", "map_updates"),
    ("/robot_description", "robot_description"),
    ("/scan", "scan"),
    ("/particle_cloud", "particle_cloud"),
    ("/plan", "plan"),
    ("/local_plan", "local_plan"),
    ("/marker", "marker"),
    ("/waypoints", "waypoints"),
    ("/goal_pose", "goal_pose"),
    ("/clicked_point", "clicked_point"),
    ("/initialpose", "initialpose"),
    ("/global_costmap/costmap", "global_costmap/costmap"),
    ("/global_costmap/costmap_updates", "global_costmap/costmap_updates"),
    ("/global_costmap/voxel_marked_cloud", "global_costmap/voxel_marked_cloud"),
    ("/global_costmap/published_footprint", "global_costmap/published_footprint"),
    ("/local_costmap/costmap", "local_costmap/costmap"),
    ("/local_costmap/costmap_updates", "local_costmap/costmap_updates"),
    ("/local_costmap/voxel_marked_cloud", "local_costmap/voxel_marked_cloud"),
    ("/local_costmap/published_footprint", "local_costmap/published_footprint"),
    ("/downsampled_costmap", "downsampled_costmap"),
    ("/downsampled_costmap_updates", "downsampled_costmap_updates"),
]


def generate_launch_description():
    pkg_share = get_package_share_directory("main_bot")

    namespace_arg = DeclareLaunchArgument(
        "namespace", description="Robot namespace to view, e.g. robot1"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        # Own copy of nav2_bringup's nav2_default_view.rviz: the stock file
        # ships with the RobotModel display's "Enabled" set to false, so the
        # robot mesh never renders no matter how correctly /robot_description
        # is remapped/namespaced.
        default_value=os.path.join(pkg_share, "rviz", "multi_robot_view.rviz"),
        description="Full path to the RViz config file to use",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use the Gazebo /clock as the ROS time source"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("namespace"),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        remappings=_REMAPPINGS,
    )

    return LaunchDescription(
        [
            namespace_arg,
            rviz_config_arg,
            use_sim_time_arg,
            # VS Code's snap packaging injects GTK_PATH into every integrated
            # terminal it spawns. Qt's gtk3 platform theme then resolves the
            # canberra-gtk-module from inside that snap, which drags in the
            # snap's bundled (older) libpthread and crashes rviz2 on startup
            # with "symbol lookup error: ... undefined symbol: __libc_pthread_init" -
            # this is what leaves both robot1 and robot2's Nav2 panels stuck on
            # "unknown", since rviz2 never actually comes up to run them.
            UnsetEnvironmentVariable('GTK_PATH'),
            rviz_node,
        ]
    )
