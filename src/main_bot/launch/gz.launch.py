import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    package_name = "main_bot"
    pkg_share = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_share, "worlds", "warehouse.sdf"),
        description="Gazebo world to load (name or full path to an .sdf file)",
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="main_bot",
        description="Name used to spawn the robot in Gazebo",
    )

    # Defaults match the charging/docking bay documented in worlds/warehouse.sdf.
    # North is +Y in this world (receiving docks sit at y=4.425, charging bay at
    # y=-4.175), so yaw=pi/2 turns the robot's forward (+X) to face north.
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="3.75")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="-4.175")
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.025")
    spawn_yaw_arg = DeclareLaunchArgument("spawn_yaw", default_value="1.5707963267948966")

    xacro_file = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": [LaunchConfiguration("world"), " -r"]}.items(),
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", LaunchConfiguration("robot_name"),
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    # /clock and /scan still live on the Gazebo transport side and need bridging;
    # cmd_vel/odom/tf/joint_states are now published natively by ros2_control.
    bridge_config = os.path.join(pkg_share, "config", "gz_bridge.yaml")
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            # diff_drive_controller publishes on private ~/cmd_vel and ~/odom;
            # remap to the plain topic names the rest of the stack expects.
            "--controller-ros-args", "-r /diff_drive_controller/cmd_vel:=/cmd_vel",
            "--controller-ros-args", "-r /diff_drive_controller/odom:=/odom",
        ],
        output="screen",
    )

    # controller_manager only becomes available once gz_ros2_control loads the
    # spawned model, so chain the controller spawners off the spawn_entity exit.
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            world_arg,
            robot_name_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            robot_state_publisher_node,
            gz_sim,
            spawn_entity_node,
            ros_gz_bridge_node,
            delayed_joint_state_broadcaster_spawner,
            delayed_diff_drive_controller_spawner,
        ]
    )
