from launch import LaunchDescription
from launch.actions import UnsetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # VS Code's snap packaging injects GTK_PATH into every integrated
        # terminal it spawns. Qt's gtk3 platform theme then resolves the
        # canberra-gtk-module from inside that snap, which drags in the
        # snap's bundled (older) libpthread and crashes rviz2 on startup
        # with "symbol lookup error: ... undefined symbol: __libc_pthread_init".
        UnsetEnvironmentVariable('GTK_PATH'),
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])