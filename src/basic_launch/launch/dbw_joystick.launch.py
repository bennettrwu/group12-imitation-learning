from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('pacmod2_game_control')
    launch_path = os.path.join(pkg_dir, 'launch', 'pacmod2_game_control.launch.xml')

    # Redirect game_control's steering output to the raw topic so the clamp
    # node can intercept it before it reaches the PACMod driver.
    game_control = GroupAction([
        SetRemap('/pacmod/steering_cmd', '/pacmod/steering_cmd_raw'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(launch_path)),
    ])

    # Clamp node: subscribes to /pacmod/steering_cmd_raw, publishes to
    # /pacmod/steering_cmd with angular_position clamped to ±7.5 rad.
    # The Polaris GEM e4 physical limit is 8.5 rad; 7.5 leaves ~1 rad buffer.
    clamp_node = Node(
        package='gem_steering_bounds',
        executable='steering_clamp',
        name='steering_clamp',
        parameters=[{'max_steering_angle': 7.5}],
    )

    return LaunchDescription([game_control, clamp_node])
