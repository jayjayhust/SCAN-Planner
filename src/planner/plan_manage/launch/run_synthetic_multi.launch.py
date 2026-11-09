"""Preset: synthetic two-floor PCD map, open-loop odometry replay.

Wraps run.launch.py for the procedurally generated two-floor scene
(maps/synthetic_two_floor_0.05m.pcd) built by tools/gen_multi_floor_pcd.py.
It is a clean, noise-free map for exercising SCAN-Planner's basic floor-to-floor
logic (A* + B-spline + PRESET_TARGET FSM) without the difficulty of the real
GoSLAM indoor scan.

open_loop is required because the quadruped kinematic sim cannot climb stairs;
open_loop_controller republishes the planned trajectory as odometry so floor
changes work. Its spawn is read from config/controllers.synthetic.yaml
(floor-1 entry at (0, -4.8, 0.55) facing +Y), passed via the controllers_file
override so the xiaohao preset's controllers.yaml stays untouched.

Default navi_mode:=2 replays map_generator's configs/keypoints.synthetic.yaml, a
9-point tour: floor 1 -> straight staircase (5 tread stops at tread+0.55) -> floor 2 west end.
Pass navi_mode:=1 to drive with RViz "2D Nav Goal" instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    map_share = get_package_share_directory("map_generator")
    run_launch = os.path.join(scan_share, "launch", "run.launch.py")
    pcd_map_file = os.path.join(map_share, "maps", "synthetic_two_floor_0.05m.pcd")
    default_keypoints = os.path.join(map_share, "configs", "keypoints.synthetic.yaml")
    controllers_file = os.path.join(scan_share, "config", "controllers.synthetic.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("navi_mode", default_value="2"),
            DeclareLaunchArgument("keypoints_file", default_value=default_keypoints),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(run_launch),
                launch_arguments={
                    "is_real_world": "false",
                    "navi_mode": LaunchConfiguration("navi_mode"),
                    "sensor_type": LaunchConfiguration("sensor_type"),
                    "controller_mode": "open_loop",
                    "use_gpu": LaunchConfiguration("use_gpu"),
                    "use_pcd_map": "true",
                    "pcd_map_file": pcd_map_file,
                    "keypoints_file": LaunchConfiguration("keypoints_file"),
                    "controllers_file": controllers_file,
                    "map_size_x": "20.0",
                    "map_size_y": "16.0",
                    "map_size_z": "8.0",
                }.items(),
            ),
        ]
    )
