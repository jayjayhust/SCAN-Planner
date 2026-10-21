"""Preset: single-floor GoSLAM indoor PCD map, closed-loop quadruped sim.

Wraps run.launch.py with the single-floor map and the spawn pose computed from
the real scan. Uses the CPU (pcl) renderer and 2D Nav Goal targeting
(navi_mode:=1) by default. The PCD path is resolved from the installed
map_generator share directory, so no absolute path needs to be typed.
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
    pcd_map_file = os.path.join(
        map_share, "maps", "goslam_xiaohao_hq_indoor_single_0.1m.pcd"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(run_launch),
                launch_arguments={
                    "is_real_world": "false",
                    "navi_mode": "1",
                    "sensor_type": LaunchConfiguration("sensor_type"),
                    "controller_mode": "closed_loop",
                    "use_gpu": LaunchConfiguration("use_gpu"),
                    "use_pcd_map": "true",
                    "pcd_map_file": pcd_map_file,
                    # Entry-floor spawn recomputed from the real scan bbox.
                    "init_x": "-4.4",
                    "init_y": "-2.5",
                    "init_z": "0.25",
                }.items(),
            ),
        ]
    )
