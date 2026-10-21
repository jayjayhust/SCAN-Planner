"""Preset: multi-floor GoSLAM indoor PCD map, open-loop odometry replay.

Wraps run.launch.py for the tall multi-floor scan. open_loop is required because
the quadruped kinematic sim cannot climb stairs; open_loop_controller republishes
the planned trajectory as odometry so floor changes work.

Spawn pose: open_loop_controller reads init_x/y/z from config/controllers.yaml
(run.launch.py init_x/y/z args are NOT forwarded to it). That file is already set
to the entry-floor spawn (-4.4, -2.5, 0.25).

Default navi_mode:=1 lets you drive with RViz "2D Nav Goal" and record waypoints
(see config/keypoints.multi.yaml). Pass navi_mode:=2 to replay a recorded tour.
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
        map_share, "maps", "goslam_xiaohao_hq_indoor_multi_0.1m.pcd"
    )
    default_keypoints = os.path.join(scan_share, "config", "keypoints.multi.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("navi_mode", default_value="1"),
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
                }.items(),
            ),
        ]
    )
