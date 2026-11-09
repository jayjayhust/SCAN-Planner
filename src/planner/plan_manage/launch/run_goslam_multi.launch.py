"""Preset: multi-floor GoSLAM indoor PCD map, open-loop odometry replay.

Wraps run.launch.py for the tall multi-floor scan. open_loop is required because
the quadruped kinematic sim cannot climb stairs; open_loop_controller republishes
the planned trajectory as odometry so floor changes work.

Spawn pose: open_loop_controller reads init_x/y/z from config/controllers.yaml
(run.launch.py init_x/y/z args are NOT forwarded to it). That file is already set
to the entry-floor spawn (-4.4, -2.5, 0.25).

Map: loads the REPAIRED stairwell scan (..._carved.pcd). The raw GoSLAM cloud
compresses the stairwell to 0.22-0.40 m free, narrower than the 0.50 m the real
Go2 footprint (double_cylinder_radius 0.25) needs, so the descent corridor was
carved to ~1.1 m; see configs/go2-xiaohao-keypoints.yaml for the full fix set.

Default navi_mode:=2 replays configs/go2-xiaohao-keypoints.yaml, the verified
42-waypoint floor1 -> stair descent -> floor2 tour (ends at W42 z=-3.72).
Pass navi_mode:=1 to drive with RViz "2D Nav Goal" and record waypoints instead.
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
        map_share, "maps", "goslam_xiaohao_hq_indoor_multi_0.1m_carved.pcd"
    )
    default_keypoints = os.path.join(
        map_share, "configs", "go2-xiaohao-keypoints.yaml"
    )

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
                }.items(),
            ),
        ]
    )
