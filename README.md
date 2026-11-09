<div align="center">
  <h1>SCAN-Planner ROS 2</h1>
  <h2>面向路线引导四足长程导航的空间碰撞感知局部规划器</h2>
  <a href="https://arxiv.org/abs/2606.19555"><img alt="论文" src="https://img.shields.io/badge/论文-arXiv-b31b1b?logo=arxiv&logoColor=white"/></a>
  <a href="https://www.bilibili.com/video/BV15a7P6UEXb/"><img alt="视频" src="https://img.shields.io/badge/视频-Bilibili-FB7299?logo=bilibili&logoColor=white"/></a>
  <a href="https://wuyi2121.github.io/SCAN-Planner/"><img alt="项目主页" src="https://img.shields.io/badge/项目主页-Website-4A90E2?logo=googlechrome&logoColor=white"/></a>
</div>

<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

SCAN-Planner 是一款面向四足机器人导航的空间碰撞感知局部规划器。本分支是原生 ROS 2 自移植版本，适配 Ubuntu 22.04、ROS 2 Humble、C++17 和 `colcon` 构建系统。

本仓库是 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的衍生 ROS 2 移植版。核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin；ROS 2 适配由本仓库维护者完成，不代表原作者的官方发布或认可。

## 构建

安装 ROS 2 Humble 及包依赖后，在工作空间根目录下执行构建：

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# run this before colcon build: source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

默认构建 CPU 端本地感知后端，如需构建 OpenGL 后端可执行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
```
仓库不再链接自带的 x86_64 架构 GLFW 动态库，GPU 构建依赖系统安装的 GLFW、GLEW 和 OpenGL 相关包。

## 快速启动

启动自定义确定性仿真器与规划器：

```bash
source install/setup.bash
# 使用默认点云地图
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```

```bash
source install/setup.bash
# 使用自定义点云地图
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/home/xiaoqi_wen/Desktop/scan/SCAN-Planner/map.pcd

# 使用自定义点云地图(单层地图，使用 RViz2 的 2D 目标点工具选择导航目标)
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/home/jay/workspace/robotics/unitree/SCAN-Planner/src/simulator/map_generator/maps/goslam_xiaohao_hq_indoor_single_0.1m.pcd

# 使用自定义点云地图(多层地图，按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航)
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=2 sensor_type:=lidar \
  controller_mode:=open_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/home/jay/workspace/robotics/unitree/SCAN-Planner/src/simulator/map_generator/maps/goslam_xiaohao_hq_indoor_multi_0.1m.pcd \
  keypoints_file:=/home/jay/workspace/robotics/unitree/SCAN-Planner/src/simulator/map_generator/configs/go2-xiaohao-keypoints.yaml

# 使用自定义点云地图(多层地图-楼梯点云做了修正处理，按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航)
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=2 sensor_type:=lidar \
  controller_mode:=open_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/home/jay/workspace/robotics/unitree/SCAN-Planner/src/simulator/map_generator/maps/goslam_xiaohao_hq_indoor_multi_0.1m_carved.pcd \
  keypoints_file:=/home/jay/workspace/robotics/unitree/SCAN-Planner/src/simulator/map_generator/configs/go2-xiaohao-keypoints.yaml
```

在另一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```

RViz2 配置已适配 ROS 2 Humble：Go2 的 RobotModel 使用现有的 `meshes/base.dae`，Sliding Map Bounds 订阅 `/grid_map/sliding_map_bbox`，Goal 订阅 `/goal_point`。



导航模式说明：
- `navi_mode:=1`：使用 RViz2 的 2D 目标点工具选择导航目标
- `navi_mode:=2`：按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航
- `navi_mode:=3`：订阅 `initial_path` 话题获取全局路径，并在局部范围内进行避障

控制器模式分为 `open_loop`（开环）和 `closed_loop`（闭环）两种。本次移植保留的核心启动参数包括：`is_real_world`、`navi_mode`、`sensor_type`、`controller_mode`、`use_gpu`、`use_pcd_map` 和 `pcd_map_file`。

当 `use_pcd_map:=true` 时，必须提供已有的 PCD 点云地图文件：

```bash
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=/absolute/path/to/map.pcd
```

实际硬件部署时，激光惯导里程计（LIO）、相机和宇树（Unitree）驱动均为外部依赖，默认启动会将规划器输入映射到 `/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar` 话题以及 RealSense 对齐深度图话题，可根据实际安装的驱动栈修改话题重映射配置。

## 配置与接口

规划器、控制器和仿真器的参数分别位于：
- `src/planner/plan_manage/config/planner.yaml`
- `src/planner/plan_manage/config/controllers.yaml`
- `src/planner/plan_manage/config/simulator.yaml`

ROS 2 参数名称使用点号分隔，例如 `grid_map.resolution`。预设路径点是由 xyz 三元组组成的浮点数组：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [0.0, 0.0, 0.3, 5.0, 1.0, 0.3]
```

自定义消息类型为 `scan_planner_msgs/msg/Bspline` 和 `scan_planner_msgs/msg/DataDisp`。规划器相关话题均为相对话题，支持重映射，核心输出话题包括 `planning/bspline`、`planning/data_display` 和 `planning/go2_execution_frozen`。

关键点记录器现在是原生的 `rclpy` 可执行程序：

```bash
ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml
```

## GoSLAM 实拍地图仿真

本分支内置三张 GoSLAM 实拍的室内点云地图，位于 `src/simulator/map_generator/maps/`，随 `map_generator` 包一起安装（`colcon build` 后可通过 `get_package_share_directory("map_generator")` 解析路径），无需手动指定绝对路径：

- `goslam_xiaohao_hq_indoor_single_0.1m.pcd`：单层室内地图
- `goslam_xiaohao_hq_indoor_multi_0.1m.pcd`：多层室内地图（原始重建，楼梯井偏窄）
- `goslam_xiaohao_hq_indoor_multi_0.1m_carved.pcd`：多层室内地图（楼梯井修复版，`run_goslam_multi.launch.py` 默认加载；详见下文「多层楼梯点云修复」）

两个预设启动文件封装了 `run.launch.py`，已按实拍地图重新计算出生点（entry floor 约 `-4.4, -2.5, 0.25`）：

```bash
source install/setup.bash

# 单层：闭环四足运动学仿真 + RViz2 2D Nav Goal（navi_mode:=1），开箱即用
ros2 launch scan_planner run_goslam_single.launch.py

# 多层：开环里程计回放，默认加载 carved 修复地图并按 go2-xiaohao-keypoints.yaml 自动巡层（navi_mode:=2）
# 四足运动学仿真爬不了楼梯，必须用 open_loop；如需 RViz 2D Nav Goal 手动打点，加 navi_mode:=1
ros2 launch scan_planner run_goslam_multi.launch.py
```

两个预设都支持 `use_gpu:=true`（OpenGL 渲染后端）和 `sensor_type:=depth` 覆盖参数。

多层出生点说明：`open_loop_controller` 的 `init_x/init_y/init_z` 只从 `config/controllers.yaml` 读取，`run.launch.py` 的 `init_x/y/z` 参数**不会**转发给它（闭环路径的 `go2_kinematic_sim` 才会被 launch 参数覆盖）。修改多层出生点请编辑 `controllers.yaml` 中 `open_loop_controller` 一节。

### 多层自动巡层（navi_mode:=2）

多层自动巡层需要真实录制的三维路径点。`config/keypoints.multi.yaml` 目前只是占位模板（单点=出生点），实际使用前需先录制。`keypoint_recorder.py` 订阅里程计，把机器人当前位姿写成 `fsm.waypoints`，因此录制的路径点天然位于里程计坐标系，比在稠密点云上用鼠标点选更准（尤其是层与层难以单独显示的多层场景）。录制有两种方式：

**方式一：仿真内 2D Nav Goal 打点**（随内置仿真地图快速验证）

```bash
# 1. 手动探索模式启动（默认 navi_mode:=1），在 RViz2 里用 2D Nav Goal 驱动机器人走遍各层
ros2 launch scan_planner run_goslam_multi.launch.py

# 2. 另开终端录制停靠点为路径点（a/Enter 添加，l 列出，q 保存退出）
ros2 run scan_planner keypoint_recorder.py --odom /quad_0/body_pose \
  --output $(ros2 pkg prefix scan_planner)/share/scan_planner/config/keypoints.multi.yaml

# 3. 回放巡层
ros2 launch scan_planner run_goslam_multi.launch.py navi_mode:=2
```

**方式二：遥控实走 + rosbag 回放打点**（推荐用于真实楼栋，作者建议的做法）

无需在点云图上鼠标点选：先遥控机器人实走一遍并录包，再对着回放的里程计打点。

```bash
# 1. 遥控机器人走遍各层（含每段楼梯），录制里程计 bag；在每个目标路径点处停留约 1s
#    真机里程计话题为 /LIO/odom_vehicle；仿真改用 /quad_0/body_pose
#    可选：一并录制 /LIO/clouds_lidar /LIO/odom_imu，便于事后对照点云复核
ros2 bag record -o building_tour /LIO/odom_vehicle

# 2. 终端 A：暂停式播放，便于逐点步进
ros2 bag play building_tour --start-paused

# 3. 终端 B：对着回放的里程计打点（A 里用空格步进/暂停，机器人到停靠点后在 B 里按 a 记录）
ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle \
  --output $(ros2 pkg prefix scan_planner)/share/scan_planner/config/keypoints.multi.yaml
#    按键：a/Enter 添加，l 列出，u 撤销，r/d 按下标替换/删除，s 保存，q 保存退出
```

**真机回放巡层**：真机用 `closed_loop`（真狗能爬楼；`open_loop` 仅用于运动学仿真），并显式传入录好的路径点文件：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true navi_mode:=2 controller_mode:=closed_loop sensor_type:=lidar \
  keypoints_file:=$(ros2 pkg prefix scan_planner)/share/scan_planner/config/keypoints.multi.yaml
```

`keypoints.multi.yaml` 中的 `fsm.waypoints` 是 `[x, y, z, x, y, z, ...]` 的扁平浮点数组，坐标系为 `world`；`keypoint_recorder.py` 会以相同格式回写该文件。

录制要点：

- **z 抗抖动**：打点工具默认对 z 做 0.5s 窗口平均以抑制小跑步态的机身起伏（`--average-window` 可调，`0` 为瞬时值），x/y 取最新值；在每个路径点停留片刻可得到更干净的 z。
- **每段楼层切换至少三个点**：楼梯底、楼梯顶、目标点，缺一不可。全局参考是过点的 min-snap，A\* 前端只在 XY 扩展、z 沿直线线性插值，唯有短分段才能贴合真实楼梯几何。
- **坐标系一致**：路径点存于录制时的里程计 `world` 原点，回放导航时的定位必须与录制时同一世界原点（例如针对同一先验地图重定位），否则点位会整体偏移。
- **免重新构建**：本仓库以 `--symlink-install` 构建，`keypoints.multi.yaml` 为软链，录制写入 install 路径即写回源文件，回放前无需再 `colcon build`。

### 多层楼梯点云修复（`..._carved.pcd`）

`goslam_xiaohao_hq_indoor_multi_0.1m.pcd` 的楼梯井在 GoSLAM 重建时被压窄：按 SCAN-Planner 的机体膨胀判据（`double_cylinder_radius` 0.25、`obstacles_inflation_z_up` 0.1、`z_down` 0.4），W22–W24 处空闲走廊仅 0.22–0.40 m、W33–W35 完全堵塞，而 0.31 m 的 Go2 机身至少需要 0.50 m 空闲。半径离群点滤波无效（仅删 0–58 点）——这是真实连通几何，不是噪声。

`tools/carve_stairwell.py` 在仿真中修复该地图：沿 W22→W36 下行中心线删除一条管道内的点（横向半径 `R_LAT` 0.55，z 覆盖 `[中心线z-0.22, 中心线z+0.48]`，完整包住膨胀带以保证机体空闲）。共删除 1035 点（约占全图 0.25%），写出 420364 点的非破坏性副本 `..._carved.pcd`（`map_pub` 降采样后加载 419179 点）。修复后走廊宽度（用同一精确膨胀判据、真实半径量得）：W22 1.32 m、W23 1.46 m、W24 1.52 m、W33 0.66 m、W34 0.58 m、W35 0.92 m，均 ≥ 0.50 m。

```bash
python3 tools/carve_stairwell.py          # 试跑：只打印修复前后空闲走廊报告，不写文件
python3 tools/carve_stairwell.py --write  # 生成修复地图 ..._carved.pcd
```

**完整巡层依赖三项修复，缺一不可**（carve 只是第 1 项，另两项已固化进 `configs/go2-xiaohao-keypoints.yaml`）：

1. **地图 carve**（本工具）——把楼梯井走廊拓到 ~1.1 m，让真实 0.50 m 机身可通过。
2. **楼梯路径点 Z 抬到 tread+0.35**（W21–W42，仅抬升；W22–W24 同时在 X 上居中）——保证每个目标点**及相邻踏面间的 min-snap 斜坡**都避开膨胀带 `[tread-0.4, tread+0.1]`；一层 W1–W20 保持原值（抬升会破坏夹层下穿）。
3. **末段速度 `max_vel` 0.75→0.4**——最后 W41→W42 间距（~1.06 m）小于 `max_vel²/max_acc = 1.125 m`，默认速度下短 min-snap 剖面会触发加速度上限（`max_acc` 0.5 + `acc_tolerance` 1.0 = 1.5）。该覆盖写在 keypoints 文件里（在 `planner.yaml` 之后加载），提交版 `planner.yaml` 因此保持上游默认。

三项均已固化，`--symlink-install` 下改完即生效、无需重新构建。一键复现完整 42 点巡层（一层 → 下楼 → 二层，终点 W42 `-14.94, -7.40, -3.72` 静止）：

```bash
ros2 launch scan_planner run_goslam_multi.launch.py navi_mode:=2
```

**注意事项**：

- **carve 仅用于仿真**：真机楼梯井确实太窄（0.22–0.40 m < 0.50 m），实机对应处理是**重新 SLAM 楼梯井**而非删点；切勿把 carved 地图当真值用于实机部署。
- **务必确认加载的是 carved 地图**：`map_pub` 启动行应为 `Loaded 419179 PCD points from ..._carved.pcd`。若加载的是原始 `.pcd`（无 `_carved`），会在 W24（目标 `-13.3, -10.9, -0.35`）复现 `A-star failed` 卡死——这正是「用了错地图」的特征。
- **三项是一个整体**：只 carve 不抬 Z / 不调速仍会卡（`First three control points are in obstacles` 或末段 `Dynamic feasibility failed`）；只抬 Z / 调速不 carve 则窄走廊无法布线。
- **非破坏性**：源 PCD 永不修改，carve 只产出独立副本；重新 SLAM 后需对新点云重跑 `--write`，并按新踏面重新量取 tread+0.35。
- **到达 W42 后偶发的 `Replan failed 1000` 属良性空转**（FSM 在终点做零长度重规划），非导航失败——以机器人位姿是否已等于最后路径点来判断。
- **启动前确认只有一套节点**（`scan_planner_node`/`map_pub`/`pcl_render_node`/`open_loop_controller` 各 1 个）：并发 launch 会污染 `/quad_0/*` 共享话题，得到不可信结果。

### 坐标系对应与点云地图初始位置（关键）

打点录制能否正确回放，取决于**坐标系对应**而非路径点数值本身。SCAN-Planner 不做任何对齐：它默认 `body_pose` 里程计、点云地图和 `fsm.waypoints` 三者同处一个 `world` 坐标系。真机上栅格地图由实时点云经里程计变换到 `world`（`is_real_world:=true` 时 `grid_map.cloud_is_world=false`、`grid_map.need_extrinsic=true`）。一句话：**`world` 坐标系等同于里程计坐标系**，录制得到的路径点继承录制时 `/LIO/odom_vehicle` 所在的坐标系。

**本质要求**：回放导航时的里程计坐标系必须与录制时是同一个物理坐标系；机器人的“出发点”只是把两者锚定在一起的基准。

**出发点是完整位姿，尤其是 yaw**：里程计原点是 6 自由度位姿，对地面机器人即 `x, y, z（楼层）+ yaw（朝向）`。最易踩坑的是 **yaw**——若回放开机朝向与录制时不同，整个坐标系会绕原点旋转、所有路径点随之旋转，路线会斜切进墙体。仿真里的起始朝向即 `controllers.yaml` 中 `open_loop_controller` 的 `init_yaw`。

落地有两种方式：

- **有重定位（推荐，出发点可任意）**：定位针对先验点云地图重定位，实时里程计被表达在地图坐标系下；录制与回放都对同一先验地图重定位，机器狗即可从任意位置出发。
- **无重定位（纯里程计，出发点必须固定）**：原点即开机位姿，每次开机都会重建，必须在**同一物理位姿（位置 + 朝向）**开机——用地面标记点定位置、用墙面或特征定朝向，该 home 位姿即录制与回放共用的原点。

**点云地图初始位置配置**：

- **真机**：由定位系统重定位到先验点云地图，`world` 系即地图系，本仓库无需配置地图偏移。
- **仿真加载 PCD**：`map_generator` 的 `map_pub` 节点以 `frame_id: world` 发布点云，并提供 `map_offset_x/y/z` 参数**平移**点云以对齐里程计原点（仅平移；若两坐标系还存在 yaw 差异，需另行旋转点云或路径点）。`simulator.launch.py` 目前未把 `map_offset_*` 透出为 launch 参数（默认 `0.0`），需要时在该文件里为 `map_pub` 节点补上。
- **仿真出生位姿**：`controllers.yaml` 中 `open_loop_controller` 的 `init_x/init_y/init_z/init_yaw` 必须等于“物理出发点在该 PCD `world` 系下的位姿”；多层预设为入口层 `-4.4, -2.5, 0.25, 0.0`。

把真机录制的点云拿到仿真里验证时：要么把 `init_x/y/z/init_yaw` 设为机器狗起始位姿在该 PCD 系下的取值，要么用 `map_offset_*` 把 PCD 平移到里程计原点（朝向不一致时还需先旋转）。

**回放前先验证**：在 RViz 里把录制的路径点（FSM 通过 `displayGoalPoint` 发布目标标记）叠加到点云地图上——对齐正确时每个点都落在对应楼层的空闲处，回放的录制里程计轨迹应沿真实走过的走廊与楼梯延伸而不切墙。整体平移偏差 ⇒ 平移未对齐（用 `map_offset_*` 或固定 home 点）；出现旋转或斜切 ⇒ **yaw** 未对齐（重定位或匹配 `init_yaw`）。

## Gazebo Fortress / Go2 仿真

Go2 四足机器人物理模型基于 Gazebo Fortress、`ros_gz_sim` 和 `gz_ros2_control` 构建，对外提供 12 关节的 `joint_trajectory_controller`、`/joint_states` 话题、IMU 数据、四个足端接触力话题以及 `/clock` 时钟话题：

```bash
ros2 launch go2_description go2_sim.launch.py
```

如需在不启动物理仿真时查看模型，可运行：

```bash
ros2 launch go2_description go2_rviz.launch.py
```
旧版 Gazebo Classic 的轨迹/力可视化插件、外力插件以及宇树 ROS 1 专用插件已不在本 ROS 2 仿真版本中提供。


## 致谢

首先感谢原项目 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin 开源其研究成果与实现。本仓库在保留原项目 Apache-2.0 许可证的前提下完成 ROS 2 移植，完整署名见 [NOTICE](NOTICE)。

SCAN-Planner 的实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 的算法思路与开源代码，真实机器人定位方案基于 Elevator-LIO / FAST-LIO2 实现。

## 许可证

本仓库遵循 [Apache License 2.0](LICENSE)。分发或派生本项目时，请保留 [NOTICE](NOTICE) 中的原项目署名与许可证信息。
