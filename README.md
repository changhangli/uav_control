# uav_control

机载 Mini-PC（Ubuntu 20.04 / ROS Noetic）源码备份。

## 目录

| 文件夹 | 内容 |
|--------|------|
| `01_catkin_ws` | 飞控/通信（swarm_msgs、uav_control、vrpn…） |
| `02_vision_ws` | 下视相机 + ArUco 识别 |
| `03_land_ws` | 视觉精准降落（auto_land_by_pose） |
| `04_ws_livox` | Livox 驱动 + FAST-LIO（已去掉 PCD/大 gif） |
| `05_ego_ws` | EgoPlanner |
| `06_uav_worklog` | 启动脚本与说明 |
| `07_Livox-SDK2` | Livox SDK2 |

不含 `build/`、`devel/`、rosbag、PCD 点云。请在各工作空间内自行编译。
