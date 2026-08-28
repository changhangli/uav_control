# 视觉精准降落：依赖、编译与运行

适用系统：Ubuntu 20.04 + ROS Noetic（与机载 `uav@192.168.50.151` 一致）。  
源码用 U 盘拷贝即可，不要拷 `build/`、`devel/`。

拷贝后建议目录：

- `~/vision_ws/src/sunray_vision_detection`（相机 + ArUco 地标识别）
- `~/land_ws/src/sunray_tutorial`（降落节点 `auto_land_by_pose`）
- `~/catkin_ws`（飞控工作空间，须已有并先编译好；降落节点依赖其中的 `swarm_msgs` 和 `uav_common/common_lib`）

真飞主路径是 **home_land**：在机载终端跑脚本，不要手敲 `roslaunch`。脚本会起相机 → 地标识别 → `auto_land_by_pose`（`home_land_mode:=true`）→ `uav_control` → PX4。  
`ellipse_detection` / `qrcode_detection` 可随仓库一起拷，降落主流程不依赖它们。

---

## 一、需要安装哪些依赖

源码拷过来后，先装系统和 ROS 依赖。下面按本机实际用到的包列出。

### 1. 系统编译工具

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  pkg-config \
  python3 \
  python3-dev \
  python3-pip \
  python3-numpy \
  python3-yaml \
  python3-opencv
```

本机核对过：CMake 3.16.3、Python 3.8、OpenCV 4.2。`landmark_detection` 要求 CMake ≥ 3.12、OpenCV 4。

### 2. 图像 / 相机底层库

```bash
sudo apt install -y \
  libopencv-dev \
  libeigen3-dev \
  libboost-all-dev \
  libyaml-cpp-dev \
  libv4l-dev \
  v4l-utils \
  ffmpeg \
  libavcodec-dev \
  libavformat-dev \
  libavutil-dev \
  libswscale-dev
```

`web_cam` 通过 pkg-config 找 `libavcodec` / `libavutil` / `libswscale` / `libv4l2`。

### 3. ROS Noetic

若尚未装 ROS：

```bash
sudo apt install -y ros-noetic-desktop-full
```

视觉降落还需要（`desktop-full` 通常已带大部分，缺哪个补哪个）：

```bash
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-image-geometry \
  ros-noetic-image-view \
  ros-noetic-rqt-image-view \
  ros-noetic-camera-info-manager \
  ros-noetic-camera-calibration \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-eigen \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-mavros \
  ros-noetic-mavros-msgs \
  ros-noetic-cmake-modules
```

`camera-calibration` 只用于棋盘格标定，真飞不需要一直开。

写入 `~/.bashrc`（没有就加）：

```bash
source /opt/ros/noetic/setup.bash
```

### 4. 源码里用到的 ROS 包（不用 apt，跟 U 盘走）

| 工作空间 | 包名 | 作用 |
|---|---|---|
| `vision_ws` | `detection_msgs` | 识别结果消息 |
| `vision_ws` | `web_cam` | USB 相机驱动 |
| `vision_ws` | `landmark_detection_ros` | ArUco 地标检测 |
| `vision_ws` | `detection_libs` | 检测库（被 landmark 编译引用，不是独立 catkin 包） |
| `land_ws` | `sunray_tutorial` | `auto_land` / `auto_land_by_pose` |
| `catkin_ws` | `swarm_msgs` | 飞控指令 / 状态 |
| `catkin_ws` | `uav_common`（`common_lib`） | 降落节点头文件 |
| `catkin_ws` | `uav_control`、`vrpn_client_ros`、`swarm_communication_bridge` | 真飞控制、动捕、地面站，不改 |

`land_ws` 的 `CMakeLists.txt` 里写死了：

```text
/home/uav/catkin_ws/src/uav_common/common_lib
```

若用户名不是 `uav`，编译前把这一行改成你机器上的真实路径。

---

## 二、如何编译拷贝过来的源码

先保证 `~/catkin_ws` 已经编译过一次：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### 1. 编译视觉（`vision_ws`）

U 盘只拷 `src`。若没有工作空间外壳：

```bash
mkdir -p ~/vision_ws/src
# 把 sunray_vision_detection 整个目录放到 ~/vision_ws/src/ 下
```

然后：

```bash
source /opt/ros/noetic/setup.bash
cd ~/vision_ws
catkin_make
source ~/vision_ws/devel/setup.bash
```

成功后应能 `rospack find web_cam`、`rospack find landmark_detection_ros`、`rospack find detection_msgs`。

### 2. 编译降落（`land_ws`）

`land_ws` 需要能找到 `detection_msgs` 和 `swarm_msgs`。本机是用软链接指向视觉包：

```bash
mkdir -p ~/land_ws/src
# 把 sunray_tutorial 放到 ~/land_ws/src/
ln -sfn ~/vision_ws/src/sunray_vision_detection/detection_msg ~/land_ws/src/detection_msg
```

然后必须先 source 飞控工作空间，再编：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/vision_ws/devel/setup.bash
cd ~/land_ws
catkin_make
source ~/land_ws/devel/setup.bash
```

成功后应有可执行文件：

```bash
ls ~/land_ws/devel/lib/sunray_tutorial/auto_land_by_pose
```

相机设备：`web_cam` 配置默认是

```text
/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0
```

换机后若没有这个 by-id，改 `~/vision_ws/src/sunray_vision_detection/web_cam/config/web_cam.yml` 里的 `video_device` 为 `/dev/video0`（不要用 `video1`，那是元数据口）。

---

## 三、量板：改 yaml，再用脚本写进 json

识别节点**只读** `aruco_detector_params.json` 里的 `markerLengths`（每个码的黑色边框边长，单位米），用来把像素尺寸解成真实距离。  
`board_physical.yaml` 是给人填的实测尺寸，**节点不读它**。换板、换打印比例后必须：尺子量 → 改 yaml → 跑脚本刷 json。只改 yaml 不跑脚本，高度 `pz` 会整体错。

本机 A4 板当前值（已写进 json）：整板约 **0.20 m**，小码 **0.012 m**（ID 1–57），大码 **0.050 m**（ID 91–94）。

**步骤：**

1. 尺子量 **小码黑色方块外沿边长**、**大码黑色方块外沿边长**（米）。整板边长可填 `board_side_m` 作记录。
2. 编辑：

```text
~/vision_ws/src/sunray_vision_detection/landmark_detection/config/board_physical.yaml
```

改这三项（有实测就填实测，脚本优先用实测，不再按整板比例缩放）：

```yaml
board_side_m: 0.20
measured_small_marker_m: 0.012
measured_large_marker_m: 0.050
depth_scale: 1.0
```

`depth_scale` 保持 `1.0`。只有高度整体偏大/偏小时才微调（例如 1.05），一般应先把相机内参标准。

3. 把 yaml 写进 json（脚本在包内 `scripts/`，会改 `config/aruco_detector_params.json` 的 `markerLengths`，并保持 CRLF）：

```bash
python3 ~/vision_ws/src/sunray_vision_detection/landmark_detection/scripts/apply_board_size.py
```

终端应打印小码/大码边长，以及 `ids/lengths` 条数都是 61。抽查 json：前 57 个长度等于小码，后 4 个（91–94）等于大码。

4. **不用** `catkin_make`。停掉再开识别（真飞栈则 `stop_land_stack.sh` 后再 `start_home_land.sh`），新边长才生效。

若 yaml 里删掉 `measured_*` 两项，脚本会按 `board_side_m / design_board_side_m`（设计稿 0.42 m）去缩放 `design_small_marker_m` / `design_large_marker_m`。有尺子时不要走这条，直接填实测。

---

## 四、棋盘格标定，以及真飞脚本

机载上日常用 `~/uav_worklog/scripts/` 里的脚本，不要再手敲 `roslaunch`。脚本自己 `source` 工作空间。

### 1. 棋盘格标定（Mini-PC 本机桌面终端）

脚本参数 `--size` 填的是 **内角点个数**（黑白格交叉点），不是格子个数。格子边长填米。

本机当前板：**9×6 格、每格 2 cm**。9 列 × 6 行格子 → 交叉点是 **8×5**，边长 **0.020 m**。  
若你数交叉点横向正好 9 个、纵向 6 个，才改成 `9x6`。数错一个，标定会失败或内参全错。

**终端 1：停旧栈并开相机**

```bash
bash ~/uav_worklog/scripts/stop_land_stack.sh
bash ~/uav_worklog/scripts/start_vision_verify.sh --daemon
```

（只要相机在出 `/web_cam/image_raw` 即可。标定窗口需要本机显示器，不要用纯 SSH。）

**终端 2：开标定（9×6 格、2 cm → 用 8x5）**

```bash
bash ~/uav_worklog/scripts/run_camera_calibrate.sh 8x5 0.020
```

若交叉点确实是 9×6：

```bash
bash ~/uav_worklog/scripts/run_camera_calibrate.sh 9x6 0.020
```

画面里棋盘变绿 = 尺寸填对了。不绿就停掉，改 `8x5`/`9x6` 再开。

采集：远近、左右、上下、倾斜、四角都拍到，绿框样本尽量多（官方建议几十张量级）。  
按钮亮了再点 **CALIBRATE**，等算完点 **SAVE**，可再点 **COMMIT**。不要只关窗口。

**把结果接到识别用的 yaml**

SAVE 默认写出 `/tmp/calibrationdata.tar.gz`。在本机：

```bash
mkdir -p /tmp/cam_calib && cd /tmp/cam_calib
tar -xzf /tmp/calibrationdata.tar.gz
ls
```

里面通常有 `ost.yaml`（或 `ost.txt`）。覆盖识别配置，并先备份旧文件：

```bash
cp ~/vision_ws/src/sunray_vision_detection/landmark_detection/config/head_camera.yaml \
   ~/vision_ws/src/sunray_vision_detection/landmark_detection/config/head_camera.yaml.bak_$(date +%Y%m%d)

cp /tmp/cam_calib/ost.yaml \
   ~/vision_ws/src/sunray_vision_detection/landmark_detection/config/head_camera.yaml
```

打开新 yaml，确认 `image_width` / `image_height` 是 **1280 / 720**（和 `web_cam.yml` 一致）。`camera_name` 可改成 `head_camera`。  
**不用** `catkin_make`。停掉再开视觉后新内参才生效：

```bash
bash ~/uav_worklog/scripts/stop_land_stack.sh
bash ~/uav_worklog/scripts/start_vision_verify.sh --daemon
```

### 2. 真飞（主路径：home_land）

飞控栈（MAVROS、`uav_control`、动捕 VRPN、地面站通信桥）仍按原流程先拉起来。视觉降落 **一个终端**：

```bash
bash ~/uav_worklog/scripts/start_home_land.sh
```

这条脚本会：

1. 后台起视觉栈（`start_vision_verify.sh --daemon`：相机 `web_cam` + `landmark_detection` + 位姿门控）
2. 起提示词监视 `watch_land_status.py`
3. `roslaunch sunray_tutorial auto_home_land.launch`，参数为 `home_land_mode:=true listen_hover_mode:=false route_land_mode:=false use_cmd_mux:=false`（内部 include `auto_land_by_pose.launch`，节点名 `/auto_land_1`）

**地面站（另一台电脑）顺序：** 解锁 → 起飞 → 选视觉降落航线（方/圆/六边）。然后本机跑上面的 start。节点就绪后：杀地面站 demo → 飞动捕原点附近 `(0, 0, 1)` → 悬停约 2.5 s → 视觉对准下降。离地约 **0.18 m** 发 Land（`land_agl`）。

日志：`tail -f ~/uav_worklog/run/home_land.log` 以及 `tail -f ~/uav_worklog/run/land_tips.log`。看到 `[KILL-DEMO]`、`[HOME-READY]`、`[HOME] auto land`、`[AGL-LAND]` 即按设计在走。

手持/台架不解锁：`bash ~/uav_worklog/scripts/start_home_land.sh --bench`。

### 3. 停掉视觉降落（含 home_land）

```bash
bash ~/uav_worklog/scripts/stop_land_stack.sh
```

已在机载核对：`start_home_land.sh` 拉起的降落节点就是 `/auto_land_1`；`stop_land_stack.sh` 会 `rosnode kill /auto_land_1`，再停 bag / 提示词 / CSV，并调用 `stop_vision_verify.sh` 关掉相机和识别。手持脚本 `handheld_home_bench.sh` 也用这一条停。不要用 `stop_vision_verify.sh` 当真飞停栈——它只停视觉，不停降落节点。不要关飞控的 `uav_control` / MAVROS，除非整机结束。

### 4. 另外两套脚本（不是真飞主路径）

| 脚本 | 何时用 |
|---|---|
| `start_home_land.sh` / `stop_land_stack.sh` | **真飞主路径**。对接老师地面站 demo，接管后回原点再视觉降落。 |
| `start_land_stack.sh` | 旧的「等老师起飞悬停后按回车再起降落」路径。视觉 + bag + 提示词后台，终端里 `read` 等回车后前台 `roslaunch auto_land_by_pose.launch`（**不是** `auto_home_land`，没有杀 demo / 回原点）。停同样用 `stop_land_stack.sh`。 |
| `start_vision_verify.sh` | 只起视觉：相机 + 地标识别 + 门控 + 记 CSV，**不起降落节点**。地面看图、对板、查码用。`--daemon` 不占终端。只停视觉：`stop_vision_verify.sh`。`start_home_land.sh` / `start_land_stack.sh` 内部都会以 `--daemon` 调它。 |

标定只需换相机或内参明显不准时做一次。真飞日常：地面站起飞选航线 → `start_home_land.sh` → 结束后 `stop_land_stack.sh`。
