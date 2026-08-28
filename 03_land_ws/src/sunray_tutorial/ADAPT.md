# sunray_tutorial 本机适配说明

来源：`https://gitee.com/yundrone_sunray2023/sunray` → `General_Module/sunray_tutorial`  
官方未改动备份：`official_upstream/`

## 行为（官方）

`auto_land_by_pose`：全自动对准 + 逐步下降落地；丢码等 5s，超时直接降落。  
官方代码里已对检测结果做：`py = -py`，`pz = -pz`，`yaw = -yaw`。

## 行为（本机，以 worklog 为准）

完整「相对官方改动 + 给老师说明」见：

`~/uav_worklog/2026-08-07_相对官方auto_land改动说明.md`

摘要：只降不升；1 码起纠偏、≥2 码才降；真飞用 `XyzPosYaw` 惯性系纠偏；`min_start_z` 以下不发令；官方备份在 `official_upstream/`。

**随用随听（2026-08-11，对接老师地面站）**：

- 老师用**图形地面站**解锁/起飞/选航线，**没有 remap**。
- 默认：`listen_hover` + `use_cmd_mux:=false`，见码后仿「点降落」切 `WITHOUT_CONTROL`→`CMD_CONTROL` 打断 demo，再悬停在码上方。
- `demo_with_listen` / `--with-demo` 仅调试用（才 remap+mux）。
- 见码降落：`start_listen_hover.sh --land`。

## 相对上游的必要改动（本机命名）

| 项目 | 官方 | 本机 |
|---|---|---|
| 控制消息包 | `sunray_msgs` | `swarm_msgs` |
| 检测消息包 | `sunray_msgs` | `detection_msgs`（与 landmark 发布类型一致） |
| 状态话题 | `.../sunray/uav_state` | `.../swarm/uav_state` |
| 控制话题 | `.../sunray/uav_control_cmd` | `.../uav_control_cmd` |
| 设置话题 | `.../sunray/setup` | `.../setup` |
| 日志头 | `sunray_logger.h` | `printf_format.h` |
| 公共库路径 | `../sunray_common/common_lib` | `../uav_common/common_lib` |

检测话题未改：`/uav1/sunray_detect/qrcode_detection_ros`

## 启动

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/land_ws/devel/setup.bash
source ~/vision_ws/devel/setup.bash   # 需要 landmark 时

# 视觉识别已跑通后：
roslaunch sunray_tutorial auto_land_by_pose.launch
```

接官方降落时，门控 `invert_py` 应设为 `false`（官方节点已取反 py）。
