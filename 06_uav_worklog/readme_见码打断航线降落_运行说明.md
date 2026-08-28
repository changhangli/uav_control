# 见码打断航线并降落 — 运行说明

适用：Mini-PC（`uav@192.168.50.151`），Ubuntu 20.04 + ROS Noetic。  
功能：飞机在老师地面站航线飞行时，摄像头识别到二维码 → **打断航线** → 视觉对准 → **降落**。

脚本与代码已在机载，不必重新编译即可按下面运行。

---

## 一、相关文件位置

| 类型 | 路径 |
|------|------|
| 启动脚本 | `~/uav_worklog/scripts/start_listen_hover.sh` |
| 停止脚本 | `~/uav_worklog/scripts/stop_land_stack.sh` |
| Launch | `~/land_ws/src/sunray_tutorial/launch/auto_listen_hover.launch` |
| 节点源码 | `~/land_ws/src/sunray_tutorial/advanced/auto_land_by_pose.cpp` |
| 运行日志 | `~/uav_worklog/run/listen_hover.log` |
| 提示词日志 | `~/uav_worklog/run/land_tips.log` |

---

## 二、两种模式（不要搞混）

| 命令 | 模式 | 见码后 |
|------|------|--------|
| `bash ~/uav_worklog/scripts/start_listen_hover.sh --land` | `route_land_mode` | 打断航线 → 对准 → **Land 落地** |
| `bash ~/uav_worklog/scripts/start_listen_hover.sh` | `listen_hover_mode` | 打断航线 → 码上方**悬停**（不落地） |
| `bash ~/uav_worklog/scripts/start_home_land.sh` | `home_land` | 另一套：杀 demo → 回原点再降，**不要和本说明同时开** |

本说明只写 **见码降落**（带 `--land`）。

---

## 三、运行前准备

1. 飞控栈已按原流程起来：MAVROS、`uav_control`、动捕、地面站通信等（`catkin_ws`，不要改老师工作空间）。
2. 下视相机、降落板（ArUco）就位；板子在航线能扫到的位置。
3. 若刚跑过 home_land / 旧视觉栈，先停干净：

```bash
bash ~/uav_worklog/scripts/stop_land_stack.sh
```

4. 在 **Mini-PC** 上开终端（SSH 或本机均可；本流程不依赖标定那种图形窗口）。

---

## 四、真飞步骤（按顺序）

### 1. 老师地面站

1. 解锁  
2. 起飞  
3. 选方 / 圆 / 六边等航线，飞机开始按航线飞  

不要在地面上就开本脚本；等飞机在空中飞航线后再开。

### 2. Mini-PC：启动见码降落

```bash
bash ~/uav_worklog/scripts/start_listen_hover.sh --land
```

脚本会：

1. 后台起视觉（`start_vision_verify.sh --daemon`：相机 + 地标识别）  
2. 起 `auto_listen_hover.launch`，并传入  
   `listen_hover_mode:=false`、`route_land_mode:=true`  
3. 未见码时不抢指令，地面站航线继续飞  
4. 见码后打断航线，进入视觉对准与降落  

### 3. 另开终端：看日志

```bash
tail -f ~/uav_worklog/run/listen_hover.log
```

可选：

```bash
tail -f ~/uav_worklog/run/land_tips.log
```

成功标志示例：

- `[ROUTE-READY]` / 高度足够后开始监听  
- `[ROUTE-LAND]` 或 `[INTERRUPT]`：见码并打断航线  
- 随后对准、`[FINAL]` / `[FINAL-EARLY]`、`Land`  

### 4. 地面站注意

脚本跑起来后，尽量不要再在地面站发移动 / 改航线，避免和视觉指令抢总线。

### 5. 结束 / 出问题

```bash
bash ~/uav_worklog/scripts/stop_land_stack.sh
```

地面站可手动悬停或降落。不要关飞控相关的 `uav_control` / MAVROS，除非整机收工。

---

## 五、最短命令（环境已 source 过时）

```bash
bash ~/uav_worklog/scripts/stop_land_stack.sh
bash ~/uav_worklog/scripts/start_listen_hover.sh --land
tail -f ~/uav_worklog/run/listen_hover.log
```

---

## 六、台架 / 手持（不真飞）

只看日志、不解锁：

```bash
bash ~/uav_worklog/scripts/start_listen_hover.sh --land --bench
```

---

## 七、和 home_land 怎么选

| 场景 | 用哪个 |
|------|--------|
| 航线飞着，扫到码就停航线并降到码上 | **本说明：`start_listen_hover.sh --land`** |
| 起飞后接管，先回动捕原点再视觉降 | `start_home_land.sh` |
| 见码后只要悬停、不落地 | `start_listen_hover.sh`（不加 `--land`） |

同一时间只开其中一套。

---

## 八、常见问题

| 现象 | 处理 |
|------|------|
| 见码后仍在飞航线、不打断 | 确认加了 `--land`；看日志有无 `[ROUTE-LAND]`；确认视觉在出 `/uav1/sunray_detect/qrcode_detection_ros` |
| 提示 land/mux 已在跑 | 先 `stop_land_stack.sh` 再启动 |
| 和 home_land 混用 | 先 stop，再只开本脚本 |
| 只想悬停却落地了 | 不要加 `--land` |

---

文档日期：2026-08-28  
机载核对：`start_listen_hover.sh --land` → `route_land_mode:=true`。
