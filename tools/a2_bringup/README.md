# A2 本体参数迁移 —— 测试工具

把真机从 G1 迁移到 Unitree A2 时，用来**测量和校验本体相关参数**的脚本。
所有脚本只读或只发速度指令，**不会自动修改任何配置文件**，只会把结果和建议值打印出来。

按 `preflight` 的思路：能离线算的全部离线算，必须上机的给出现成命令和判定标准。

## 脚本清单

| 脚本 | 需要真机 | 作用 |
|---|---|---|
| `a2_preflight.py` | 否 | 离线体检：把配置和 `a2_robot.json` 里的实测尺寸逐项对比，输出 PASS/WARN/FAIL 和具体的改法 |
| `a2_measure_base_height.py` | 是 | 从 `/loc/robo/odom` + `/lio/cloud_world` 测基座离地高度 `h_b`（同一坐标系内差值），并推导 `d_up`/`d_down`/`path_z_offset` |
| `a2_check_inflation.py` | 是 | 把 `/self_inflation` 的两个圆柱反解成 `d_xy`/`d_off`/`d_up`/`d_down`，和配置对账 |
| `a2_check_path_height.py` | 是 | 检查 `/global_path` 的高度链，核对 `path_z_offset + body_height` 有没有重复抬升 |
| `a2_check_tf_gaps.py` | 是 | 量 `map ↔ world` 这个 TF 的到达间隔与时间戳年龄；`scan_planner` 正是靠它判断参考坐标系是否新鲜，一旦判为过期就会急停（"走一段就停"） |
| `a2_bridge_watchdog_test.py` | 是 | 按时间线发速度指令，验证桥接的 `cmd_vel_timeout` 兜底停车和 `command_duration` |
| `a2_robot.json` | — | 实测尺寸/厂商极限/安全余量，供上面的脚本读取 |

速度、加速度、yaw 上限的测试本轮**不做**，等真机稳定后再补。

## 通用准备

```bash
cd ~/unitree_g1_nav            # 仓库根目录
source /opt/ros/humble/setup.bash
source install/setup.bash      # 需要 /self_inflation 等话题时必须先 source 工作空间
```

安全：场地平整、四周留 2 m，遥控器在手，**START 使能 / B 急停**。
建议全程录包：

```bash
ros2 bag record -o a2_batch2 /nav_cmd /loc/robo/odom /planning/bspline \
  /global_path /self_inflation /grid_map/occupancy_inflate
```

> 注意：`run.launch.py` 真机分支把控制器输出发到 `/nav_cmd`，而 `cmd_vel_bridge`
> 默认订阅 `/cmd_vel`，仓库里没有任何地方做这个转换。所以启动桥接时必须显式对齐：
>
> ```bash
> ros2 run g1_bridge cmd_vel_bridge --ros-args -p cmd_vel_topic:=/nav_cmd
> ```

---

## 测 0：基座离地高度 `h_b`（静态，先做）

A2 在平地上**站好不动**，定位模块在跑，然后：

```bash
python3 tools/a2_bringup/a2_measure_base_height.py --duration 15
```

脚本同时订阅 `/loc/robo/odom` 和 `/lio/cloud_world`。`odom` 是里程计坐标系里的
位姿、点云在它自己的坐标系里，两系之间既有平移也有旋转（实测约 4.8 cm + 1.4°），
所以**不能把一边的 z 拿去减另一边的 z**。脚本用 TF 把机器人**完整位姿**换算到点云
坐标系，在那里量脚下 0.7 m 环带内地表 z 的低分位值，再按同一坐标系内的差值给出
`h_b`；原始 `odom z` 会一并打印出来供对照。

- 通过标准：`child_frame_id` 是 `base`；`z std` < 0.01 m；`xy drift` < 0.05 m；
  地表点 ≥ 200；`h_b` 落在 [0.15, 0.80] m。
- 任一项不过，脚本会打印 `measurement is not usable`，并**跳过**推导值和
  `--write`，避免把坏值写进 `a2_robot.json`。
- **务必用卷尺交叉验证**：量"地面 → base 原点平面"（base = 机体中心，一般与
  IMU/髋关节中心同高），再用 `--tape-height` 让脚本判定：

  ```bash
  python3 tools/a2_bringup/a2_measure_base_height.py --duration 15 --tape-height 0.35
  ```

  差 > 0.05 m 判 FAIL，说明 base 原点不在你以为的位置，或两坐标系的姿态差比预期大。
- 脚本会直接打印推导值：

  | 参数 | 公式 |
  |---|---|
  | `obstacles_inflation_z_up` | `(0.65 - h_b) + 0.05` |
  | `obstacles_inflation_z_down` | `h_b - 0.30` |
  | `path_z_offset` | `h_b` |
  | `grid_map.body_height` | `0`（抬升只由 `path_z_offset` 做，两者之和 = `h_b`） |

- 确认无误后写入测量文件，再跑一次体检：

  ```bash
  python3 tools/a2_bringup/a2_measure_base_height.py --duration 15 --write
  python3 tools/a2_bringup/a2_preflight.py
  ```

## 测 1：竖直 clearance `d_up` / `d_down`

**脚本部分**（配置对账）：

```bash
# 上机前：离线体检
python3 tools/a2_bringup/a2_preflight.py

# 上机：反解 /self_inflation 的真实圆柱
python3 tools/a2_bringup/a2_check_inflation.py
```

> 真机上 `scan_planner` 和 `global_planner` 分处两个工作空间（`~/scan_planner_test/`
> 与 `~/nav_ws/`，各自还多一层 `src/`），所以脚本里的默认相对路径不成立，要显式
> 指定（默认值仍指向本仓库）：
>
> ```bash
> SP=~/scan_planner_test/src/scan_planner/planner/plan_manage/config
> NW=~/nav_ws/src
> python3 tools/a2_bringup/a2_preflight.py \
>   --planner-yaml $SP/planner.yaml \
>   --reference-yaml $SP/reference_path.map.yaml \
>   --global-yaml $NW/global_planner/config/global_planner.yaml
> python3 tools/a2_bringup/a2_check_inflation.py --planner-yaml $SP/planner.yaml
> python3 tools/a2_bringup/a2_check_path_height.py \
>   --planner-yaml $SP/planner.yaml \
>   --global-yaml $NW/global_planner/config/global_planner.yaml
> ```

`a2_check_inflation.py` 会把 marker 里的几何反算成
`d_xy = scale.x/2`、`d_off = |front-rear|/2`、
`d_up = top - odom_z`、`d_down = odom_z - bottom`，并逐项跟 `planner.yaml`、
以及由 `h_b` 推导的期望值对比。四项全 PASS 才算对。

**人工场景测试**（`d_up`/`d_down` 的语义只有真机能验）：

| 场景 | 障碍 | 期望 |
|---|---|---|
| A 可钻 | 桌面约 0.7 m 的桌子，桌下净空够 | 从桌下**穿过**（`d_up` 没把桌面判成阻挡）|
| B 可上 | 25–30 cm 台阶 | 判为**可通行** |
| C 不可上 | 35–40 cm 台阶/障碍 | 判为**障碍、绕开** |

配合 RViz 看 `/self_inflation`：圆柱**顶**应约等于机身顶 + 余量，
**底**应约等于地面 + 0.30 m。顶太低 → 钻桌会撞头；底太高 → 矮障碍漏检。

## 测 2：`path_z_offset` / `body_height` 高度链

```bash
python3 tools/a2_bringup/a2_check_path_height.py
```

脚本会打印 `/global_path` 的 z 区间、`/loc/robo/odom` 的 z、两个偏移量及其**和**，
并判定 `path_z_offset + body_height` 与 `h_b` 是否一致。

- 通过标准：路径 z 与 `/loc/robo/odom` 的 z 同量级（差 < 0.10 m），且两者之和 ≈ `h_b`。
- ⚠️ 常见错误：`path_z_offset` 和 `body_height` **都**设成 `h_b` → 路径被抬两次，机器人会明显偏高。
  正确做法是让**两者之和**等于 `h_b`。
- 工作空间已 source 时可以加 `--bspline`，额外看 `/planning/bspline` 的实际高度。

## 测 3：桥接 watchdog / command duration

终端 1（只起桥接，**不要**起规划器，避免两个节点抢 `/nav_cmd`）：

```bash
ros2 run g1_bridge cmd_vel_bridge --ros-args -p cmd_vel_topic:=/nav_cmd
```

终端 2：

```bash
python3 tools/a2_bringup/a2_bridge_watchdog_test.py --vx 0.2 --hold 3
```

按遥控器 START 使能。脚本会按时间线打印：

1. 发零速基线 —— 机器人不动；
2. 以 20 Hz 发 `vx=0.2` 共 3 s —— 机器人应以约 0.2 m/s 前进；
3. **停止发布** —— 此刻开始计时，桥接应在 `cmd_vel_timeout`(0.5 s) 内打印
   `/cmd_vel timed out; stopping the robot` 并让机器人停下；
4. 最后补发零速作为安全停。

- 通过标准：第 3 步机器人在 0.5 s + 余量内停住，桥接日志出现上面那句。
- 另测 `command_duration`：单发一次速度指令（`ros2 topic pub --once`），
  机器人应持续动作约 1 s 而非立刻停；若表现为一顿一顿，说明发布频率低于 2 Hz。

## 已应用的改动

本体几何与通行硬约束（第一批）：

| 文件 | 参数 | 值 |
|---|---|---|
| `scan_planner/.../config/planner.yaml` | `grid_map.double_cylinder_radius` | 0.315 |
| 同上 | `grid_map.double_cylinder_offset` | 0.215 |
| `global_planner/config/global_planner.yaml` | `robot_radius` | 0.315 |
| 同上 | `max_step_height` | 0.30 |
| 同上 | `max_slope_degrees` | 45.0 |

水平足迹按**覆盖模型**（SCAN-Planner 的双圆柱）取：两圆柱中心沿机身纵轴
对称布置，能完全盖住 L×W 矩形的最紧半径与偏移为
`offset = L/4 = 0.215`、`radius = √((L/4)² + (W/2)²) = 0.315`（L=0.86, W=0.46）。
等效包络 1.06 × 0.63 m，比机身本身大，但四个角都在碰撞模型之内。
`robot_radius` 取同一个 `d_xy`，保证全局膨胀与局部圆柱一致。

上机实测 $h_b = 0.464$ m（卷尺读数一致）后推出的高度链：

| 文件 | 参数 | 值 | 公式 |
|---|---|---|---|
| `scan_planner/.../config/planner.yaml` | `grid_map.obstacles_inflation_z_up` | 0.24 | `(0.65 − h_b) + 0.05` |
| 同上 | `grid_map.obstacles_inflation_z_down` | 0.16 | `h_b − 0.30` |
| 同上 | `grid_map.body_height` | 0.0 | 抬升由 `path_z_offset` 单独承担 |
| `global_planner/config/global_planner.yaml` | `path_z_offset` | 0.46 | `= h_b` |

`a2_robot.json` 的 `measured.base_height_m` 已写入 `0.464`；`a2_preflight.py`
现在 **5 项全 PASS**（含此前 SKIP 的竖直 clearance）。

改的是 `src` 侧，YAML 是 symlink 因而即时生效；只有 C++ 改动才需要重新 build：

```bash
colcon build --packages-select scan_planner global_planner
```

## 待填（需要上机实测）

速度 / 加速度 / yaw 上限：`controllers.yaml` 的 `max_vx/vy/vyaw`、`planner.yaml`
的 `fsm.max_yaw_rate`/`fsm.max_yaw_acc` 与 `manager`/`optimization` 的
`max_vel`/`max_acc`、以及 `closed_loop_controller.cpp` 的 `kMaxVYawLimit`。
本轮不做，等真机走稳后再补测。

## 自测（不需要真机）

测量脚本的判定逻辑有配套的离线测试，用合成数据驱动 `report()`、用现场构造的
`PointCloud2` 驱动点云回调，覆盖正常值、机器人 z 与地表 z 分处两个坐标系、
卷尺符/不符、地表点缺失、机器人在动、帧不对、数值超界，并逐一确认这些情况下
`--write` **不会**落盘（`a2_robot.json` 始终用临时副本）。

体检脚本另有一组测试，专门钉住**严重度排序**：状态名是字符串，`'FAIL' < 'PASS' <
'WARN'`，所以用 `max()` 去合并"两个偏差里更差的那个"会把 FAIL 悄悄降级——这正是
`strictest()` 存在的原因。

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest tools/a2_bringup/
```
