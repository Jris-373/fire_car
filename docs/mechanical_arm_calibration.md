# 六自由度机械臂标定记录

本文用于记录小车机械臂抓取任务的标定项、关键姿态和 LU9685-20CU 舵机驱动板指令格式。

## 1. 硬件与通信

机械臂自由度：

| 关节 | 说明 |
|---|---|
| 底盘旋转 | 控制机械臂整体左右转向 |
| 肩部 | 控制大臂抬起/下压 |
| 肘部 | 控制小臂伸展/回收 |
| 腕部俯仰 | 控制末端上下俯仰角 |
| 腕部旋转 | 控制夹爪绕末端轴旋转 |
| 夹爪 | 控制打开/闭合 |

当前项目通信分配：

| 外设 | 接口 |
|---|---|
| ESP-01S WiFi/MQTT | UART4 |
| LU9685-20CU 舵机驱动板 | UART5 |
| JDY-31 蓝牙 | USART2 |

LU9685 示例资料中串口默认参数：

```text
9600 bps, 8 data bits, 1 stop bit, no parity, no flow control
```

如果舵机板保持默认配置，STM32 的 UART5 也应配置为 `9600 8N1`。

## 2. LU9685 串口指令格式

### 2.1 设置单路舵机角度

```text
FA ADDR CHANNEL ANGLE FE
```

字段说明：

| 字段 | 含义 |
|---|---|
| `FA` | 帧头 |
| `ADDR` | 模块地址，默认 `00` |
| `CHANNEL` | 舵机通道号，常用 `00` 到 `0F`，20 路板可实测 `00` 到 `13` |
| `ANGLE` | 舵机角度，正常范围 `00` 到 `B4`，即 0 到 180 度 |
| `FE` | 帧尾 |

示例：

```text
FA 00 00 5A FE
```

含义：地址 `0x00` 的模块，通道 `0` 转到 `90` 度。

### 2.2 释放单路舵机 PWM

资料源码说明：通道角度设置为 `200` 时释放该路 PWM 信号。

```text
FA ADDR CHANNEL C8 FE
```

示例：

```text
FA 00 05 C8 FE
```

含义：释放地址 `0x00` 模块的通道 `5`。

### 2.3 复位舵机驱动板

```text
FA ADDR FB FB FE
```

示例：

```text
FA 00 FB FB FE
```

含义：复位默认地址 `0x00` 的 LU9685 模块。

## 3. 每个舵机必须标定的内容

每个舵机都不要默认允许 `0-180` 全范围运动。先找安全范围，再写入软件限幅。

| 标定项 | 需要记录什么 | 用途 |
|---|---|---|
| 通道号 | LU9685 的 `CHANNEL` 编号 | 确认哪个通道控制哪个关节 |
| 机械方向 | 角度增大时实际运动方向 | 后续控制时决定正负方向 |
| 最小安全角 | 不撞结构、不拉线、不顶死的最小角度 | 软件限幅 |
| 最大安全角 | 不撞结构、不拉线、不顶死的最大角度 | 软件限幅 |
| 中位角 | 关节自然居中或结构对称位置 | 调试和回零 |
| 上电安全角 | 上电后不碰撞的角度 | 初始化姿态 |
| 运动步进 | 每次变化几度 | 防止舵机突跳 |
| 步进间隔 | 每步延时多少 ms | 控制动作速度 |
| 是否需要释放 | 停止后是否释放 PWM | 降低舵机发热或保持力 |

通用记录表：

| 关节 | 通道 | 方向 | 最小安全角 | 最大安全角 | 中位角 | 上电安全角 | 步进 | 步进间隔 |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| 底盘旋转 | CH5 | 角度越小越往右（从小车后边看） | 0° | 95° | 48° | 30° | 1° | 20ms |
| 肩部 | CH4 | 角度越小越往前（以对着舵机转轴那面而言，角度越小顺时针旋转） | 14° | 115° | 68° | 110° | 1° | 20ms |
| 肘部 | CH3 | 角度越小越往前（对着舵机转轴那一面,角度越大顺时针旋转） | 0° | 80° | 40° | 10° | 2 ° | 20ms |
| 腕部俯仰 | CH1 | 对着舵机转轴那面角度越大越顺时针旋转（37°时腕部与小臂呈垂直方向，37到97°，角度越大越往前，97°腕部方向与小臂方向平行，之后，角度越大越转向地面） | 37° | 160° | 91° | 145° | 2° | 20ms |
| 腕部旋转 | CH2 | 只需要一个位置（113度） | 113° | 113° | 113° | 113° | 2° | 20ms |
| 夹爪 | CH0 | （角度越小张得越大） | 41° | 75° | 58° | 41° | 1° | 20ms |

### 3.1 初始步进参数建议

LU9685 串口协议只下发目标角度，没有速度字段。动作速度由 STM32 分段发送角度控制。

串口角度命令的最小实用单位是 `1` 度，因此初始标定不要设置小于 `1` 度的步进。普通模拟舵机常用 20ms 左右 PWM 周期，所以下发角度的步进间隔建议不要小于 `20ms`。

## 4. 本任务需要记录的位置

当前任务是：小车从 MQTT 接收搬运命令，把球从 `1` 区抓取并搬运到指定区域 `2` 或 `3`。机械臂动作只区分 `1` 区取球和非 `1` 区放球；`2`、`3` 区的差异由小车导航位置决定，机械臂释放动作可以合并。

必须记录的姿态：

| 姿态名 | 用途 | 底盘 | 肩部 | 肘部 | 腕俯仰 | 腕旋转 | 夹爪 |
|---|---|---:|---:|---:|---:|---:|---:|
| `ARM_POSE_HOME` | 上电安全位，机械臂不碰车体 | 30° | 110° | 10° | 149° | 113° | 41° |
| `ARM_POSE_CARRY` | 搬运途中姿态，夹住球并抬高 | 30° | 110° | 10° | 149° | 113° | 75° |
| `ARM_POSE_ZONE1_PICK` | `1` 区实际抓取位 | 30° | 20° | 54° | 96° | 113° | 41° |
| `ARM_POSE_ZONE23_PLACE` | `2/3` 区通用释放位 | 30° | 20° | 54° | 96° | 113° | 75° |

夹爪只需要记录三个核心角度：

| 位置 | 含义 | 角度 |
|---|---|---:|
| `GRIPPER_OPEN` | 抓取前和释放时打开 | 41 |
| `GRIPPER_CLOSE` | 能稳定夹住球的位置 | 75 |
| `GRIPPER_SAFE_HOLD` | 搬运途中保持球但不过度夹紧 | 75 |

推荐动作序列：

```text
HOME
-> ZONE1_PICK
-> GRIPPER_CLOSE
-> CARRY
-> ZONE23_PLACE
-> GRIPPER_OPEN
-> HOME
```

如果 `1` 区抓取位置和 `2/3` 区释放位置相对小车固定，机械臂只需要这些固定组合姿态。视觉只负责确认球是否在 `1` 区可抓取位置，或者辅助小车/底盘对准 `1` 区球。

抓取和释放可以使用对称的末端位置：抓取时到达 `ARM_POSE_ZONE1_PICK` 后闭合夹爪，释放时到达 `ARM_POSE_ZONE23_PLACE` 后张开夹爪。二者区别主要是夹爪状态，而不是必须额外增加预备姿态。

注意：在 `1` 区捕捉到小球不等于立刻进入 `ARM_POSE_ZONE1_PICK`。看到球只是进入对准和靠近流程，只有球已经进入固定抓取窗口后，机械臂才允许从搬运/等待姿态切换到抓取姿态。

## 5. MQTT 命令格式

STM32 当前订阅主任务命令 topic：

```text
fire_car/down
```

为了兼容前期文档，固件也会同时订阅：

```text
fire_car/mission
```

推荐 JSON 指令格式：

```json
{
  "cmd": "carry_ball",
  "from": 1,
  "to": 2,
  "id": 1
}
```

字段说明：

| 字段 | 含义 |
|---|---|
| `cmd` | 固定为 `carry_ball` |
| `from` | 起点区域，当前固定为 `1` |
| `to` | 目标区域，只允许 `2` 或 `3` |
| `id` | 指令编号，便于去重和回传状态 |

最小可用格式也可以只发目标区：

```json
{
  "to": 3
}
```

也可以发纯文本 `2` 或 `3`。旧的 `A/B/C` 输入会被兼容解析成 `1/2/3`，但新指令建议统一用数字。

建议 STM32 执行后发布状态到：

```text
fire_car/status
```

状态示例：

```json
{
  "id": 1,
  "state": "done",
  "from": 1,
  "to": 2
}
```

错误示例：

```json
{
  "id": 1,
  "state": "error",
  "reason": "ball_not_found"
}
```

## 6. 视觉抓取需要标定的内容

K230 视觉脚本当前输出灭火球中心和方向。对于 `1` 区抓取任务，视觉侧只需要服务于“能不能抓”和“是否对准”。

建议 K230 发布视觉结果到：

```text
k230/vision/ball_center
```

STM32 需要用到的字段：

| 字段 | 含义 |
|---|---|
| `status` | `ok` 表示识别到球，`empty` 表示没看到球 |
| `center` | 球中心像素坐标 `[x, y]` |
| `direction` | `left`、`center`、`right`，用于粗略对准 |
| `area` | 目标框面积，用于判断距离是否适合抓取 |
| `score` | 模型识别置信度 |
| `offset` | 球心相对抓取前可见窗口中心的像素偏差 `[dx, dy]` |
| `center_ok` | 球心是否进入抓取前可见窗口 |
| `area_ok` | 面积是否进入可抓距离范围 |
| `score_ok` | 置信度是否达到抓取阈值 |
| `area_state` | `too_far`、`ok`、`too_close` 或 `not_configured` |
| `stable_count` | 连续满足 `center_ok && area_ok && score_ok` 的帧数 |
| `grab_ready` | 是否已经满足视觉抓取条件 |

视觉侧需要标定：

| 标定项 | 说明 |
|---|---|
| `GRAB_CENTER_X` | `1` 区能抓到球时，球在图像中的 x 坐标 |
| `GRAB_CENTER_Y` | `1` 区能抓到球时，球在图像中的 y 坐标 |
| `CENTER_TOLERANCE_X` | x 方向允许的中心误差 |
| `CENTER_TOLERANCE_Y` | y 方向允许的中心误差 |
| `BALL_GRAB_AREA_MIN` | 球面积达到多少时认为距离适合抓取 |
| `BALL_GRAB_AREA_MAX` | 球面积超过多少时认为太近，不再继续靠近 |
| `BALL_GRAB_SCORE_MIN` | 置信度达到多少时允许进入抓取判断 |
| `BALL_LOST_TIMEOUT_MS` | 球丢失多久后停止抓取 |
| `BALL_STABLE_FRAME_MIN` | 连续多少帧满足抓取条件后才允许抓取 |

记录表：

| 项 | 数值 | 备注 |
|---|---:|---|
| 图像宽度 | 640 | K230 显示坐标系 |
| 图像高度 | 480 | K230 显示坐标系 |
| `GRAB_CENTER_X` | 323 | `1` 区抓取中心 |
| `GRAB_CENTER_Y` | 404 | `1` 区抓取中心 |
| `CENTER_TOLERANCE_X` | 45 | 横向可抓窗口：`278~368` |
| `CENTER_TOLERANCE_Y` | 25 | 纵向可抓窗口：`379~429` |
| `BALL_GRAB_AREA_MIN` | 14000 | 低于该值认为太远 |
| `BALL_GRAB_AREA_MAX` | 17500 | 高于该值认为太近或检测框异常 |
| `BALL_GRAB_SCORE_MIN` | 0.70 | 低于该置信度不允许抓取 |
| `BALL_LOST_TIMEOUT_MS` | 500 |  |
| `BALL_STABLE_FRAME_MIN` | 3 |  |

进入 `ARM_POSE_ZONE1_PICK` 前必须满足：

```text
vision.status == "ok"
vision.center_ok == true
vision.area_ok == true
vision.score_ok == true
vision.grab_ready == true
stable_count >= BALL_STABLE_FRAME_MIN
chassis_speed == 0
```

推荐第一版参数：

```text
BALL_STABLE_FRAME_MIN = 3
BALL_LOST_TIMEOUT_MS = 500
BALL_GRAB_SCORE_MIN = 0.70
```

当前视觉参数来自新的 `TRUE` 可抓样本和保留的 `far` 太远负样本。若后续重新采集“太近不可抓”样本，需要重新收敛 `BALL_GRAB_AREA_MAX`。

当前 K230 标定脚本路径：

```text
tools/vision/ball_center_calibration_k230_mqtt.py
```

当前先使用离线标定模式，不依赖 MQTT。脚本不会持续刷串口小球 JSON，也不会主动连接 WiFi/MQTT；识别到黄色球后，会每 `CALIBRATION_LOG_EVERY_N_FRAMES` 帧写入一行本地 CSV，并保存同编号图片：

```text
/sdcard/fire_car_vision_calibration.csv
/sdcard/fire_car_vision_images/1.jpg
/sdcard/fire_car_vision_images/2.jpg
...
```

CSV 主要字段：

```text
sample_index,image_name,session,ts_ms,found,cx,cy,area,box_x,box_y,box_w,box_h,score,direction,center_ok,area_ok,grab_ready,stable_count,fps,display_w,display_h
```

对应关系：

```text
CSV 中 sample_index=1, image_name=1.jpg -> /sdcard/fire_car_vision_images/1.jpg
CSV 中 sample_index=2, image_name=2.jpg -> /sdcard/fire_car_vision_images/2.jpg
```

当前脚本采用追加保存：

```text
CALIBRATION_LOG_RESET_ON_START = False
```

每次启动会扫描 `fire_car_vision_images/` 里已有的 `数字.jpg`，从下一个编号继续保存，避免覆盖旧图片。

用日志确定 `ARM_POSE_ZONE1_PICK` 的最小流程：

```text
1. 机械臂停在 ARM_POSE_HOME 或固定视觉标定姿态。
2. 小车/球移动到“看起来刚好可以抓”的位置。
3. 运行 K230 脚本，记录 CSV 中稳定出现的 cx、cy、area。
4. 人工微调机械臂到刚好能稳定抓球的位置。
5. 记录六个舵机角度为 ARM_POSE_ZONE1_PICK。
6. 把对应的 cx、cy、area 作为后续 BALL_GRAB_* 参数参考。
```

如果要让 Agent 根据 CSV 和图片辅助做视觉标定，需要同时提供：

```text
1. `M:\fire_car_vision_calibration.csv`。
2. `M:\fire_car_vision_images\` 目录下与 CSV 对应的 jpg。
3. 哪些 sample_index 是“刚好能抓”的正样本。
4. 哪些 sample_index 是“太远 / 太近 / 偏左 / 偏右 / 误检”的负样本。
5. 这些样本采集时机械臂姿态必须固定，例如 ARM_POSE_HOME。
6. 最终人工试抓成功时的六个舵机角度，也就是 ARM_POSE_ZONE1_PICK。
```

仅靠图片和 CSV 可以标定视觉抓取窗口，例如 `GRAB_CENTER_X/Y`、`CENTER_TOLERANCE_X/Y`、`BALL_GRAB_AREA_MIN/MAX`。但不能单独反推出六自由度机械臂的舵机角度；夹取姿态仍需要人工试抓或额外提供机械臂几何参数、相机外参和成功抓取样本。

### 6.1 抓取后的视觉限制

当前相机安装在夹爪上方，视角较高。球被夹爪抓住并抬起后，可能离开相机视野，或者被夹爪、机械臂结构遮挡。因此：

```text
抓取后图像里看不到球，不等于抓取失败。
```

视觉只作为抓取前判断依据：

```text
看到球 -> 对准 -> 距离合适 -> 稳定 N 帧 -> 停车 -> 执行抓取
```

进入 `MISSION_PICK_ZONE1` 后，不再用 `vision.status == "empty"` 判断是否失败。第一版抓取成功判断建议使用动作流程判断：

```text
ZONE1_PICK reached
-> GRIPPER_CLOSE command sent
-> wait close delay
-> CARRY reached
-> mark ball_assumed_captured = true
```

如果需要更可靠的抓取确认，可以后续增加以下任一硬件/策略：

| 方法 | 说明 |
|---|---|
| 夹爪闭合角差 | 有球时夹爪闭合角通常不会到空载极限，但普通舵机没有位置反馈时难以直接判断 |
| 夹爪电流/舵机电流 | 有球时电流变化可作为夹持判断，需要额外电流检测 |
| 夹爪内侧触碰开关 | 球进入夹爪后触发，可靠但需要加硬件 |
| 低位/侧向辅助相机 | 能看到夹爪内目标，但会增加安装和处理复杂度 |

当前项目先采用“抓取前视觉确认 + 抓取动作完成后默认已夹取”的策略。若 `CARRY` 后实际掉球，再通过任务失败或人工测试调整 `ARM_POSE_ZONE1_PICK`、`GRIPPER_CLOSE` 和 `ARM_POSE_CARRY`。

## 7. 机械臂逆运动学怎么做

### 7.1 第一阶段不建议先做完整 6DOF IK

这台机械臂虽然有 6 个舵机通道，但当前搬运任务不需要第一版就实现完整 6DOF 逆运动学。

当前任务目标很明确：

```text
1 区抓球 -> 小车搬运 -> 2/3 区释放
```

第一阶段建议优先使用固定姿态和少量关键点：

| 阶段 | 推荐做法 | 原因 |
|---|---|---|
| 第一版 | 记录 `ARM_POSE_ZONE1_PICK`、`ARM_POSE_CARRY`、`ARM_POSE_ZONE23_PLACE` 固定角度 | 能最快闭环验证抓取、夹持、搬运、释放 |
| 第一版视觉 | 只判断球是否在固定抓取窗口内 | 降低 STM32 端计算和调试复杂度 |
| 后续优化 | 对 `ARM_POSE_ZONE1_PICK` 做简化 IK 动态修正 | 适合处理 `1` 区内球心有小范围变化的情况 |
| 不建议第一版做 | 完整 6DOF 位姿 IK、连续轨迹规划、多解自动切换 | 标定量大，调试风险高，容易先卡在机械限位和坐标误差上 |

也就是说，第一阶段可以先把机械臂当成“固定抓取动作执行器”。只有当球在 `1` 区的位置变化明显，固定 `ARM_POSE_ZONE1_PICK` 覆盖不了时，再把视觉测得的球心落点转换成动态抓取姿态。

### 7.2 做 IK 前必须先测量的参数

逆运动学计算出来的是几何关节角，不是 LU9685 直接要下发的舵机角。要先把机械尺寸、坐标偏移、零位和限幅测准，否则公式正确也会撞结构。

| 参数 | 建议符号 | 怎么测 | 用途 |
|---|---|---|---|
| 肩关节高度 | `H_shoulder` | 机械臂底座安装平面或地面到肩部俯仰轴中心的垂直距离 | 把目标高度转换到肩-肘二维平面 |
| 肩-肘长度 | `L1` | 肩部俯仰轴中心到肘部俯仰轴中心的直线距离 | 二连杆第一段 |
| 肘-腕长度 | `L2` | 肘部俯仰轴中心到腕部俯仰轴中心的直线距离 | 二连杆第二段 |
| 腕到夹爪中心偏移 | `L_tool` / `Z_tool` | 腕部俯仰轴中心到夹爪夹持中心的距离；抓球时最好量到球心或夹爪中心 | 从目标点反推腕关节点位置 |
| 底盘旋转轴到车体坐标偏移 | `T_arm_in_car` | 车体参考点到机械臂底盘旋转轴的 `x/y/z` 偏移 | 把视觉/车体目标点转换到机械臂坐标 |
| 相机到车体坐标偏移 | `T_cam_in_car` | 相机光心或地面投影点相对车体参考点的位置 | 把 K230 视觉结果转换到车体坐标 |
| 夹爪抓取高度 | `Z_grab` | 能稳定夹住球时，夹爪中心或球心离地高度 | 确定 IK 的目标 `z` |
| 舵机几何零位 | `zero` | 每个关节几何角为 0° 时对应的 LU9685 舵机角 | 几何角转换成舵机命令角 |
| 舵机方向 | `direction_sign` | 几何角增大时，舵机命令角是增大还是减小 | 取 `+1` 或 `-1` |
| 舵机安全限幅 | `min_angle` / `max_angle` | 单关节慢速扫角，记录不碰撞、不拉线、不顶死范围 | IK 输出最终安全检查 |

建议先填写以下记录表，再写 IK 代码：

| 项 | 数值 | 备注 |
|---|---:|---|
| `H_shoulder` |  | 单位 mm |
| `L1` |  | 肩轴到肘轴，单位 mm |
| `L2` |  | 肘轴到腕俯仰轴，单位 mm |
| `L_tool` |  | 腕俯仰轴到夹爪中心或球心，单位 mm |
| `Z_tool` |  | 如腕到夹爪中心有明显上下偏移，单独记录 |
| `arm_offset_x` |  | 机械臂底盘旋转轴相对车体参考点，前后方向 |
| `arm_offset_y` |  | 机械臂底盘旋转轴相对车体参考点，左右方向 |
| `arm_offset_z` |  | 机械臂底盘旋转轴相对地面或车体基准高度 |
| `camera_offset_x` |  | K230 相机相对车体参考点，前后方向 |
| `camera_offset_y` |  | K230 相机相对车体参考点，左右方向 |
| `Z_grab` |  | 抓球时夹爪中心或球心离地高度 |

### 7.3 坐标系定义和视觉目标转换

为避免方向混乱，建议本项目先统一使用右手坐标，并固定如下定义：

| 坐标系 | 原点 | `x` 方向 | `y` 方向 | `z` 方向 | 用途 |
|---|---|---|---|---|---|
| 车体坐标 `CAR` | 建议选小车中心线上的固定点，或相机地面投影点 | 小车前进方向 | 小车左侧 | 向上 | 导航、视觉地面点、机械臂安装偏移 |
| 机械臂底座坐标 `ARM` | 底盘旋转轴中心在安装平面上的投影点 | 底盘舵机几何 0° 时机械臂正前方 | 左侧 | 向上 | IK 计算输入 |
| 视觉地面坐标 `VISION_GROUND` | K230 标定出的地面参考点 | 按相机标定结果定义 | 按相机标定结果定义 | 通常只输出地面点 | 把像素/面积转换成球心落点 |

如果 K230 只输出像素坐标，STM32 不要直接把像素当作机械臂坐标。需要先在 K230 侧或上位机侧完成地面标定，得到球心在车体坐标下的落点：

```text
pixel_center + ball_area
-> camera ground calibration / lookup table
-> p_ball_car = (x_car, y_car, z_car)
```

如果视觉已经输出车体坐标下的球心落点，则转换到机械臂坐标可以先用简化平移：

```text
x_arm = x_car - arm_offset_x
y_arm = y_car - arm_offset_y
z_arm = z_car - arm_offset_z
```

如果相机坐标和车体坐标存在旋转，需要使用完整外参：

```text
p_car = R_cam_to_car * p_cam + t_cam_to_car
p_arm = R_car_to_arm * (p_car - t_arm_in_car)
```

当前抓球任务里，IK 的目标点建议定义为“夹爪闭合后希望夹住球的位置”，不是图像里的像素点：

```text
p_target_arm = p_ball_arm + p_gripper_to_ball_offset
```

如果夹爪从球两侧水平夹住球，可以先把 `p_gripper_to_ball_offset` 设为 0，只用实测的 `Z_grab` 控制高度。后续如果发现夹爪中心和球心不重合，再补偿 `x/y/z` 偏移。

### 7.4 适合当前机械臂的简化 3DOF IK

这台机械臂可以先把位置 IK 简化成 3 个主要关节：

| 关节 | 是否参与位置 IK | 说明 |
|---|---|---|
| 底盘旋转 | 是 | 把机械臂转向目标点 |
| 肩部 | 是 | 在垂直平面内控制大臂 |
| 肘部 | 是 | 在垂直平面内控制小臂 |
| 腕部俯仰 | 半参与 | 不主要决定位置，用于保持夹爪抓取姿态 |
| 腕部旋转 | 否 | 当前球体抓取基本固定为 `112°` |
| 夹爪 | 否 | 只负责 `GRIPPER_OPEN` / `GRIPPER_CLOSE` |

输入目标点为机械臂底座坐标下的夹爪中心：

```text
p_target_arm = (x, y, z)
```

底盘旋转角：

```text
theta_base = atan2(y, x)
r = sqrt(x * x + y * y)
```

其中 `r` 是目标点到机械臂底盘旋转轴的水平距离。肩、肘只需要在 `r-z` 二维平面内求解。

如果腕部到夹爪中心有明显长度，并且希望夹爪保持固定俯仰姿态 `phi_tool`，先从夹爪目标点反推腕部俯仰轴目标点：

```text
r_wrist = r - L_tool * cos(phi_tool)
z_wrist = z - H_shoulder - L_tool * sin(phi_tool)
```

第一版如果还没有测准 `L_tool`，可以临时设：

```text
L_tool = 0
z_wrist = z - H_shoulder
```

二连杆求肩、肘几何角：

```text
D = (r_wrist^2 + z_wrist^2 - L1^2 - L2^2) / (2 * L1 * L2)

if D < -1 or D > 1:
    target is unreachable

theta_elbow = atan2(elbow_sign * sqrt(1 - D^2), D)

theta_shoulder =
    atan2(z_wrist, r_wrist)
    - atan2(L2 * sin(theta_elbow), L1 + L2 * cos(theta_elbow))

theta_wrist_pitch = phi_tool - theta_shoulder - theta_elbow
```

`elbow_sign` 用来选择肘上/肘下解：

| `elbow_sign` | 含义 | 建议 |
|---:|---|---|
| `+1` | 一种肘部弯曲方向 | 先离线算出角度，再看是否符合当前机械结构 |
| `-1` | 另一种肘部弯曲方向 | 选择不撞车体、不拉线、在安全限幅内的那一个 |

注意：以上 `theta_base`、`theta_shoulder`、`theta_elbow`、`theta_wrist_pitch` 都是几何角。它们不能直接当作 LU9685 的角度下发。

### 7.5 几何角转换成舵机命令角

每个关节都要单独做角度映射：

```text
servo_angle = zero + direction_sign * joint_angle_deg
```

字段含义：

| 字段 | 含义 |
|---|---|
| `zero` | 该关节几何 0° 时的舵机命令角 |
| `direction_sign` | 几何角增大时舵机角变化方向，取 `+1` 或 `-1` |
| `joint_angle_deg` | IK 算出的几何角，单位度 |
| `servo_angle` | 最终发给 LU9685 的角度，必须在安全限幅内 |

转换后不要简单粗暴地夹到限幅内继续执行。更安全的策略是：只要任一关节超限，就判定这次 IK 目标不可执行，让小车重新对准、靠近或退回固定姿态。

```text
if servo_angle < min_angle or servo_angle > max_angle:
    reject IK result
    do not move arm to this target
```

当前已标定的安全限幅应继续作为最终保护层：

| 关节 | 当前安全范围 |
|---|---|
| 底盘旋转 | `0°` 到 `95°` |
| 肩部 | `20°` 到 `115°` |
| 肘部 | `0°` 到 `80°` |
| 腕部俯仰 | `37°` 到 `145°` |
| 腕部旋转 | 固定 `112°` |
| 夹爪 | `41°` 到 `75°` |

### 7.6 IK 伪代码

下面伪代码只描述计算流程，具体 `zero`、`direction_sign` 和机械尺寸需要用实测值填入。

```text
function BuildZone1PickPoseFromVision(vision):
    if vision.status != "ok":
        return IK_FAIL_BALL_NOT_FOUND

    p_ball_car = VisionToCarGroundPoint(vision.center, vision.area)

    p_target_car.x = p_ball_car.x
    p_target_car.y = p_ball_car.y
    p_target_car.z = Z_grab

    p_target_arm = CarToArm(p_target_car)

    x = p_target_arm.x
    y = p_target_arm.y
    z = p_target_arm.z

    theta_base = atan2(y, x)
    r = sqrt(x * x + y * y)

    r_wrist = r - L_tool * cos(PHI_TOOL)
    z_wrist = z - H_shoulder - L_tool * sin(PHI_TOOL)

    D = (r_wrist * r_wrist + z_wrist * z_wrist - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if D < -1 or D > 1:
        return IK_FAIL_UNREACHABLE

    best_pose = none

    for elbow_sign in [+1, -1]:
        theta_elbow = atan2(elbow_sign * sqrt(1 - D * D), D)
        theta_shoulder = atan2(z_wrist, r_wrist) - atan2(L2 * sin(theta_elbow), L1 + L2 * cos(theta_elbow))
        theta_wrist_pitch = PHI_TOOL - theta_shoulder - theta_elbow

        pose.base = JointToServo(BASE, theta_base)
        pose.shoulder = JointToServo(SHOULDER, theta_shoulder)
        pose.elbow = JointToServo(ELBOW, theta_elbow)
        pose.wrist_pitch = JointToServo(WRIST_PITCH, theta_wrist_pitch)
        pose.wrist_rotate = 112
        pose.gripper = GRIPPER_OPEN

        if PoseInSafeLimit(pose) and PoseNoKnownCollision(pose):
            best_pose = pose
            break

    if best_pose is none:
        return IK_FAIL_LIMIT_OR_COLLISION

    return best_pose

function JointToServo(joint, joint_angle_rad):
    joint_angle_deg = rad_to_deg(joint_angle_rad)
    servo_angle = joint.zero + joint.direction_sign * joint_angle_deg

    if servo_angle < joint.min_angle or servo_angle > joint.max_angle:
        return INVALID_SERVO_ANGLE

    return servo_angle
```

执行 IK 姿态时仍然要走步进控制，不要一次性从当前角度跳到目标角度：

```text
HOME or CARRY
-> PRE_PICK_SAFE_HEIGHT
-> IK_ZONE1_PICK
-> GRIPPER_CLOSE
-> CARRY
```

如果第一版没有 `PRE_PICK_SAFE_HEIGHT`，至少保证从 `HOME` 到 `ZONE1_PICK` 是逐关节或小步同步移动，并且每步都检查限幅。

### 7.7 不可达点、多解和上电验证

IK 输出必须处理以下情况：

| 情况 | 判断方式 | 处理建议 |
|---|---|---|
| 目标太远 | `D > 1` | 小车继续靠近，或进入 `MISSION_ALIGN/APPROACH` |
| 目标太近 | `D < -1` 或 `r` 小于机械臂最小工作半径 | 小车后退或固定底盘角度重新对准 |
| 目标过高/过低 | `z_wrist` 导致肩/肘/腕超限 | 拒绝 IK，调整 `Z_grab` 或使用固定姿态 |
| 肘上/肘下都有解 | 两组角度都在数学上成立 | 固定选择经过实测的那一组，不要运行中频繁切换 |
| 数学可达但机械不可达 | 舵机角超限、夹爪撞车体、连杆拉线 | 以机械限位和碰撞检查为准 |
| 角度变化过大 | 当前姿态到目标姿态跨度大 | 插入中间姿态，低速步进执行 |

验证顺序建议：

1. 离线填入 `H_shoulder`、`L1`、`L2`、`L_tool`，用计算器或脚本算几个固定点。
2. 对比离线结果和手动标定的 `ARM_POSE_ZONE1_PICK`，确认角度方向大致一致。
3. 不夹球、低速、手靠近断电开关，只验证空载运动。
4. 先只动底盘、肩、肘，腕部俯仰固定在安全角；确认不撞后再加入腕部俯仰补偿。
5. 上电测试时使用 1° 或 2° 小步进，步进间隔不小于当前记录的 20ms 到 25ms。
6. 任何一次出现顶死、拉线、抖动严重，立即把该关节限幅收窄，不要继续依赖公式。

### 7.8 和当前 1 区到 2/3 区任务的结合方式

当前任务可以分两阶段做：

| 阶段 | 抓取 | 释放 | 视觉用途 |
|---|---|---|---|
| 固定姿态阶段 | 使用实测 `ARM_POSE_ZONE1_PICK` | 使用实测 `ARM_POSE_ZONE23_PLACE` | 只判断球是否居中、距离是否合适 |
| 简化 IK 阶段 | 根据球心落点动态生成 `ARM_POSE_ZONE1_PICK` | 仍可使用固定 `ARM_POSE_ZONE23_PLACE` | 把球心地面坐标转换成机械臂目标点 |

建议第一版状态机仍保持：

```text
vision.grab_ready == true
-> stop chassis
-> use recorded ARM_POSE_ZONE1_PICK
-> close gripper
-> carry
```

当固定抓取点稳定后，再替换为：

```text
vision.grab_ready == true and ground point valid
-> stop chassis
-> dynamic_pose = BuildZone1PickPoseFromVision(vision)
-> if dynamic_pose valid:
       move to dynamic_pose
       close gripper
       carry
   else:
       adjust chassis or enter MISSION_ERROR
```

`2/3` 区释放点在当前项目中由小车导航位置决定，机械臂可以继续用同一个 `ARM_POSE_ZONE23_PLACE`。只有后续释放容器或目标区域位置也变化时，才需要把释放动作也接入 IK。

## 8. 建议控制状态机

```text
MISSION_IDLE
MISSION_WAIT_CMD
MISSION_FIND_BALL_ZONE1
MISSION_ALIGN_BALL_ZONE1
MISSION_APPROACH_BALL_ZONE1
MISSION_PICK_ZONE1
MISSION_CARRY
MISSION_PLACE_ZONE23
MISSION_DONE
MISSION_ERROR
```

第一版可以这样实现：

1. `MISSION_WAIT_CMD`：等待 MQTT 指令，解析 `to=2/3`。
2. `MISSION_FIND_BALL_ZONE1`：读取视觉结果，只确认 `1` 区是否能看到球。
3. `MISSION_ALIGN_BALL_ZONE1`：根据 `direction` 或 `center.x` 调整小车/底盘，让球进入抓取中心。
4. `MISSION_APPROACH_BALL_ZONE1`：球居中后慢速靠近，直到 `area` 进入抓取范围。
5. `MISSION_PICK_ZONE1`：停车并确认稳定帧数后，执行 `ZONE1_PICK -> GRIPPER_CLOSE -> CARRY`。
6. `MISSION_CARRY`：小车移动到 MQTT 指定区域 `2/3`。
7. `MISSION_PLACE_ZONE23`：到达任意非 `1` 区后执行 `ZONE23_PLACE -> GRIPPER_OPEN`。
8. `MISSION_DONE`：回到 `HOME`，通过 MQTT 回传完成状态。

`1` 区抓取判断伪代码：

```text
if vision.status != "ok":
    stable_count = 0
    stay in MISSION_FIND_BALL_ZONE1

if vision.center_ok == false:
    stable_count = 0
    stay in MISSION_ALIGN_BALL_ZONE1

if vision.area_state == "too_far":
    stable_count = 0
    stay in MISSION_APPROACH_BALL_ZONE1

if vision.area_state == "too_close":
    stable_count = 0
    stop chassis
    back off or enter MISSION_ERROR

if vision.grab_ready == true:
    stop chassis
    enter MISSION_PICK_ZONE1
```

## 9. 实测记录模板

每次标定都记录日期、供电电压和目标物，避免后面复现实验时角度不一致。

| 日期 | 电池/舵机电源 | 目标物 | 备注 |
|---|---|---|---|
|  |  |  |  |

单关节测试记录：

| 关节 | 通道 | 测试角度 | 现象 | 是否安全 |
|---|---:|---:|---|---|
|  |  |  |  |  |

组合姿态测试记录：

| 姿态 | 是否碰撞 | 是否拉线 | 是否抖动 | 是否能抓取 | 备注 |
|---|---|---|---|---|---|
| `ARM_POSE_READY` |  |  |  |  |  |
| `ARM_POSE_PRE_GRAB` |  |  |  |  |  |
| `ARM_POSE_ZONE1_PICK` |  |  |  |  |  |
| `ARM_POSE_LIFT` |  |  |  |  |  |
| `ARM_POSE_ZONE23_PLACE` |  |  |  |  |  |
