# 你应该知道的外设使用通信接口

1. ESP-01S（WIFI，烧录了 MQTT 固件）-> UART4

2. 舵机驱动板 -> UART5

3. JDY-31（蓝牙）-> USART2

## ESP-01S 基本驱动协议

- 模块：ESP-01S / ESP8266
- 固件：`ESP8266-AT_MQTT-1M.bin`
- 串口：UART4
- MCU 引脚：PC10 -> UART4_TX，PC11 -> UART4_RX
- 串口参数：9600，8 数据位，1 停止位，无校验，无硬件流控
- AT 命令结束符：`\r\n`
- 驱动文件：`hardware/ESP01S.c`、`hardware/ESP01S.h`

基础初始化顺序：

```text
AT
ATE0
AT+CWMODE=1
AT+CWJAP="ssid","password"
AT+MQTTUSERCFG=0,1,"client_id","username","password",0,0,""
AT+MQTTCONN=0,"broker_host",1883,60
```

固件上电后会由 `esp01sTask` 自动执行上述连接流程。当前关键字符串在 `hardware/ESP01S.c` 顶部以占位宏保存：

```c
#define ESP01S_WIFI_SSID              "YOUR_WIFI_SSID"
#define ESP01S_WIFI_PASSWORD          "YOUR_WIFI_PASSWORD"
#define ESP01S_MQTT_CLIENT_ID         "fire_car_esp01s"
#define ESP01S_MQTT_USERNAME          "YOUR_MQTT_USERNAME"
#define ESP01S_MQTT_PASSWORD          "YOUR_MQTT_PASSWORD"
#define ESP01S_MQTT_HOST              "YOUR_MQTT_BROKER_HOST"
#define ESP01S_MQTT_SUB_TOPIC         "fire_car/down"
#define ESP01S_MQTT_COMPAT_SUB_TOPIC  "fire_car/mission"
```

连接失败时任务会等待 5 秒后重试；收到 `WIFI DISCONNECT` 或 `+MQTTDISCONNECTED` 上报时，会重新进入连接流程。

## MQTT 任务指令协议

STM32 当前主订阅 topic：

```text
fire_car/down
```

兼容订阅 topic：

```text
fire_car/mission
```

区域编号统一使用数字：

| 区域编号 | 含义 |
|---:|---|
| `1` | 取球区，原 A 区 |
| `2` | 目标放球区，原 B 区 |
| `3` | 目标放球区，原 C 区 |

推荐下发 JSON：

```json
{"cmd":"carry_ball","from":1,"to":2,"id":1}
```

最小可用 JSON：

```json
{"to":3}
```

也可以下发纯文本：

```text
2
```

固件只接受 `to=2` 或 `to=3`。收到合法指令后，`mqttCmdTask` 会缓存最新任务，并向 `fire_car/status` 回传：

```json
{"id":1,"state":"received","from":1,"to":2}
```

收到非法指令会回传：

```json
{"state":"error","reason":"invalid_to"}
```

发布消息：

```text
AT+MQTTPUB=0,"topic","payload",0,0
```

订阅消息：

```text
AT+MQTTSUB=0,"topic",0
```

收到订阅消息时，模块会通过 UART4 主动上报：

```text
+MQTTSUBRECV:0,"topic",payload_len,payload
```

当前驱动采用 UART4 单字节中断接收 AT 响应行，命令接口会等待 `OK`、`ERROR`、`FAIL` 或超时。MQTT 订阅上报会缓存到 `ESP01S_MqttMessage_t`，上层可周期调用 `ESP01S_Poll()` 和 `ESP01S_ReadLatestMessage()` 获取最近一条消息。

## LU9685-20CU 舵机驱动板协议

- 串口：UART5
- MCU 引脚：PC12 -> UART5_TX，PD2 -> UART5_RX
- 串口参数：9600，8 数据位，1 停止位，无校验，无硬件流控
- 驱动文件：`hardware/LU9685.c`、`hardware/LU9685.h`

设置单路舵机角度：

```text
FA ADDR CHANNEL ANGLE FE
```

示例：通道 `0` 转到 `90°`：

```text
FA 00 00 5A FE
```

释放单路 PWM：

```text
FA ADDR CHANNEL C8 FE
```

复位模块：

```text
FA ADDR FB FB FE
```

当前驱动已经按 `docs/mechanical_arm_calibration.md` 的标定表写入安全范围和步进参数。可用接口包括：

```c
LU9685_SetJointAngle(ARM_JOINT_GRIPPER, 41);
LU9685_MoveJointSmooth(ARM_JOINT_BASE, 30);
LU9685_MovePoseById(ARM_POSE_HOME);
LU9685_MovePoseById(ARM_POSE_CARRY);
LU9685_MovePoseById(ARM_POSE_ZONE1_PICK);
LU9685_MovePoseById(ARM_POSE_ZONE23_PLACE);
LU9685_ReleaseJoint(ARM_JOINT_GRIPPER);
```

当前驱动已经内置 `ARM_POSE_HOME`、`ARM_POSE_CARRY`、`ARM_POSE_ZONE1_PICK` 和 `ARM_POSE_ZONE23_PLACE`，角度来自 `docs/mechanical_arm_calibration.md` 的最新标定表。

## K230 视觉回传协议

- 脚本：`tools/vision/ball_center_calibration_k230_mqtt.py`
- MQTT topic：`k230/vision/ball_center`
- 默认 broker：`192.168.137.1:1883`
- 用途：在 `ARM_POSE_HOME` 或固定视觉观察姿态下，判断球是否进入抓取前可见窗口。

当前离线标定模式下：

```python
ENABLE_JSON_LOG = False
ENABLE_MQTT = False
CALIBRATION_LOG_EVERY_N_FRAMES = 5
```

当前可抓取窗口参数：

```python
GRAB_CENTER_X = 323
GRAB_CENTER_Y = 404
CENTER_TOLERANCE_X = 45
CENTER_TOLERANCE_Y = 25
BALL_GRAB_AREA_MIN = 14000
BALL_GRAB_AREA_MAX = 17500
BALL_GRAB_SCORE_MIN = 0.70
```

对应范围：

```text
x: 278~368
y: 379~429
area: 14000~17500
score >= 0.70
```

之后 payload 会额外包含：

```json
{
  "offset": [3, -2],
  "center_ok": true,
  "area_ok": true,
  "score_ok": true,
  "area_state": "ok",
  "stable_count": 3,
  "grab_ready": true
}
```

STM32 后续只需要在小车停稳后等待 `grab_ready == true`，再执行 `ARM_POSE_ZONE1_PICK -> GRIPPER_CLOSE -> ARM_POSE_CARRY`。
