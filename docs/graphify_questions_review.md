# Graphify Questions Review

生成时间：2026-05-22

本文是对 `graphify-out/GRAPH_REPORT.md` 中 Suggested Questions 的人工/agent 补充核查。结论优先级高于自动报告中的推断边展示。

## 总结

- `Six DOF Mechanical Arm Calibration Record` 连接机械臂和 K230 视觉是合理的，直接边基本都是 `EXTRACTED`。
- `fire_car Project` 同时连接 K230 视觉和机械臂也是合理的，它是项目总览节点，覆盖了视觉目录、视觉标定数据、ESP-01S/MQTT 搬运命令流。
- `ESP01S_RunCommand()` 和 `ESP01S_ConnectConfiguredNetwork()` 的关系大多真实存在，但 Graphify 对不少 `calls` 边的方向和置信类型处理不够准。
- `main()` 没有把 STM32 主函数和 K230 Python 入口合成同一个节点；误导来自两个节点都显示为 `main()`。
- K230 Python `main()` 的多条 `calls` 推断边方向应反向重建，STM32 `main()` 只确认直接调用 `SystemClock_Config()`。

## 1. 为什么机械臂标定记录连接 K230 Vision

节点：`Six DOF Mechanical Arm Calibration Record`

来源：`docs/mechanical_arm_calibration.md`

结论：合理，不是误连。

这个文档不只是舵机角度表，还同时定义了以下内容：

- 六自由度机械臂、LU9685-20CU 舵机驱动板、UART 分配和安全角度。
- 固定姿态策略，包括 `ARM_POSE_HOME`、`ARM_POSE_CARRY`、`ARM_POSE_ZONE1_PICK`、`ARM_POSE_ZONE23_PLACE`。
- K230 抓取前视觉判断，包括 `grab_ready`、球心窗口、面积范围、置信度、稳定帧数。
- K230 标定脚本、CSV/图片输出和“抓取前视觉确认 + 抓取动作完成后默认已夹取”的策略。

直接连接里，机械臂侧和 `K230 Vision Grab Readiness` 都是 `EXTRACTED`，说明这些内容在文档中明确出现。真正的 `INFERRED` 主要出现在下一层，例如：

- `K230 Vision Grab Readiness` 与 `K230 Vision Return Protocol` 的语义相似关系。
- `Vision Calibration Pipeline` 超边把机械臂标定文档、通信文档和视觉工具 README 串起来。

## 2. 为什么 fire_car Project 同时连接 K230 Vision 和机械臂

节点：`fire_car Project`

来源：`README.md`

结论：合理，但社区标签容易误导。

`fire_car Project` 是项目总览节点。它直接 `EXTRACTED` 到：

- `tools/vision Directory`
- `data/vision_calibration Directory`
- `ESP-01S MQTT Command Flow`
- 电机、IMU、BLE、RAM 地图等项目模块

机械臂侧主要是通过任务链路间接连接：

```text
fire_car Project
-> ESP-01S MQTT Command Flow
-> ESP-01S UART4 Protocol
-> Mechanical Arm UART Allocation
-> Six DOF Mechanical Arm Calibration Record
```

这反映的是搬球任务闭环：MQTT 接收目标区域，K230 判断球是否进入抓取窗口，STM32 执行任务状态机，LU9685 机械臂执行抓取/搬运/释放。

需要注意的是自动社区命名不够理想：

- `Mechanical Arm` 社区里混有 MQTT、ESP-01S、K230 Vision Return Protocol 和任务状态机，更准确应叫 `Carry-Ball Mission / Arm-MQTT-Vision`。
- `K230 Vision` 社区里混有 README、AGENTS、Keil/CubeMX 路径和项目索引，更准确应叫 `Project Overview / Vision Assets`。

## 3. ESP01S_RunCommand() 推断关系是否正确

节点：`ESP01S_RunCommand()`

来源：`hardware/ESP01S.c`

结论：关系大多真实存在，但 Graphify 的方向/置信类型需要修正。

`ESP01S_RunCommand()` 是 ESP-01S 驱动里的底层 AT 命令执行核心。源码中它负责：

- 调用 `ESP01S_TryLockCommand()` 加锁。
- 调用 `ESP01S_RunCommandUnlocked()` 执行命令。
- 调用 `ESP01S_UnlockCommand()` 解锁。

同时，许多公开 AT/MQTT helper 会调用它，例如：

- `ESP01S_SendCommand()`
- `ESP01S_Test()`
- `ESP01S_Restart()`
- `ESP01S_SetEcho()`
- `ESP01S_SetStationMode()`
- `ESP01S_JoinAP()`
- `ESP01S_MQTTUserConfig()`
- `ESP01S_MQTTConnect()`
- `ESP01S_MQTTPublish()`
- `ESP01S_MQTTSubscribe()`
- `ESP01S_MQTTDisconnect()`

因此，这些边不应删除。但按严格 `source calls target` 理解时，部分边方向需要反向。例如公开 helper 调用 `ESP01S_RunCommand()`，不是 `ESP01S_RunCommand()` 调用所有公开 helper。

建议：

- 保留这些关系。
- 将源码里明确出现的调用从 `INFERRED` 改成 `EXTRACTED`。
- 将方向错误的 `calls` 边反向重建，或改成 `called_by` / `uses_command_core`。

## 4. main() 推断关系是否正确

Graphify 中有两个不同的 `main()`：

- STM32 固件入口：`main_main`，来源 `Core/Src/main.c`。
- K230 Python 视觉脚本入口：`ball_center_calibration_k230_mqtt_main`，来源 `tools/vision/ball_center_calibration_k230_mqtt.py`。

结论：节点没有混合，但报告只显示 `main()`，容易误读。

STM32 `main()` 当前确认的推断边：

- `main()` -> `SystemClock_Config()`：正确，源码中明确调用，但应标为 `EXTRACTED` 更合适。

报告中与 `BallDetectModel`、`.config_preprocess()`、`.collect_detections()`、`mqtt_connect()`、`mqtt_publish()` 等相关的 `main()`，实际是 K230 Python 脚本的 `main()`，不是 STM32 的 `main()`。

K230 Python `main()` 的多条调用关系本身合理，但 Graphify 的方向应反过来：

- 应为 `Python main() -> build_ball_event()`
- 应为 `Python main() -> empty_event()`
- 应为 `Python main() -> open_calibration_log()`
- 应为 `Python main() -> save_calibration_image()`
- 应为 `Python main() -> write_calibration_log()`
- 应为 `Python main() -> draw_ball()`
- 应为 `Python main() -> draw_grab_center()`
- 应为 `Python main() -> BallDetectModel`
- 应为 `Python main() -> mqtt_connect()`
- 应为 `Python main() -> mqtt_publish()`

建议下次 Graphify 报告里把这两个节点显示为：

- `STM32 main()`
- `K230 vision main()`

## 5. ESP01S_ConnectConfiguredNetwork() 推断关系是否正确

节点：`ESP01S_ConnectConfiguredNetwork()`

来源：`hardware/ESP01S.c`

结论：主要连接流程正确，但部分关系方向/置信类型仍需修正。

该函数是 ESP-01S 上电自动连接 WiFi 和 MQTT 的编排函数，源码里确实按顺序调用：

- `ESP01S_SetConnectionState()`
- `ESP01S_Test()`
- `ESP01S_Restart()`
- `ESP01S_SetEcho()`
- `ESP01S_SetStationMode()`
- `ESP01S_JoinAP()`
- `ESP01S_MQTTUserConfig()`
- `ESP01S_MQTTConnect()`
- `ESP01S_MQTTSubscribe()`
- `ESP01S_MQTTPublish()`

这些和 `docs/tongxin.md` 里的 ESP-01S 初始化流程一致：`AT`、`ATE0`、`AT+CWMODE`、`AT+CWJAP`、`AT+MQTTUSERCFG`、`AT+MQTTCONN`、订阅 `fire_car/down`。

需要修正的点：

- `ESP01S_Task()` 调用 `ESP01S_ConnectConfiguredNetwork()`，不是反过来。
- `ESP01S_SetConnectionState()` 是被 `ESP01S_ConnectConfiguredNetwork()` 多次调用，用于标记连接阶段。
- 这些直接源码调用更适合标成 `EXTRACTED`，而不是 `INFERRED`。

## 后续处理建议

短期不需要为了这些问题改业务代码。建议只处理 Graphify 输出质量：

1. 给同名函数节点加来源前缀，例如 `STM32 main()` 和 `K230 vision main()`。
2. 修正 AST 调用边方向，尤其是 C/Python 函数调用。
3. 将源码中明确出现的调用关系标为 `EXTRACTED`。
4. 调整社区标签：
   - `Carry-Ball Mission / Arm-MQTT-Vision`
   - `Project Overview / Vision Assets`

业务文档本身目前不需要修正；它们反映了真实任务闭环。
