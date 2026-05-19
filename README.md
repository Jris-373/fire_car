# fire_car

版本：**v1**

## 项目背景

`fire_car` 是一个基于 STM32F407 的智能小车固件项目，由 STM32CubeMX 生成外设初始化代码，并使用 Keil MDK 构建。项目运行在 FreeRTOS 任务模型上，面向电机闭环控制、IMU 航向反馈、BLE 指令与遥测通信，以及栅格地图探索等嵌入式小车控制场景。

当前 `v1` 版本作为 GitHub 初始发布基线。地图探索状态仅保存在 RAM 中；此前规划的外部 W25Q64 Flash 地图持久化功能已经废弃，并已从代码中移除。

## 主要功能

- STM32F407 HAL 外设初始化，配置来源于 STM32CubeMX。
- FreeRTOS 多任务调度，负责应用层控制逻辑。
- BLE 串口接收速度指令，并周期性输出速度遥测数据。
- 通过 I2C 读取 JY61P IMU 的加速度与角度数据。
- 使用 TIM4 PWM、TIM3/TIM8 编码器和 PID 实现双电机速度闭环。
- 基于 IMU yaw 角反馈进行航向稳定和差速修正。
- RAM 版栅格地图、DFS 探索和回溯辅助逻辑。

## 项目结构

- `fire_car.ioc`：STM32CubeMX 配置源文件。
- `Core/Inc`、`Core/Src`：CubeMX 生成的 HAL 初始化代码和应用入口。
- `hardware/`：自定义硬件与业务模块，包括 BLE、JY61P、motor、PID、map、NVIC。
- `Drivers/`：STM32 HAL 与 CMSIS 厂商代码。
- `Middlewares/Third_Party/FreeRTOS/`：FreeRTOS 源码。
- `MDK-ARM/`：Keil MDK 工程文件、启动文件和调试配置。

## 构建方式

打开 CubeMX 配置：

```powershell
D:\stm32-mx\STM32CubeMX.exe fire_car.ioc
```

打开 Keil 工程：

```powershell
F:\KEIL_MDK\UV4\UV4.exe MDK-ARM\fire_car.uvprojx
```

命令行构建：

```powershell
F:\KEIL_MDK\UV4\UV4.exe -b MDK-ARM\fire_car.uvprojx -t fire_car -j0
```

当前 `v1` 基线已通过 Keil MDK 构建验证：`0 Error(s), 0 Warning(s)`。

## 版本记录

### v1

- 建立 GitHub 初始发布版本。
- 补充项目背景、目录结构和构建说明。
- 移除已废弃的外部 W25Q64 Flash 地图持久化功能。
- 保留 RAM 中的地图探索状态和 DFS 辅助逻辑。

## 说明

`MDK-ARM/fire_car/` 下的 `.o`、`.d`、`.crf`、`.map`、`.axf`、`.hex` 和构建日志属于本地生成产物，已通过 `.gitignore` 忽略。需要时可在本地使用 Keil MDK 重新构建生成。
