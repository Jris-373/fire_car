# Repository Guidelines

## Project Structure & Module Organization

This is an STM32F407 firmware project generated with STM32CubeMX and built with Keil MDK. `fire_car.ioc` is the CubeMX configuration source. `Core/Inc` and `Core/Src` contain generated HAL setup plus user application entry points such as `main.c` and `freertos.c`. Custom board logic lives in `hardware/`, including motor, BLE, PID, map, NVIC, and JY61P modules. `Drivers/` contains STM32 HAL and CMSIS vendor code; `Middlewares/Third_Party/FreeRTOS/` contains FreeRTOS sources. `MDK-ARM/` contains the Keil project, startup file, debug configuration, and generated build outputs.

## Build, Test, and Development Commands

- Open CubeMX configuration: `"D:\stm32-mx\STM32CubeMX.exe" fire_car.ioc`
- Open the Keil project: `"F:\KEIL_MDK\UV4\UV4.exe" MDK-ARM\fire_car.uvprojx`
- Command-line build: `"F:\KEIL_MDK\UV4\UV4.exe" -b MDK-ARM\fire_car.uvprojx -t fire_car -j0 -o MDK-ARM\build.log`

Generated artifacts such as `.o`, `.d`, `.crf`, `.map`, `.axf`, `.hex`, and build logs are produced under `MDK-ARM/fire_car/`.

## Coding Style & Naming Conventions

Use C with the existing STM32Cube style. Keep two-space indentation in generated-style blocks and brace placement consistent with neighboring files. Public module functions use `Module_Action` naming, for example `Motor_HeadingTask`; private helpers should be `static`. Global firmware state uses a `g_` prefix, and constants/macros use uppercase names. Preserve `/* USER CODE BEGIN ... */` sections in CubeMX-managed files so regeneration does not erase custom code.

## Testing Guidelines

There is no host-side unit test framework in this repository. Validate changes by building the Keil target and, when hardware behavior changes, flashing the STM32F407 board and testing the affected peripherals or FreeRTOS tasks. For motor, sensor, or BLE work, record the tested scenario and expected telemetry or behavior in the PR notes.

## Commit & Pull Request Guidelines

No Git history is present in this workspace, so use a simple imperative commit style such as `motor: clamp heading correction` or `ble: add packet timeout`. Pull requests should describe the firmware behavior changed, list hardware tested, include build status, and mention any CubeMX regeneration. Avoid mixing generated/vendor updates with application logic unless the change requires it.

## Agent-Specific Instructions

Prefer editing `hardware/` and `Core/*` user sections. Do not reformat `Drivers/`, `Middlewares/`, or broad CubeMX output while making focused firmware changes.

## Project Memory

- Before broad codebase analysis, read `graphify-out/GRAPH_REPORT.md`.
- For the latest manual review of Graphify suggested questions, read `docs/graphify_questions_review.md`.
- When locating cross-module relationships, query Graphify before doing broad text search.
- Use `rg` to verify exact file paths and implementation details after Graphify suggests likely modules.
- If code structure changes substantially, update Graphify through the local graphify skill/pipeline. The installed `graphify.exe` may not accept `graphify . --update` directly in PowerShell.
- Treat Graphify call edges as leads, not proof: the current AST extraction can reverse some `calls` edges and mark direct source calls as `INFERRED`.
- Disambiguate same-label nodes by source file. In this project, `main()` can mean STM32 `Core/Src/main.c` or the K230 script `tools/vision/ball_center_calibration_k230_mqtt.py`.
