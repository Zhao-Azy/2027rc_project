# 2027rc_project

河北工程大学 **UPRE 战队** RoboCon 2027 赛季电控代码仓库。

## 仓库内容

本仓库只收录**固件源码**，位于 `dm_demo/`：

| 目录 | 说明 |
| --- | --- |
| `dm_demo/Core/` | STM32CubeMX 生成的 HAL 初始化与中断代码 |
| `dm_demo/app/` | 应用层：整车状态机、控制逻辑 |
| `dm_demo/bsp/` | 板级驱动：底盘、舵向、机械臂、电机、IMU 等 |
| `dm_demo/Middlewares/` | FreeRTOS / CMSIS-OS v2 |
| `dm_demo/Drivers/` | STM32H7 HAL 与 CMSIS |
| `dm_demo/MDK-ARM/` | Keil MDK 工程文件（`R1.uvprojx` 为入口工程） |

## 开发环境

- **MCU**：STM32H723VGTx
- **RTOS**：FreeRTOS（CMSIS-OS v2）
- **IDE**：Keil MDK-ARM（打开 `dm_demo/MDK-ARM/R1.uvprojx`）
- **配置工具**：STM32CubeMX（`dm_demo/R1.ioc`）

## 构建

用 Keil uVision 打开 `dm_demo/MDK-ARM/R1.uvprojx` 后编译。
编译输出到 `MDK-ARM/R1/`，该目录已被 `.gitignore` 排除，不会污染仓库。

## 未纳入版本管理的内容

以下内容保留在本地，不随仓库分发：

- `HI14/` —— IMU 厂家上位机与数据手册（含 Qt 运行库，体积较大）
- `原理图.pdf` / `说明书.pdf` / `ROBOCON_电控入队手册.docx` —— 纸质资料电子版
- `tools/` —— 一次性的 SBUS / 姿态数据分析脚本
- `vofa+.csv` / `yaw_log.txt` —— 串口调试导出数据
- `MDK-ARM/R1/` —— Keil 编译产物

## 提交规范

提交信息使用中文或英文均可，建议格式：

```
<模块>: <做了什么>

例如：
chassis: 修复舵向零位标定偏移
fdcan: 增加电机掉线重连保护
```
