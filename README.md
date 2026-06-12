# 雷达控制器 HAL 固件（STM32F103）

基于 STM32F103 的激光雷达避障控制器：采集雷达点云、检测车头正前方障碍物，并通过 UART 与四轮车运动平台交换预警与速度信息。

完整协议说明见仓库内文档：[雷达控制器与四轮车串口通信协议_V1.0.docx](./雷达控制器与四轮车串口通信协议_V1.0.docx)

## 功能概述

| 模块 | 说明 |
|------|------|
| 雷达采集 | USART2 + DMA 循环接收，解析标准点云包（0x54 帧头） |
| 障碍检测 | 正前方走廊内点云聚类，多帧确认后输出最近障碍距离 |
| 预警分级 | 按距离映射 `warn_level`：0 无障碍 / 1 预警 / 2 急停 |
| 车辆通信 | UART4 双向帧通信，CRC16/MODBUS 校验 |

### 检测逻辑（固件实现）

- **检测范围**：约 0.1 m ~ 9 m；9 m 内走廊总宽约 2 m（中心左右各 1 m，远处随距离变窄，半角上限约 30°）
- **目标判定**：约 0.25 m 目标宽度对应的聚类阈值；一般需 ≥2 个有效点，极近距（≤0.8 m）单点也可确认
- **时序**：200 ms 滑动窗口累计点云；连续 3 帧确认后上报；连续 5 帧丢失或 500 ms 无更新则清除
- **告警阈值**（与协议 V1.0 一致）：
  - `dist_mm > 7000` → `warn_level = 0`
  - `5000 < dist_mm ≤ 7000` → `warn_level = 1`（预警）
  - `dist_mm ≤ 5000` → `warn_level = 2`（急停）

STM32 **只上报**预警等级与距离，不直接控制车辆；停/减速/恢复由四轮车主控实现。

## 硬件与接线

| 项目 | 配置 |
|------|------|
| MCU | STM32F103RCT6（LQFP64） |
| 雷达串口 | USART2，115200 8N1，PA2(TX) / PA3(RX) |
| 车辆串口 | UART4，115200 8N1，PC10(TX) / PC11(RX) |

### 与四轮车 / PC 仿真器接线（TTL）

```
四轮车/PC TX  ──? STM32 PC11 (UART4_RX)
四轮车/PC RX  ?── STM32 PC10 (UART4_TX)
GND           ─── GND
```

雷达模块 TX → PA3，雷达 RX ← PA2（以实际模块为准）。

## 工程结构

```
├── Core/                 # CubeMX 生成的主程序与外设
├── app/
│   ├── app_detect.c/h    # 障碍检测与 warn_level 映射
│   └── app_communicat.c/h# 与四轮车 UART 协议收发
├── bsp/
│   └── bsp_lidar.c/h     # 雷达 DMA 接收与点云解析
├── tools/
│   └── vehicle_comm_sim.py  # PC 端四轮车协议仿真脚本
├── 10_lidir_hal.ioc      # STM32CubeMX 工程文件
└── 雷达控制器与四轮车串口通信协议_V1.0.docx
```

## 通信协议摘要（V1.0）

### 帧格式

无 CMD/SEQ 字段；帧头之后直接为长度与数据。

```
| 0xAA | 0x55 | LEN | DATA[0..N-1] | CRC_L | CRC_H |
```

- **总长度**：`5 + N` 字节
- **N**：当前固定为 **8**（LEN = 0x08）
- **字节序**：多字节数值为小端（Little-Endian）
- **CRC**：CRC16/MODBUS，校验范围 `[LEN, DATA...]`（不含帧头）

### STM32 → 四轮车（约 20 Hz，状态变化时立即发送）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | warn_level | uint8 | 0=无障碍，1=预警，2=急停 |
| 1~2 | dist_mm | uint16 | 最近障碍距离（mm）；无障碍为 0xFFFF |
| 3~6 | 保留 | — | 默认 0x00 |
| 7 | HEARTBEAT | uint8 | bit0=雷达工作；bit1=有确认障碍 |

### 四轮车 → STM32（建议 20~50 Hz）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0~1 | vx_mm_s | int16 | 前进速度（**mm/s**，小端；负值表示后退） |
| 2~6 | 保留 | — | 默认 0x00 |
| 7 | HEARTBEAT | uint8 | 链路/状态字节（双方可协商位定义） |

> 协议文档中速度字段写作 m/s；当前固件与仿真脚本统一使用 **mm/s**（int16），对接时请与对端确认单位。


### 链路超时

- 500 ms 未收到对端合法帧：STM32 侧认为车辆链路断开（`link_ok = 0`），仍按本地检测结果上报预警

## 编译与烧录

1. 使用 **STM32CubeIDE** 或 **Keil MDK** 打开工程（CubeMX 文件：`10_lidir_hal.ioc`）
2. 固件包：STM32Cube FW_F1 V1.8.7
3. 编译后通过 ST-Link / J-Link 下载到目标板

主循环逻辑（`Core/Src/main.c`）：

```c
lidar_process();
if (lidar_take_frame_ready()) {
    app_detect_feed(lidar_output_points, POINT_PER_PACK);
}
app_detect_process();
app_comm_process();
```

## PC 端仿真脚本

目录：`tools/vehicle_comm_sim.py`

在 PC 上模拟**四轮车主控**：向 STM32 周期性发送速度帧，并解析 STM32 上报的预警/距离帧。用于联调 UART4 通信，无需真实车辆。

### 依赖

```bash
pip install pyserial
```

### 用法

```bash
# 基本：指定串口，默认交替发送 10 / 15 mm/s
python tools/vehicle_comm_sim.py -p COM13

# 固定速度 800 mm/s，发送频率 25 Hz
python tools/vehicle_comm_sim.py
# 自定义交替速度与切换间隔
python tools/vehicle_comm_sim.py 
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-p, --port` | （必填） | 串口名，如 `COM13` |
| `-b, --baud` | 115200 | 波特率 |
| `--speed` | — | 固定速度（mm/s）；指定后忽略 `--alternate` |
| `--alternate V1 V2` | 10 15 | 两档速度交替（mm/s） |
| `--switch-sec` | 3.0 | 交替模式下切换间隔（秒） |
| `--tx-hz` | 20.0 | 向 STM32 发送速度帧频率（Hz） |
| `--heartbeat` | 0x05 | 速度帧 HEARTBEAT 字节 |

### 输出示例

```
[RX #0001] warn=0 clear (0)  dist=no obstacle (0xFFFF)  heartbeat=0x01
[RX #0042] warn=1 warn  (1)  dist=6500 mm (6.50 m)      heartbeat=0x03
[RX #0058] warn=2 stop  (2)  dist=4000 mm (4.00 m)      heartbeat=0x03
```

脚本内置与固件相同的帧解析状态机（帧头 0xAA 0x55 → LEN → DATA → CRC）及 CRC16/MODBUS 校验。

## 分支说明

当前开发分支：`avoid_communication`  
远程仓库：`https://github.com/ptking7/lidar_project.git`

## 参考

- 协议全文：[雷达控制器与四轮车串口通信协议_V1.0.docx](./雷达控制器与四轮车串口通信协议_V1.0.docx)
- 检测参数宏定义：`app/app_detect.h`
- 通信参数宏定义：`app/app_communicat.h`
