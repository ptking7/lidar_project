# lidar_project

基于 STM32F103RCT6 的激光雷达原始数据接收与解析工程。通过 DMA 循环接收雷达串口数据，完成帧同步、CRC8 校验与点云解析，并通过调试串口输出统计信息。

## 硬件平台

| 项目 | 说明 |
|------|------|
| MCU | STM32F103RCT6 |
| 雷达数据接收 | 雷达 TX → STM32 **PA3**（USART2_RX） |
| 调试输出 | STM32 **PC10**（UART4_TX）→ 串口 RX |
| 供电 | 雷达 5V 独立供电 |

## 串口配置

| 串口 | 波特率 | 用途 |
|------|--------|------|
| USART2 | 230400 | 接收雷达原始数据 |
| UART4 | 115200 | 调试打印输出 |

## DMA 配置

- **USART2_RX**：DMA1 通道 6，循环模式，外设到内存
- **UART4_TX**：DMA2 通道 5，普通模式，内存到外设
- 串口接收采用 DMA，不依赖逐字节中断

## 项目结构

```
lidar_project/
├── bsp/                  # 激光雷达 BSP 驱动
│   ├── bsp_lidar.c
│   └── bsp_lidar.h
├── Core/                 # STM32CubeMX 生成的核心代码
├── Drivers/              # STM32 HAL / CMSIS 驱动
├── MDK-ARM/              # Keil MDK 工程文件
│   └── 10_lidir_hal.uvprojx
└── README.md
```

## 数据协议

每帧固定 **47 字节**，每包包含 **12 个点**：

| 字段 | 说明 |
|------|------|
| header | 帧头 `0x54` |
| ver_len | 版本/长度 `0x2C` |
| speed | 转速 |
| start_angle / end_angle | 起止角度（0.01° 单位） |
| point[12] | 距离（mm）+ 强度 |
| timestamp | 时间戳 |
| crc8 | CRC8 校验（多项式 0x31） |

解析后的点云结构 `LidarPoint_t` 包含：角度（度）、距离（mm）、强度。

## 快速使用

1. 使用 Keil MDK 打开 `MDK-ARM/10_lidir_hal.uvprojx`
2. 编译并下载到 STM32F103RCT6 开发板
3. 连接雷达与调试串口，雷达侧波特率设为 **230400**
4. 上电后，调试串口（115200）将周期性输出接收统计，例如：

```
[LIDAR] B/s=xxx, FPS=xxx, success=xx.x%
```

### 应用层调用示例

```c
static LidarPoint_t lidar_output_points[POINT_PER_PACK];

// 初始化（传入点云输出缓冲区）
lidar_init(lidar_output_points);

while (1) {
    lidar_process();                              // 处理 DMA 数据、解析帧、打印统计
    // lidar_print_points_periodic(lidar_output_points);  // 可选：每 200ms 打印点云
}
```

## 主要 API

| 函数 | 说明 |
|------|------|
| `lidar_init()` | 初始化模块，启动 USART2 DMA 接收 |
| `lidar_process()` | 主循环调用，处理数据流与超时检测 |
| `lidar_print_stats()` | 周期性打印接收速率、帧率、成功率 |
| `lidar_print_points_periodic()` | 周期性打印 12 个点云数据 |
| `dma_printf()` | 通过 UART4 DMA 非阻塞打印 |

## 分支说明

| 分支 | 说明 |
|------|------|
| `main` | 主分支，原始激光雷达数据接收与处理 |
| `avoid_communication` | 避障通信相关功能分支 |

## 开发环境

- Keil MDK-ARM
- STM32CubeMX / STM32 HAL 库
- ST-Link 或其他 SWD 下载器

## 仓库地址

https://github.com/ptking7/lidar_project
