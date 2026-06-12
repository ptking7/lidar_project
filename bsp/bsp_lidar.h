#ifndef __BSP_LIDAR_H
#define __BSP_LIDAR_H

#include "stdint.h"
#include "usart.h"

// ==================== 宏定义 ====================
#define RX_BUF_SIZE            1024        // DMA接收缓冲区大小（字节）
#define POINT_PER_PACK         12          // 每包点数
#define LIDAR_PACK_LEN         (1+1+2+2+ POINT_PER_PACK*3 +2+2+1)  // 47字节
#define LIDAR_ANGLE_UNIT36000  36000u      // 360.00度对应0.01度单位值


#define LIDAR_STATS_ENABLE      1          // 启用统计信息打印 定义为0则不打印统计信息 

#define LIDAR_STATS_PERIOD_MS   50        // 统计打印周期（毫秒）


// ==================== 数据结构 ====================
// 原始点数据结构（对应通讯协议）
typedef struct __attribute__((packed)) {
    uint16_t distance;   // 距离(mm)
    uint8_t  intensity;  // 强度
} LidarRawPoint_t;

// 原始包结构
typedef struct __attribute__((packed)) {
    uint8_t  header;          // 0x54
    uint8_t  ver_len;         // 0x2C
    uint16_t speed;
    uint16_t start_angle;
    LidarRawPoint_t point[POINT_PER_PACK];    //点云数据
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t  crc8;
} LidarRawFrame_t;

// 对外输出的点结构（带角度）
typedef struct {
    float    angle_deg;      // 角度（度）
    uint16_t distance_mm;    // 距离（毫米）
    uint8_t  intensity;
} LidarPoint_t;

// ==================== 接口函数（只需关注这些函数） ====================
/**
 * @brief 初始化激光雷达模块（启用DMA接收）
 * @param out_points 外部提供的点数据缓冲区指针（大小为 POINT_PER_PACK * LidarPoint_t）
 */
void lidar_init(LidarPoint_t *out_points);

void lidar_process(void);
/** 有新点云帧解析成功时返回 1（读后清除），供上层检测模块使用 */
int lidar_take_frame_ready(void);
int dma_printf(const char *format, ...);
//循环打印统计信息
void lidar_print_stats(void);
//周期打印点云(200ms打印一次，避免刷屏)
void lidar_print_points_periodic(LidarPoint_t *points);
void bsp_lidar_uart_tx_cplt(UART_HandleTypeDef *huart);
#endif // __BSP_LIDAR_H
