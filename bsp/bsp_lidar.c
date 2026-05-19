/*********************************************************************************
 * @brief    激光雷达驱动使用说明 & 硬件接线 & DMA配置
 * ===============================================================================
 * MCU:STM32F103RCT6
 * 硬件接线：
 * 1. 雷达 TX   →  STM32 PA3 (USART2_RX)
 * 2. 调试RX   →  STM32 PC10 (UART4_TX)
 * 3. 所有设备共地，雷达供电5V
 * ===============================================================================
 * 使用说明：
 * 1. 主函数调用 lidar_init() 初始化
 * 2. 主循环调用 lidar_process() 处理数据
 * 3. 调用 lidar_print_points_periodic() 打印点云数据
 * 4. 串口参数：雷达230400 ，调试115200
 * ===============================================================================
 * DMA配置：
 * 1. USART2_RX：DMA1通道6，环形模式，外设→内存
 * 2. UART4_TX：DMA2通道5，普通模式，内存→外设
 * 3. 两个串口均开启全局中断
 *********************************************************************************/

#include "bsp_lidar.h"
#include "usart.h"          
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// ==================== 内部全局变量 ====================
// DMA接收缓冲区（环形缓冲区，存储雷达原始接收数据）
static uint8_t rx_dma_buf[RX_BUF_SIZE];

// 统计用变量（内部使用，外部可通过调试方式读取）
static volatile uint32_t g_lidar_rx_byte_count = 0;  // 累计接收字节数
static volatile uint32_t g_lidar_frame_ok_count = 0; // 解析成功的帧数

// 指向外部提供的点数据缓冲区的指针（由lidar_init传入）
static LidarPoint_t *g_output_points = NULL;

// 包解析缓冲区（临时存储正在解析的单帧数据）
static uint8_t lidar_frame_buf[LIDAR_PACK_LEN];
static uint16_t lidar_frame_idx = 0; // 帧解析缓冲区当前索引

// DMA接收环缓冲区指针（记录上一次处理到的位置）
static uint16_t lidar_dma_old_pos = 0;

// DMA发送忙标志（0=空闲，1=发送中）
static volatile uint8_t dma_tx_busy = 0;
static char tx_buf[512]; // DMA打印缓冲区

static uint32_t last_rx_byte_count = 0; // 上一次接收字节数（用于超时检测）
static uint32_t last_rx_time = 0;       // 上一次接收数据的时间戳（ms）
static volatile uint8_t g_frame_ready = 0;

// ==================== CRC8表（雷达帧校验专用） ====================
static const uint8_t LIDAR_CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

// ==================== 内部函数声明 ====================
static uint8_t calc_crc8(const uint8_t *data, uint32_t len);
static void parse_lidar_frame(const uint8_t *raw_data, LidarPoint_t *out_points, uint32_t *point_cnt);
static void lidar_feed_stream(const uint8_t *data, uint16_t len);
static void lidar_process_dma_stream(void);

// ==================== 内部实现 ====================
/**
 * @brief 计算数据的CRC8校验值（雷达帧校验专用）
 * @param data 待校验数据指针
 * @param len  待校验数据长度
 * @return 计算得到的CRC8值
 * @note  使用雷达协议指定的CRC8表，多项式为0x31
 */
static uint8_t calc_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc = LIDAR_CRC_TABLE[(crc ^ *data) & 0xFFu];
        data++;
    }
    return crc;
}

/**
 * @brief 解析雷达原始帧数据为点云结构
 * @param raw_data 原始帧数据指针（需长度为LIDAR_PACK_LEN）
 * @param out_points 解析后的点云输出缓冲区
 * @param point_cnt 输出：成功解析的点云数量（成功=POINT_PER_PACK，失败=0）
 * @note  1. 会校验帧头、帧长度、CRC8，任一校验失败则返回0个点
 *        2. 角度计算考虑360°循环（如结束角度<起始角度时自动补360°）
 *        3. 角度精度转换：原始0.01°单位 → 浮点数°
 */
static void parse_lidar_frame(const uint8_t *raw_data, LidarPoint_t *out_points, uint32_t *point_cnt)
{
    LidarRawFrame_t *frame = (LidarRawFrame_t*)raw_data;
    *point_cnt = 0;
    //包头包尾crc校验
    if (frame->header != 0x54) return;
    if (frame->ver_len != 0x2C) return;
    if (calc_crc8(raw_data, LIDAR_PACK_LEN - 1) != frame->crc8) return;

    uint32_t start_q = (uint32_t)frame->start_angle % LIDAR_ANGLE_UNIT36000;
    uint32_t end_q   = (uint32_t)frame->end_angle   % LIDAR_ANGLE_UNIT36000;
    uint32_t end_unwrapped = end_q;
    if (end_unwrapped < start_q) {
        end_unwrapped += LIDAR_ANGLE_UNIT36000;
    }
    const uint32_t span = end_unwrapped - start_q;
    const int denom = (POINT_PER_PACK > 1) ? (POINT_PER_PACK - 1) : 1;

    for (int i = 0; i < POINT_PER_PACK; i++) {
        uint32_t angle_q = start_q + (uint32_t)(((uint64_t)span * i) / denom);
        angle_q %= LIDAR_ANGLE_UNIT36000;
        float angle_deg = (float)angle_q / 100.0f;

        out_points[i].angle_deg   = angle_deg;
        out_points[i].distance_mm = frame->point[i].distance;
        out_points[i].intensity   = frame->point[i].intensity;
    }
    *point_cnt = POINT_PER_PACK;
}

/**
 * @brief 流式数据喂入函数（逐字节解析帧数据）
 * @param data 待解析的原始数据指针
 * @param len  待解析数据长度
 * @note  1. 按雷达帧协议逐字节拼接数据，直到凑够一帧
 *        2. 帧头为0x54，帧长度为0x2C，校验通过后调用解析函数
 *        3. 处理帧边界：若帧尾字节为0x54，直接作为下一帧的帧头，减少丢包
 *        4. 若外部缓冲区未初始化（g_output_points=NULL），直接返回
 */
static void lidar_feed_stream(const uint8_t *data, uint16_t len)
{
    // 确保输出缓冲区有效
    if (g_output_points == NULL) return;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (lidar_frame_idx == 0) {
            if (byte != 0x54) continue;
            lidar_frame_buf[lidar_frame_idx++] = byte;
            continue;
        }

        lidar_frame_buf[lidar_frame_idx++] = byte;

        if (lidar_frame_idx == 2 && lidar_frame_buf[1] != 0x2C) {
            lidar_frame_idx = 0;
            continue;
        }

        if (lidar_frame_idx >= LIDAR_PACK_LEN) {
            uint32_t point_cnt = 0;
            // 直接解析到外部提供的缓冲区
            parse_lidar_frame(lidar_frame_buf, g_output_points, &point_cnt);
            if (point_cnt == POINT_PER_PACK) {
#if LIDAR_STATS_ENABLE
                g_lidar_frame_ok_count++;
#endif
                g_frame_ready = 1u;
            }

            // 处理包边界
            if (byte == 0x54) {
                lidar_frame_buf[0] = 0x54;
                lidar_frame_idx = 1;
            } else {
                lidar_frame_idx = 0;
            }
        }
    }
}

/**
 * @brief 处理DMA接收的流式数据（环形缓冲区处理）
 * @note  1. 计算当前DMA接收位置与上一次处理位置的差值，获取新接收的数据长度
 *        2. 处理环形缓冲区的回绕（pos < old_pos时，分两段处理）
 *        3. 累计接收字节数，调用流式解析函数处理新数据
 *        4. 更新上一次处理位置，避免重复解析
 */
static void lidar_process_dma_stream(void)
{
    uint16_t pos = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (pos == lidar_dma_old_pos) return;

    uint32_t chunk_len;
    if (pos > lidar_dma_old_pos) {
        chunk_len = pos - lidar_dma_old_pos;
    } else {
        chunk_len = (RX_BUF_SIZE - lidar_dma_old_pos) + pos;
    }
#if LIDAR_STATS_ENABLE
    g_lidar_rx_byte_count += chunk_len;
#endif

    if (pos > lidar_dma_old_pos) {
        lidar_feed_stream(&rx_dma_buf[lidar_dma_old_pos], pos - lidar_dma_old_pos);
    } else {
        lidar_feed_stream(&rx_dma_buf[lidar_dma_old_pos], RX_BUF_SIZE - lidar_dma_old_pos);
        if (pos > 0) {
            lidar_feed_stream(&rx_dma_buf[0], pos);
        }
    }
    lidar_dma_old_pos = pos;
}

/**
 * @brief 打印雷达统计信息（内部调用，lidar_process中周期执行）
 * @note  1. 按LIDAR_STATS_PERIOD_MS周期计算并打印：
 *           - B/s：每秒接收字节数
 *           - FPS：每秒解析成功帧数
 *           - success：解析成功率（成功帧字节数/总接收字节数*100%）
 *        2. 基于HAL_GetTick()计时，兼容裸机环境
 *        3. 仅在LIDAR_STATS_ENABLE=1时生效
 */
void lidar_print_stats(void)
{
    static uint32_t stats_prev_bytes = 0;
    static uint32_t stats_prev_frames = 0;
    static uint32_t stats_last_tick = 0;  

    uint32_t now = HAL_GetTick();  
    if ((now - stats_last_tick) >= LIDAR_STATS_PERIOD_MS) {
        uint32_t b = g_lidar_rx_byte_count;
        uint32_t f = g_lidar_frame_ok_count;
        uint32_t dbytes = b - stats_prev_bytes;
        uint32_t dframes = f - stats_prev_frames;

        stats_prev_bytes = b;
        stats_prev_frames = f;
        stats_last_tick = now;

        uint32_t bps_est = (dbytes * 1000u) / LIDAR_STATS_PERIOD_MS;
        uint32_t fps_est = (dframes * 1000u) / LIDAR_STATS_PERIOD_MS;

        // 计算成功率：实际帧数 / 理论帧数 * 100%，避免除零
        float success_rate = 0.0f;
        if (bps_est > 0) {
            success_rate = ((float)fps_est * LIDAR_PACK_LEN / bps_est) * 100.0f;
					  //因为雷达传输速率太高，有时候有1ms的误差会导致超出100，这里矩形限幅
					  if(success_rate > 100.0f) success_rate = 100.0f;
        }

//        dma_printf("[LIDAR] B/s=%lu, FPS=%lu, success=%.1f%%\r\n",
//        bps_est, fps_est, success_rate);
    }
}

// ==================== 对外接口 ====================
void lidar_init(LidarPoint_t *out_points)
{
    if (out_points == NULL) return;
    g_output_points = out_points;
    HAL_UART_Receive_DMA(&huart2, rx_dma_buf, RX_BUF_SIZE);
}

int lidar_take_frame_ready(void)
{
    if (g_frame_ready == 0u) {
        return 0;
    }
    g_frame_ready = 0u;
    return 1;
}

void lidar_process(void)
{
    lidar_process_dma_stream();
#if LIDAR_STATS_ENABLE
    lidar_print_stats();
#endif
   
	
    // 超时检测逻辑：500ms无数据则清空点云
    uint32_t now = HAL_GetTick();  
    if (g_lidar_rx_byte_count == last_rx_byte_count) {
        // 无接收字节
        if ((now - last_rx_time) > 500) {  
            if (g_output_points != NULL) {
                memset(g_output_points, 0, POINT_PER_PACK * sizeof(LidarPoint_t));
                // 可选同时清空包解析缓冲区
                lidar_frame_idx = 0;
            }
            last_rx_byte_count = g_lidar_rx_byte_count; // 防止重复触发
        }
    } else {
        last_rx_byte_count = g_lidar_rx_byte_count;
        last_rx_time = now;
    }
}

// ==================== 回调函数（HAL库） ====================
/**
 * @brief UART DMA发送完成回调函数
 * @param huart UART句柄指针
 * @note  1. 仅处理UART4的发送完成事件
 *        2. 清空dma_tx_busy标志，允许下一次dma_printf调用
 */
void bsp_lidar_uart_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        dma_tx_busy = 0;
    }
}

// dma_printf
int dma_printf(const char *format, ...)
{
    while (dma_tx_busy); // 等待上一次发送完成
    va_list args;
    va_start(args, format);
    int len = vsnprintf(tx_buf, sizeof(tx_buf), format, args); // 格式化字符串
    va_end(args);
    if (len > 0 && len < sizeof(tx_buf)) { // 避免缓冲区溢出
        dma_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart4, (uint8_t*)tx_buf, len);
    }
    return len;
}

/**
 * @brief 周期打印激光雷达点数据（每200ms打印一次）
 * @param points 点数据指针（指向大小为 POINT_PER_PACK * LidarPoint_t 的数组）
 * @note  使用静态变量记录上次打印时间，内部自动控制打印频率
 *        函数是线程安全的（基于 HAL 毫秒级 tick 实现）
 */
void lidar_print_points_periodic(LidarPoint_t *points)
{
    static uint32_t last_print_tick = 0;
    const uint32_t print_interval_ms = 50;

    uint32_t now = HAL_GetTick();
    if ((now - last_print_tick) >= print_interval_ms) {
        last_print_tick = now;

//        dma_printf("\r\n===== Lidar Points (12) =====\r\n");
			
        for (int i = 0; i < POINT_PER_PACK; i++) {
					if(points[i].angle_deg <= 45 || points[i].angle_deg >= 315)
            dma_printf("  [%d] angle:%.2f deg, dist:%u mm, int:%u\r\n",
                       i,
                       points[i].angle_deg,
                       (unsigned)points[i].distance_mm,
                       (unsigned)points[i].intensity);
        }
//        dma_printf("============================\r\n");
    }
}

