#include "app_communicat.h"
#include "usart.h"
#include "main.h"
#include <string.h>

// 串口接收状态机枚举
typedef enum {
    RX_WAIT_H1 = 0,     // 等待帧头第一个字节
    RX_WAIT_H2,         // 等待帧头第二个字节
    RX_WAIT_LEN,        // 等待数据长度字节
    RX_RECV_DATA        // 接收数据+CRC校验码
} AppCommRxState_t;

static AppCommVehicleState_t g_vehicle;         // 下位机车辆状态数据
static AppCommRxState_t g_rx_state;             // 接收状态机状态
static uint8_t g_rx_len;                        // 帧内有效数据长度
static uint8_t g_rx_buf[18u];                   // 单帧接收缓存
static uint8_t g_rx_idx;                        // 接收缓存索引

static uint8_t  g_rx_dma_buf[APP_COMM_RX_DMA_BUF_SIZE]; // DMA循环接收缓冲区
static uint16_t g_rx_dma_old_pos;               // 上一次DMA读取位置

static uint8_t  g_tx_buf[5u + APP_COMM_PAYLOAD_LEN];    // 串口发送帧缓存
static volatile uint8_t g_tx_busy;              // 发送忙标志，防止并发发送
static uint32_t g_last_tx_tick;                 // 上一次发送时间戳
static uint8_t  g_last_warn_level;              // 上一次告警等级
static uint16_t g_last_dist_mm;                 // 上一次障碍物距离

// MODBUS CRC16 预计算查表
static const uint16_t s_crc16_modbus_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8580, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8080, 0x4040
};

/**
 * @brief MODBUS CRC16 校验计算
 * @param buf 待校验数据缓冲区
 * @param len 数据长度
 * @return 计算得到的CRC16值
 */
static uint16_t crc16_modbus(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)((crc ^ buf[i]) & 0xFFu);
        crc = (uint16_t)((crc >> 8) ^ s_crc16_modbus_table[idx]);
    }
    return crc;
}

/**
 * @brief 组装并发送告警+心跳数据帧
 * @param warn_level 告警等级
 * @param dist_mm    障碍物距离
 * @param heartbeat  心跳标识位
 */
static void comm_send_warn_frame(uint8_t warn_level, uint16_t dist_mm, uint8_t heartbeat)
{
    if (g_tx_busy != 0u) {  // 发送中直接退出
        return;
    }

    uint8_t payload[APP_COMM_PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    // 填充有效载荷：告警等级、距离低字节、距离高字节、心跳位
    payload[0] = warn_level;
    payload[1] = (uint8_t)(dist_mm & 0xFFu);
    payload[2] = (uint8_t)((dist_mm >> 8) & 0xFFu);
    payload[APP_COMM_PAYLOAD_LEN - 1u] = heartbeat;

    // 拼装帧头 + 长度 + 载荷
    g_tx_buf[0] = APP_COMM_FRAME_HEAD0;
    g_tx_buf[1] = APP_COMM_FRAME_HEAD1;
    g_tx_buf[2] = APP_COMM_PAYLOAD_LEN;
    memcpy(&g_tx_buf[3], payload, APP_COMM_PAYLOAD_LEN);

    // 计算并追加CRC校验码
    uint16_t crc = crc16_modbus(&g_tx_buf[2], 1u + APP_COMM_PAYLOAD_LEN);
    g_tx_buf[3u + APP_COMM_PAYLOAD_LEN]     = (uint8_t)(crc & 0xFFu);
    g_tx_buf[3u + APP_COMM_PAYLOAD_LEN + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    g_tx_busy = 1u;
    // 优先使用DMA发送，失败则切换阻塞发送
    if (HAL_UART_Transmit_DMA(&huart4, g_tx_buf,
                              (uint16_t)(5u + APP_COMM_PAYLOAD_LEN)) != HAL_OK) {
        g_tx_busy = 0u;
        HAL_UART_Transmit(&huart4, g_tx_buf, (uint16_t)(5u + APP_COMM_PAYLOAD_LEN), 20u);
    }
}

/**
 * @brief 解析接收的车辆状态载荷数据
 * @param payload 载荷数据指针
 * @param len     载荷长度
 */
static void comm_on_vehicle_payload(const uint8_t *payload, uint8_t len)
{
    if (len < APP_COMM_PAYLOAD_LEN) {
        return;
    }
    // 解析速度、心跳，刷新通讯状态
    g_vehicle.vx_mm_s = (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
    g_vehicle.heartbeat = payload[APP_COMM_PAYLOAD_LEN - 1u];
    g_vehicle.last_rx_tick = HAL_GetTick();
    g_vehicle.link_ok = 1u;
}

/**
 * @brief 单字节接收状态机处理
 * @param byte 单字节接收数据
 */
static void comm_rx_feed_byte(uint8_t byte)
{
    switch (g_rx_state) {
    case RX_WAIT_H1:
        if (byte == APP_COMM_FRAME_HEAD0) {
            g_rx_state = RX_WAIT_H2;
        }
        break;

    case RX_WAIT_H2:
        if (byte == APP_COMM_FRAME_HEAD1) {
            g_rx_state = RX_WAIT_LEN;
        } else if (byte == APP_COMM_FRAME_HEAD0) {
            g_rx_state = RX_WAIT_H2;
        } else {
            g_rx_state = RX_WAIT_H1;
        }
        break;

    case RX_WAIT_LEN:
        if (byte > 16u) {  // 长度超限，重置状态
            g_rx_state = RX_WAIT_H1;
            break;
        }
        g_rx_len = byte;
        g_rx_idx = 0u;
        g_rx_state = RX_RECV_DATA;
        break;

    case RX_RECV_DATA:
        // 存入数据与CRC字节
        if (g_rx_idx < (g_rx_len + 2u)) {
            g_rx_buf[g_rx_idx++] = byte;
        }
        // 一帧接收完成，开始校验
        if (g_rx_idx < (g_rx_len + 2u)) {
            break;
        }

        {
            uint8_t crc_buf[1u + 16u];
            crc_buf[0] = g_rx_len;
            if (g_rx_len > 0u) {
                memcpy(&crc_buf[1], g_rx_buf, g_rx_len);
            }
            // 本地计算CRC
            uint16_t calc = crc16_modbus(crc_buf, 1u + g_rx_len);
            // 提取接收的CRC
            uint16_t recv = (uint16_t)g_rx_buf[g_rx_len] | ((uint16_t)g_rx_buf[g_rx_len + 1u] << 8);
            // CRC校验通过则解析数据
            if (calc == recv) {
                comm_on_vehicle_payload(g_rx_buf, g_rx_len);
            }
        }
        g_rx_state = RX_WAIT_H1; // 回到初始状态，等待下一帧
        break;

    default:
        g_rx_state = RX_WAIT_H1;
        break;
    }
}

/**
 * @brief 批量送入接收数据流
 * @param data 数据缓冲区
 * @param len  数据长度
 */
static void comm_rx_feed_stream(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        comm_rx_feed_byte(data[i]);
    }
}

/**
 * @brief 处理DMA循环接收数据，解析新接收内容
 */
static void comm_process_dma_rx(void)
{
    // 计算当前DMA接收位置
    uint16_t pos = APP_COMM_RX_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(huart4.hdmarx);
    if (pos == g_rx_dma_old_pos) {  // 无新数据直接返回
        return;
    }

    // 分两种情况处理循环缓冲区：未绕回 / 已绕回
    if (pos > g_rx_dma_old_pos) {
        comm_rx_feed_stream(&g_rx_dma_buf[g_rx_dma_old_pos], pos - g_rx_dma_old_pos);
    } else {
        comm_rx_feed_stream(&g_rx_dma_buf[g_rx_dma_old_pos], APP_COMM_RX_DMA_BUF_SIZE - g_rx_dma_old_pos);
        if (pos > 0u) {
            comm_rx_feed_stream(&g_rx_dma_buf[0], pos);
        }
    }
    g_rx_dma_old_pos = pos; // 更新上一次位置
}

/**
 * @brief 通讯模块初始化
 */
void app_comm_init(void)
{
    memset(&g_vehicle, 0, sizeof(g_vehicle));
    g_rx_state = RX_WAIT_H1;
    g_tx_busy = 0u;
    g_last_warn_level = 0xFFu;
    g_last_dist_mm = 0xFFFFu;
    g_last_tx_tick = 0u;
    g_rx_dma_old_pos = 0u;

    // 开启串口DMA循环接收
    HAL_UART_Receive_DMA(&huart4, g_rx_dma_buf, APP_COMM_RX_DMA_BUF_SIZE);
}

/**
 * @brief 通讯主循环处理函数，周期调用
 */
void app_comm_process(void)
{
    // 处理DMA接收数据
    comm_process_dma_rx();

    const AppDetectResult_t *det = app_detect_get_result();
    uint32_t now = HAL_GetTick();

    // 通讯超时判断
    if ((now - g_vehicle.last_rx_tick) > APP_COMM_LINK_TIMEOUT_MS) {
        g_vehicle.link_ok = 0u;
    }

    // 获取障碍物距离与告警等级
    uint16_t dist_mm = det->valid ? det->dist_mm : 0xFFFFu;
    uint8_t warn = app_detect_to_warn_level(dist_mm, det->valid);

    // 组装心跳位
    uint8_t heartbeat = 0x01u;
    if (det->valid != 0u) {
        heartbeat |= 0x02u;
    }

    // 判断是否需要发送：定时到 或 状态发生变化
    int need_tx = 0;
    if ((now - g_last_tx_tick) >= APP_COMM_TX_PERIOD_MS) {
        need_tx = 1;
    }
    if (warn != g_last_warn_level || dist_mm != g_last_dist_mm) {
        need_tx = 1;
    }

    if (need_tx != 0) {
        comm_send_warn_frame(warn, dist_mm, heartbeat);
        g_last_tx_tick = now;
        g_last_warn_level = warn;
        g_last_dist_mm = dist_mm;
    }
}

/**
 * @brief 获取解析后的车辆状态数据
 * @return 车辆状态结构体指针
 */
const AppCommVehicleState_t *app_comm_get_vehicle_state(void)
{
    return &g_vehicle;
}

/**
 * @brief UART发送完成中断回调
 * @param huart 串口句柄
 */
void app_comm_uart_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        g_tx_busy = 0u; // 清除发送忙标志
    }
}
