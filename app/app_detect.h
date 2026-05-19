#ifndef __APP_DETECT_H
#define __APP_DETECT_H

#include "stdint.h"
#include "bsp_lidar.h"

/* ==================== 检测扇区（30° 正前方） ====================
 * 雷达 0° 为正前方，本配置覆盖 0°~15° 与 345°~360°（合计 30°）
 */
#define APP_DETECT_ANGLE_LEFT_MAX_DEG    15.0f
#define APP_DETECT_ANGLE_RIGHT_MIN_DEG   345.0f

/* ==================== 距离阈值（mm） ====================
 * STL-19P：量程 0.1~12 m，角分辨率约 0.8°，2~8 m 测距精度约 ±20 mm。
 * 0.25 m×0.25 m 目标在 7 m 处张角约 2°，每圈仅约 2~3 点，小目标远距离易漏检。
 * DETECT_DIST_MAX：参与聚类的有效量程（建议 4~5 m 内较稳）。
 * COMM_*：与四轮车协议 V1.0 一致的对外预警/停车阈值。
 */
#define APP_DETECT_DIST_MIN_MM           100u
#define APP_DETECT_DIST_MAX_MM           5000u

#define APP_COMM_WARN_DIST_MM            7000u
#define APP_COMM_STOP_DIST_MM            5000u

/* 小目标确认：扇区内至少命中点数、相邻点最大角度差（度） */
#define APP_DETECT_MIN_HIT_POINTS        2u
#define APP_DETECT_CLUSTER_MAX_ANGLE_DEG 3.0f
#define APP_DETECT_MIN_INTENSITY         15u

/* 连续若干帧未检出则清除障碍状态（约 10 帧雷达包） */
#define APP_DETECT_MISS_FRAMES_MAX       10u

/* 兜底：长时间无任何成功检出则强制清除 */
#define APP_DETECT_SCAN_HOLD_MS          500u

typedef struct {
    uint8_t  valid;          /* 1=当前扇区内确认有障碍物 */
    uint16_t dist_mm;        /* 扇区内最近障碍距离，无障碍为 0xFFFF */
    uint8_t  hit_points;     /* 本帧扇区内有效命中数（调试） */
} AppDetectResult_t;

void app_detect_init(void);
void app_detect_reset(void);
void app_detect_feed(const LidarPoint_t *points, uint32_t point_count);
void app_detect_process(void);
const AppDetectResult_t *app_detect_get_result(void);

/** 按协议 V1.0 将距离映射为 warn_level（0/1/2） */
uint8_t app_detect_to_warn_level(uint16_t dist_mm, uint8_t obstacle_valid);

#endif /* __APP_DETECT_H */
