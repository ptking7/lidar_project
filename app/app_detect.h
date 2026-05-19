#ifndef __APP_DETECT_H
#define __APP_DETECT_H

#include "stdint.h"
#include "bsp_lidar.h"

/* ==================== 正前方走廊（随距离变窄） ====================
 * 9 m 内保持约 2 m 总宽（中心线左右各 CORRIDOR_HALF_WIDTH_MM）。
 * 半角 ≈ atan(1000/dist_mm)，远处窄、近处宽。
 */
#define APP_DETECT_CORRIDOR_HALF_WIDTH_MM  1000u   /* 总宽 2 m */
#define APP_DETECT_CORRIDOR_MAX_HALF_DEG   30.0f   /* 近距半角上限 */
#define APP_DETECT_RANGE_MAX_MM            9000u   /* 走廊/检测最远距离 9 m */

/* ==================== 距离阈值（mm） ====================
 * COMM_*：与四轮车协议 V1.0 一致，通信层不改。
 */
#define APP_DETECT_DIST_MIN_MM           100u
#define APP_DETECT_DIST_MAX_MM           APP_DETECT_RANGE_MAX_MM

#define APP_COMM_WARN_DIST_MM            7000u
#define APP_COMM_STOP_DIST_MM            5000u

/* 0.25 m 目标宽度：用于按距离计算聚类角度（近大远小） */
#define APP_DETECT_TARGET_WIDTH_MM       250u
#define APP_DETECT_CLUSTER_ANGLE_MIN_DEG 2.0f
#define APP_DETECT_CLUSTER_ANGLE_MAX_DEG 8.0f
#define APP_DETECT_CLUSTER_DD_MIN_MM     200u
#define APP_DETECT_CLUSTER_DD_MAX_MM     500u
#define APP_DETECT_CLUSTER_DD_RATIO_PCT  5u      /* dd_max = dist * 5% */

#define APP_DETECT_MIN_HIT_POINTS        2u
#define APP_DETECT_MIN_INTENSITY         15u
#define APP_DETECT_NEAR_SINGLE_MAX_MM    800u    /* 仅 1 点且很近时直接确认 */

/* 滑动窗口：跨 UART 包累计走廊内点再聚类 */
#define APP_DETECT_WINDOW_MS             200u
#define APP_DETECT_BUF_MAX               64u

/* 多帧确认 / 清除；输出距离滤波 */
#define APP_DETECT_CONFIRM_FRAMES        3u
#define APP_DETECT_CLEAR_FRAMES          5u
#define APP_DETECT_DIST_JUMP_MM          400u   /* 单帧距离跳变抑制 */
#define APP_DETECT_SCAN_HOLD_MS          500u

typedef struct {
    uint8_t  valid;          /* 1=确认有障碍物 */
    uint16_t dist_mm;        /* 走廊内最近障碍距离，无障碍为 0xFFFF */
    uint8_t  hit_points;     /* 窗口内参与确认的点数（调试） */
} AppDetectResult_t;

void app_detect_init(void);
void app_detect_reset(void);
void app_detect_feed(const LidarPoint_t *points, uint32_t point_count);
void app_detect_process(void);
const AppDetectResult_t *app_detect_get_result(void);

/** 按协议 V1.0 将距离映射为 warn_level（0/1/2） */
uint8_t app_detect_to_warn_level(uint16_t dist_mm, uint8_t obstacle_valid);

#endif /* __APP_DETECT_H */
