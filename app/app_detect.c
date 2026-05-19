#include "app_detect.h"
#include "main.h"
#include <string.h>

static AppDetectResult_t g_result;
static uint32_t g_last_update_tick;

static int angle_in_detect_sector(float angle_deg)
{
    return (angle_deg <= APP_DETECT_ANGLE_LEFT_MAX_DEG) ||
           (angle_deg >= APP_DETECT_ANGLE_RIGHT_MIN_DEG);
}

static int point_is_valid(const LidarPoint_t *p)
{
    if (p->distance_mm < APP_DETECT_DIST_MIN_MM) {
        return 0;
    }
    if (p->distance_mm > APP_DETECT_DIST_MAX_MM) {
        return 0;
    }
    if (p->intensity < APP_DETECT_MIN_INTENSITY) {
        return 0;
    }
    return 1;
}

/* 单帧内：在扇区有效点�?找�?�度相邻簇，要求簇内点数 >= MIN_HIT 且距离接�? */
static int frame_has_obstacle_cluster(const LidarPoint_t *points, uint32_t count,
                                      uint16_t *out_min_mm, uint8_t *out_hits)
{
    float   angles[POINT_PER_PACK];
    uint16_t dists[POINT_PER_PACK];
    uint8_t flags[POINT_PER_PACK];
    uint8_t n = 0;
    uint16_t frame_min = 0xFFFFu;
    uint8_t total_hits = 0;

    for (uint32_t i = 0; i < count && n < POINT_PER_PACK; i++) {
        if (!angle_in_detect_sector(points[i].angle_deg)) {
            continue;
        }
        if (!point_is_valid(&points[i])) {
            continue;
        }
        angles[n] = points[i].angle_deg;
        dists[n]  = points[i].distance_mm;
        flags[n]  = 1u;
        n++;
        total_hits++;
        if (points[i].distance_mm < frame_min) {
            frame_min = points[i].distance_mm;
        }
    }

    *out_hits = total_hits;
    if (total_hits < APP_DETECT_MIN_HIT_POINTS) {
        return 0;
    }

    /* 单点命中且距离很近时直接�?认（近距 0.25 m �?标张角大�? */
    if (total_hits == 1u) {
        *out_min_mm = frame_min;
        return (frame_min <= 800u) ? 1 : 0;
    }

    for (uint8_t i = 0; i < n; i++) {
        if (!flags[i]) {
            continue;
        }
        uint8_t cluster_cnt = 1u;
        uint16_t cluster_min = dists[i];

        for (uint8_t j = 0; j < n; j++) {
            if (i == j || !flags[j]) {
                continue;
            }
            float da = angles[i] - angles[j];
            if (da < 0.0f) {
                da = -da;
            }
            if (da > 180.0f) {
                da = 360.0f - da;
            }
            if (da > APP_DETECT_CLUSTER_MAX_ANGLE_DEG) {
                continue;
            }
            uint16_t dd = (cluster_min > dists[j]) ?
                          (uint16_t)(cluster_min - dists[j]) :
                          (uint16_t)(dists[j] - cluster_min);
            if (dd > 300u) {
                continue;
            }
            cluster_cnt++;
            if (dists[j] < cluster_min) {
                cluster_min = dists[j];
            }
        }

        if (cluster_cnt >= APP_DETECT_MIN_HIT_POINTS) {
            *out_min_mm = cluster_min;
            return 1;
        }
    }

    return 0;
}

void app_detect_init(void)
{
    app_detect_reset();
}

void app_detect_reset(void)
{
    memset(&g_result, 0, sizeof(g_result));
    g_result.dist_mm = 0xFFFFu;
    g_last_update_tick = 0u;
}

void app_detect_feed(const LidarPoint_t *points, uint32_t point_count)
{
    uint16_t frame_min = 0xFFFFu;
    uint8_t  frame_hits = 0u;

    if (points == NULL || point_count == 0u) {
        return;
    }

    if (!frame_has_obstacle_cluster(points, point_count, &frame_min, &frame_hits)) {
        return;
    }

    g_result.valid = 1u;
    g_result.dist_mm = frame_min;
    g_result.hit_points = frame_hits;
    g_last_update_tick = HAL_GetTick();
}

void app_detect_process(void)
{
    uint32_t now = HAL_GetTick();

    if (g_result.valid != 0u) {
        if ((now - g_last_update_tick) > APP_DETECT_SCAN_HOLD_MS) {
            app_detect_reset();
        }
    }
}

const AppDetectResult_t *app_detect_get_result(void)
{
    return &g_result;
}

uint8_t app_detect_to_warn_level(uint16_t dist_mm, uint8_t obstacle_valid)
{
    if (obstacle_valid == 0u || dist_mm == 0xFFFFu) {
        return 0u;
    }
    if (dist_mm > APP_COMM_WARN_DIST_MM) {
        return 0u;
    }
    if (dist_mm <= APP_COMM_STOP_DIST_MM) {
        return 2u;
    }
    return 1u;
}
