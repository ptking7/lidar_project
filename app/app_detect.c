#include "app_detect.h"
#include "main.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_PER_RAD  (180.0f / (float)M_PI)

typedef struct {
    float    angle_deg;
    uint16_t dist_mm;
    uint32_t tick;
} DetectBufPoint_t;

static AppDetectResult_t g_result;
static uint32_t g_last_update_tick;

static DetectBufPoint_t g_buf[APP_DETECT_BUF_MAX];
static uint8_t g_buf_count;

static uint8_t g_confirm_cnt;
static uint8_t g_miss_cnt;
static uint16_t g_filt_dist_mm;

/* -------------------------------------------------------------------------- */
static float angle_dev_from_front(float angle_deg)
{
    float a = angle_deg;
    if (a > 180.0f) {
        a = 360.0f - a;
    }
    return a;
}

static int point_in_corridor(float angle_deg, uint16_t dist_mm)
{
    float dev;
    float half_deg;

    if (dist_mm < APP_DETECT_DIST_MIN_MM || dist_mm > APP_DETECT_RANGE_MAX_MM) {
        return 0;
    }

    dev = angle_dev_from_front(angle_deg);
    half_deg = atan2f((float)APP_DETECT_CORRIDOR_HALF_WIDTH_MM, (float)dist_mm) *
               DEG_PER_RAD;
    if (half_deg > APP_DETECT_CORRIDOR_MAX_HALF_DEG) {
        half_deg = APP_DETECT_CORRIDOR_MAX_HALF_DEG;
    }
    return (dev <= half_deg) ? 1 : 0;
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
    if (!point_in_corridor(p->angle_deg, p->distance_mm)) {
        return 0;
    }
    return 1;
}

static void cluster_limits(uint16_t d_ref_mm, float *angle_deg, uint16_t *dd_mm)
{
    float d_m;
    float span_deg;
    float ang;
    uint16_t dd;

    if (d_ref_mm < 100u) {
        d_ref_mm = 100u;
    }
    d_m = (float)d_ref_mm / 1000.0f;
    span_deg = ((float)APP_DETECT_TARGET_WIDTH_MM / 1000.0f) / d_m * DEG_PER_RAD;
    ang = span_deg + 1.0f;
    if (ang < APP_DETECT_CLUSTER_ANGLE_MIN_DEG) {
        ang = APP_DETECT_CLUSTER_ANGLE_MIN_DEG;
    }
    if (ang > APP_DETECT_CLUSTER_ANGLE_MAX_DEG) {
        ang = APP_DETECT_CLUSTER_ANGLE_MAX_DEG;
    }
    *angle_deg = ang;

    dd = (uint16_t)(((uint32_t)d_ref_mm * APP_DETECT_CLUSTER_DD_RATIO_PCT) / 100u);
    if (dd < APP_DETECT_CLUSTER_DD_MIN_MM) {
        dd = APP_DETECT_CLUSTER_DD_MIN_MM;
    }
    if (dd > APP_DETECT_CLUSTER_DD_MAX_MM) {
        dd = APP_DETECT_CLUSTER_DD_MAX_MM;
    }
    *dd_mm = dd;
}

static float angle_diff_deg(float a, float b)
{
    float da = a - b;
    if (da < 0.0f) {
        da = -da;
    }
    if (da > 180.0f) {
        da = 360.0f - da;
    }
    return da;
}

static void buf_prune_old(uint32_t now)
{
    uint8_t w = 0;
    for (uint8_t r = 0; r < g_buf_count; r++) {
        if ((now - g_buf[r].tick) <= APP_DETECT_WINDOW_MS) {
            if (w != r) {
                g_buf[w] = g_buf[r];
            }
            w++;
        }
    }
    g_buf_count = w;
}

static void buf_add_point(float angle_deg, uint16_t dist_mm, uint32_t tick)
{
    if (g_buf_count >= APP_DETECT_BUF_MAX) {
        return;
    }
    g_buf[g_buf_count].angle_deg = angle_deg;
    g_buf[g_buf_count].dist_mm   = dist_mm;
    g_buf[g_buf_count].tick      = tick;
    g_buf_count++;
}

/* 在滑动窗口内聚类，返回全局最近确认距离（修复：不用“某一簇”代替全场最近） */
static int window_detect(uint16_t *out_min_mm, uint8_t *out_hits)
{
    uint8_t confirmed[APP_DETECT_BUF_MAX];
    uint8_t n = g_buf_count;
    uint16_t global_min = 0xFFFFu;
    uint8_t confirmed_cnt = 0u;

    if (n == 0u) {
        return 0;
    }

    memset(confirmed, 0, sizeof(confirmed));

    for (uint8_t i = 0; i < n; i++) {
        uint8_t cluster_cnt = 1u;
        uint16_t d_ref = g_buf[i].dist_mm;

        for (uint8_t j = 0; j < n; j++) {
            float max_ang;
            uint16_t max_dd;

            if (i == j) {
                continue;
            }
            d_ref = g_buf[i].dist_mm;
            if (g_buf[j].dist_mm < d_ref) {
                d_ref = g_buf[j].dist_mm;
            }
            cluster_limits(d_ref, &max_ang, &max_dd);

            if (angle_diff_deg(g_buf[i].angle_deg, g_buf[j].angle_deg) > max_ang) {
                continue;
            }
            {
                uint16_t dd = (g_buf[i].dist_mm > g_buf[j].dist_mm) ?
                              (uint16_t)(g_buf[i].dist_mm - g_buf[j].dist_mm) :
                              (uint16_t)(g_buf[j].dist_mm - g_buf[i].dist_mm);
                if (dd > max_dd) {
                    continue;
                }
            }
            cluster_cnt++;
        }

        if (cluster_cnt >= APP_DETECT_MIN_HIT_POINTS) {
            confirmed[i] = 1u;
        }
    }

    /* 近距单点：窗口内仅 1 点且很近 */
    if (n == 1u && g_buf[0].dist_mm <= APP_DETECT_NEAR_SINGLE_MAX_MM) {
        confirmed[0] = 1u;
    }

    for (uint8_t i = 0; i < n; i++) {
        if (confirmed[i] == 0u) {
            continue;
        }
        confirmed_cnt++;
        if (g_buf[i].dist_mm < global_min) {
            global_min = g_buf[i].dist_mm;
        }
    }

    *out_hits = confirmed_cnt;
    if (confirmed_cnt == 0u || global_min == 0xFFFFu) {
        return 0;
    }

    *out_min_mm = global_min;
    return 1;
}

static void dist_filter_update(uint16_t raw_min_mm)
{
    if (g_filt_dist_mm == 0xFFFFu) {
        g_filt_dist_mm = raw_min_mm;
        return;
    }

    if (raw_min_mm + APP_DETECT_DIST_JUMP_MM < g_filt_dist_mm) {
        /* 明显变近：快速跟随 */
        g_filt_dist_mm = (uint16_t)(((uint32_t)g_filt_dist_mm + (uint32_t)raw_min_mm * 3u) / 4u);
    } else if (raw_min_mm > g_filt_dist_mm + APP_DETECT_DIST_JUMP_MM) {
        /* 大幅变远：慢速跟随，抑制手挡一下又放开导致的乱跳 */
        g_filt_dist_mm = (uint16_t)(((uint32_t)g_filt_dist_mm * 3u + (uint32_t)raw_min_mm) / 4u);
    } else {
        g_filt_dist_mm = raw_min_mm;
    }
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
    g_buf_count = 0u;
    g_confirm_cnt = 0u;
    g_miss_cnt = 0u;
    g_filt_dist_mm = 0xFFFFu;
}

void app_detect_feed(const LidarPoint_t *points, uint32_t point_count)
{
    uint16_t raw_min = 0xFFFFu;
    uint8_t  raw_hits = 0u;
    uint32_t now = HAL_GetTick();

    if (points == NULL || point_count == 0u) {
        return;
    }

    buf_prune_old(now);

    for (uint32_t i = 0; i < point_count; i++) {
        if (!point_is_valid(&points[i])) {
            continue;
        }
        buf_add_point(points[i].angle_deg, points[i].distance_mm, now);
    }

    if (!window_detect(&raw_min, &raw_hits)) {
        g_miss_cnt++;
        g_confirm_cnt = 0u;
        return;
    }

    g_miss_cnt = 0u;
    if (g_confirm_cnt < 255u) {
        g_confirm_cnt++;
    }

    dist_filter_update(raw_min);

    if (g_confirm_cnt < APP_DETECT_CONFIRM_FRAMES) {
        return;
    }

    g_result.valid = 1u;
    g_result.dist_mm = g_filt_dist_mm;
    g_result.hit_points = raw_hits;
    g_last_update_tick = now;
}

void app_detect_process(void)
{
    uint32_t now = HAL_GetTick();

    buf_prune_old(now);

    if (g_result.valid != 0u) {
        if (g_miss_cnt >= APP_DETECT_CLEAR_FRAMES) {
            app_detect_reset();
            return;
        }
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
