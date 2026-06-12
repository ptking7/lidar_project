#include "app_detect.h"
#include "main.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 角度弧度转换系数：弧度转角度
#define DEG_PER_RAD  (180.0f / (float)M_PI)

// 激光点缓存结构体：角度、距离、时间戳
typedef struct {
    float    angle_deg;    // 角度(度)
    uint16_t dist_mm;      // 距离(毫米)
    uint32_t tick;         // 采样时间戳
} DetectBufPoint_t;

static AppDetectResult_t g_result;       // 检测结果输出
static uint32_t g_last_update_tick;      // 最后一次有效检测时间戳

static DetectBufPoint_t g_buf[APP_DETECT_BUF_MAX];  // 激光点滑动缓存区
static uint8_t g_buf_count;              // 当前缓存有效点数

static uint8_t g_confirm_cnt;            // 连续有效检测帧数
static uint8_t g_miss_cnt;                // 连续丢失目标帧数
static uint16_t g_filt_dist_mm;           // 滤波后距离值

/**
 * @brief 计算当前角度相对正前方的偏差(取左右最小夹角)
 * @param angle_deg 原始角度
 * @return 相对正前方偏差角度
 */
static float angle_dev_from_front(float angle_deg)
{
    float a = angle_deg;
    if (a > 180.0f) {
        a = 360.0f - a;
    }
    return a;
}

/**
 * @brief 判断激光点是否在有效检测通道范围内
 * @param angle_deg 点角度
 * @param dist_mm   点距离
 * @return 1:在通道内  0:超出范围
 */
static int point_in_corridor(float angle_deg, uint16_t dist_mm)
{
    float dev;
    float half_deg;

    // 距离超限直接剔除
    if (dist_mm < APP_DETECT_DIST_MIN_MM || dist_mm > APP_DETECT_RANGE_MAX_MM) {
        return 0;
    }

    dev = angle_dev_from_front(angle_deg);
    // 根据距离计算通道半角范围
    half_deg = atan2f((float)APP_DETECT_CORRIDOR_HALF_WIDTH_MM, (float)dist_mm) *
               DEG_PER_RAD;
    // 限制最大半角
    if (half_deg > APP_DETECT_CORRIDOR_MAX_HALF_DEG) {
        half_deg = APP_DETECT_CORRIDOR_MAX_HALF_DEG;
    }
    return (dev <= half_deg) ? 1 : 0;
}

/**
 * @brief 校验单个激光点是否合法(距离、强度、通道)
 * @param p 激光点数据指针
 * @return 1:有效点  0:无效点
 */
static int point_is_valid(const LidarPoint_t *p)
{
    // 距离下限
    if (p->distance_mm < APP_DETECT_DIST_MIN_MM) {
        return 0;
    }
    // 距离上限
    if (p->distance_mm > APP_DETECT_DIST_MAX_MM) {
        return 0;
    }
    // 反射强度过滤
    if (p->intensity < APP_DETECT_MIN_INTENSITY) {
        return 0;
    }
    // 通道范围过滤
    if (!point_in_corridor(p->angle_deg, p->distance_mm)) {
        return 0;
    }
    return 1;
}

/**
 * @brief 根据参考距离计算聚类判定阈值(角度差、距离差)
 * @param d_ref_mm 参考距离
 * @param angle_deg 输出：最大允许角度差
 * @param dd_mm     输出：最大允许距离差
 */
static void cluster_limits(uint16_t d_ref_mm, float *angle_deg, uint16_t *dd_mm)
{
    float d_m;
    float span_deg;
    float ang;
    uint16_t dd;

    // 限制最小参考距离
    if (d_ref_mm < 100u) {
        d_ref_mm = 100u;
    }
    d_m = (float)d_ref_mm / 1000.0f;
    // 按目标宽度计算聚类角度范围
    span_deg = ((float)APP_DETECT_TARGET_WIDTH_MM / 1000.0f) / d_m * DEG_PER_RAD;
    ang = span_deg + 1.0f;
    // 钳位角度上下限
    if (ang < APP_DETECT_CLUSTER_ANGLE_MIN_DEG) {
        ang = APP_DETECT_CLUSTER_ANGLE_MIN_DEG;
    }
    if (ang > APP_DETECT_CLUSTER_ANGLE_MAX_DEG) {
        ang = APP_DETECT_CLUSTER_ANGLE_MAX_DEG;
    }
    *angle_deg = ang;

    // 按比例计算距离差阈值
    dd = (uint16_t)(((uint32_t)d_ref_mm * APP_DETECT_CLUSTER_DD_RATIO_PCT) / 100u);
    // 钳位距离差上下限
    if (dd < APP_DETECT_CLUSTER_DD_MIN_MM) {
        dd = APP_DETECT_CLUSTER_DD_MIN_MM;
    }
    if (dd > APP_DETECT_CLUSTER_DD_MAX_MM) {
        dd = APP_DETECT_CLUSTER_DD_MAX_MM;
    }
    *dd_mm = dd;
}

/**
 * @brief 计算两个角度的最小差值(0~180°)
 * @param a 角度1
 * @param b 角度2
 * @return 角度差值
 */
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

/**
 * @brief 清理滑动窗口内超时的历史点
 * @param now 当前系统时间戳
 */
static void buf_prune_old(uint32_t now)
{
    uint8_t w = 0;
    for (uint8_t r = 0; r < g_buf_count; r++) {
        // 保留窗口时间内的有效点
        if ((now - g_buf[r].tick) <= APP_DETECT_WINDOW_MS) {
            if (w != r) {
                g_buf[w] = g_buf[r];
            }
            w++;
        }
    }
    g_buf_count = w;
}

/**
 * @brief 将有效激光点加入缓存
 * @param angle_deg 角度
 * @param dist_mm   距离
 * @param tick      当前时间戳
 */
static void buf_add_point(float angle_deg, uint16_t dist_mm, uint32_t tick)
{
    // 缓存满则丢弃
    if (g_buf_count >= APP_DETECT_BUF_MAX) {
        return;
    }
    g_buf[g_buf_count].angle_deg = angle_deg;
    g_buf[g_buf_count].dist_mm   = dist_mm;
    g_buf[g_buf_count].tick      = tick;
    g_buf_count++;
}

/**
 * @brief 滑动窗口内点聚类检测，判定是否存在障碍物
 * @param out_min_mm 输出：窗口内最近距离
 * @param out_hits   输出：有效聚类点数
 * @return 1:检测到目标  0:未检测到
 */
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

    // 逐点聚类匹配
    for (uint8_t i = 0; i < n; i++) {
        uint8_t cluster_cnt = 1u;
        uint16_t d_ref = g_buf[i].dist_mm;

        for (uint8_t j = 0; j < n; j++) {
            float max_ang;
            uint16_t max_dd;

            if (i == j) {
                continue;
            }
            // 取两点中更近距离作为参考
            d_ref = g_buf[i].dist_mm;
            if (g_buf[j].dist_mm < d_ref) {
                d_ref = g_buf[j].dist_mm;
            }
            // 获取当前距离对应的聚类阈值
            cluster_limits(d_ref, &max_ang, &max_dd);

            // 角度差超限则不属于同一聚类
            if (angle_diff_deg(g_buf[i].angle_deg, g_buf[j].angle_deg) > max_ang) {
                continue;
            }
            // 距离差超限则不属于同一聚类
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

        // 聚类点数达标，判定为有效目标点
        if (cluster_cnt >= APP_DETECT_MIN_HIT_POINTS) {
            confirmed[i] = 1u;
        }
    }

    // 特殊规则：窗口仅单个近点也判定有效
    if (n == 1u && g_buf[0].dist_mm <= APP_DETECT_NEAR_SINGLE_MAX_MM) {
        confirmed[0] = 1u;
    }

    // 统计有效点、查找最小距离
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

/**
 * @brief 距离一阶滤波：近变快跟、远变慢跟，抑制跳变
 * @param raw_min_mm 原始最近距离
 */
static void dist_filter_update(uint16_t raw_min_mm)
{
    // 首次赋值
    if (g_filt_dist_mm == 0xFFFFu) {
        g_filt_dist_mm = raw_min_mm;
        return;
    }

    // 距离明显变近：快速跟随
    if (raw_min_mm + APP_DETECT_DIST_JUMP_MM < g_filt_dist_mm) {
        g_filt_dist_mm = (uint16_t)(((uint32_t)g_filt_dist_mm + (uint32_t)raw_min_mm * 3u) / 4u);
    }
    // 距离明显变远：慢速跟随，防遮挡误跳
    else if (raw_min_mm > g_filt_dist_mm + APP_DETECT_DIST_JUMP_MM) {
        g_filt_dist_mm = (uint16_t)(((uint32_t)g_filt_dist_mm * 3u + (uint32_t)raw_min_mm) / 4u);
    }
    // 小幅变化直接更新
    else {
        g_filt_dist_mm = raw_min_mm;
    }
}

/**
 * @brief 检测模块初始化
 */
void app_detect_init(void)
{
    app_detect_reset();
}

/**
 * @brief 复位所有检测状态与缓存
 */
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

/**
 * @brief 送入一帧激光雷达数据进行检测处理
 * @param points      激光点数组
 * @param point_count 有效点数
 */
void app_detect_feed(const LidarPoint_t *points, uint32_t point_count)
{
    uint16_t raw_min = 0xFFFFu;
    uint8_t  raw_hits = 0u;
    uint32_t now = HAL_GetTick();

    if (points == NULL || point_count == 0u) {
        return;
    }

    // 先清理超时旧点
    buf_prune_old(now);

    // 遍历所有点，筛选有效点存入缓存
    for (uint32_t i = 0; i < point_count; i++) {
        if (!point_is_valid(&points[i])) {
            continue;
        }
        buf_add_point(points[i].angle_deg, points[i].distance_mm, now);
    }

    // 滑动窗口聚类检测
    if (!window_detect(&raw_min, &raw_hits)) {
        g_miss_cnt++;
        g_confirm_cnt = 0u;
        return;
    }

    // 检测到目标，刷新计数
    g_miss_cnt = 0u;
    if (g_confirm_cnt < 255u) {
        g_confirm_cnt++;
    }

    // 距离滤波
    dist_filter_update(raw_min);

    // 连续帧数不足，不输出最终结果
    if (g_confirm_cnt < APP_DETECT_CONFIRM_FRAMES) {
        return;
    }

    // 确认有效，更新输出结果
    g_result.valid = 1u;
    g_result.dist_mm = g_filt_dist_mm;
    g_result.hit_points = raw_hits;
    g_last_update_tick = now;
}

/**
 * @brief 后台周期处理：超时/丢失目标则复位状态
 */
void app_detect_process(void)
{
    uint32_t now = HAL_GetTick();

    // 定时清理超时缓存点
    buf_prune_old(now);

    if (g_result.valid != 0u) {
        // 连续丢失帧数达标，清除检测结果
        if (g_miss_cnt >= APP_DETECT_CLEAR_FRAMES) {
            app_detect_reset();
            return;
        }
        // 长时间无新数据，清除检测结果
        if ((now - g_last_update_tick) > APP_DETECT_SCAN_HOLD_MS) {
            app_detect_reset();
        }
    }
}

/**
 * @brief 获取当前检测结果
 * @return 检测结果结构体指针
 */
const AppDetectResult_t *app_detect_get_result(void)
{
    return &g_result;
}

/**
 * @brief 根据距离转换为告警等级
 * @param dist_mm        障碍物距离
 * @param obstacle_valid 障碍物是否有效
 * @return 0:无告警 1:预警 2:急停告警
 */
uint8_t app_detect_to_warn_level(uint16_t dist_mm, uint8_t obstacle_valid)
{
    if (obstacle_valid == 0u || dist_mm == 0xFFFFu) {
        return 0u;
    }
    // 距离过远，无告警
    if (dist_mm > APP_COMM_WARN_DIST_MM) {
        return 0u;
    }
    // 近距离：急停等级
    if (dist_mm <= APP_COMM_STOP_DIST_MM) {
        return 2u;
    }
    // 中距离：预警等级
    return 1u;
}
