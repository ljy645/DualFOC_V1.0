/** @file motor_calibration.c
 *  @brief 电角零位/方向校准与低速双向齿槽标定。
 */
#include "motor_calibration.h"
#include "cogging.h"
#include "app.h"
#include "enc_mt6826s.h"
#include "board_critical.h"
#include "main.h"
#include "dsp/controller_functions.h"
#include <math.h>
#include <string.h>

#define MOTOR_CALIBRATION_STARTUP_TIMEOUT_MS 10U
#define MOTOR_CALIBRATION_TWO_PI 6.2831853071795865f
#define MOTOR_CALIBRATION_MEASURE_MS 200U
/* 标定设置集中在本模块，调用者只传轴号；不自动升压重试。 */
#define CAL_VOLTAGE_V 0.4f
#define CAL_CURRENT_LIMIT_A 3.0f
#define CAL_SETTLE_MS 500U
#define CAL_SWEEP_MS 1500U
#define CAL_PROBE_ANGLE_RAD (MOTOR_CALIBRATION_TWO_PI * 0.5f)

#define COG_CAL_RPM 10.0f
#define COG_CAL_SPEED_WINDOW_MS 500U
#define COG_CAL_STABLE_WINDOWS 2U
#define COG_CAL_SETTLE_TIMEOUT_MS 10000U
#define COG_CAL_SPEED_TOLERANCE_RPM 2.0f
#define COG_CAL_TIMEOUT_MS 30000U
#define COG_CAL_REVOLUTIONS 3.0f

/* 仅前台改变命令角度/电压；ADC IRQ 和前台共同监测，临界区发布状态。 */
static struct
{
    volatile bool active;
    unsigned int axis;
    float current_limit_a;
    float angle;
    float voltage_v;
    volatile MotorCalibrationResult result;
} calibration;

/* 全局诊断快照，首次失败时冻结，方便 Keil Watch 直接观察。 */
volatile struct
{
    MotorHwSample feedback;
    bool feedback_available;
    EncMt6826sResult encoder_result;
    uint8_t app_faults;
    float forward_deg;
    float backward_deg;
    float expected_deg;
    uint32_t encoder_age_us; /* 关断前编码器帧年龄，通信结果无效时为 UINT32_MAX。 */
} g_calibration_debug;

int MotorCalibration_ActiveAxis(void)
{
    return calibration.active ? (int)calibration.axis : -1;
}

/** @brief 保留首次标定故障并立即关断，防止前台覆盖 ADC 中断的故障原因。 */
static void MotorCalibration_Fail(MotorCalibrationResult result)
{
    uint32_t saved = Board_EnterCritical();
    if (calibration.result == MOTOR_CALIBRATION_OK || calibration.result == MOTOR_CALIBRATION_NOT_READY)
    {
        MotorHwSample sample = {0};
        EncMt6826sSample encoder;
        g_calibration_debug.feedback_available = MotorHw_PeekSample(calibration.axis, &sample);
        g_calibration_debug.feedback = sample;
        g_calibration_debug.encoder_result = EncMt6826s_ReadLatest(calibration.axis, &encoder);
        g_calibration_debug.encoder_age_us = g_calibration_debug.encoder_result == ENC_MT6826S_OK ?
                                            TIM5->CNT - encoder.time_us : UINT32_MAX;
        calibration.result = result;
        (void)MotorHw_SetOutputEnabled(calibration.axis, false);
    }
    Board_ExitCritical(saved);
}

/** @brief 标定专用实测保护；前台也调用，避免 ADC 中断停止后保留电压输出。 */
uint8_t MotorCalibration_Monitor(uint8_t faults)
{
    MotorHwSample sample;
    uint8_t axis_bit = (uint8_t)(1U << calibration.axis);
    /* 失败已在标定期间上报；空闲时不能用上次结果反复锁住后续显式启动。 */
    if (!calibration.active) { return 0U; }
    if ((faults & axis_bit) != 0U &&
        (calibration.result == MOTOR_CALIBRATION_OK || calibration.result == MOTOR_CALIBRATION_NOT_READY))
    {
        g_calibration_debug.app_faults = faults;
        MotorCalibration_Fail(MOTOR_CALIBRATION_FEEDBACK_ERROR);
    }
    /* 初始等待反馈时不施加电压，也不把尚未就绪误报为运行故障。 */
    if (calibration.result == MOTOR_CALIBRATION_NOT_READY) { return 0U; }
    if (calibration.result != MOTOR_CALIBRATION_OK) { return axis_bit; }
    if (!MotorHw_PeekSample(calibration.axis, &sample) ||
        !sample.feedback_complete || sample.faults != 0U)
    {
        MotorCalibration_Fail(MOTOR_CALIBRATION_FEEDBACK_ERROR);
    }
    else if (sample.vbus_v < APP_BUS_MIN_V || sample.vbus_v > APP_BUS_MAX_V)
    {
        MotorCalibration_Fail(MOTOR_CALIBRATION_BUS_VOLTAGE);
    }
    else if (fabsf(sample.ia_a) >= calibration.current_limit_a ||
             fabsf(sample.ib_a) >= calibration.current_limit_a ||
             fabsf(sample.ia_a + sample.ib_a) >= calibration.current_limit_a)
    {
        MotorCalibration_Fail(MOTOR_CALIBRATION_OVERCURRENT);
    }
    return calibration.result == MOTOR_CALIBRATION_OK ? 0U : axis_bit;
}

/** @brief 连续移动电压矢量；扫描时展开编码器跨圈，得到真实机械位移。 */
static bool MotorCalibration_Move(float angle, float voltage_v, uint32_t duration_ms, float *travel)
{
    EncMt6826sSample encoder;
    uint32_t started = HAL_GetTick();
    uint32_t previous_ms = started - 1U;
    float start_angle = calibration.angle;
    float start_voltage = calibration.voltage_v;
    float previous_angle;
    float movement = 0.0f;
    if (EncMt6826s_ReadLatest(calibration.axis, &encoder) != ENC_MT6826S_OK)
    {
        MotorCalibration_Fail(MOTOR_CALIBRATION_FEEDBACK_ERROR);
        return false;
    }
    previous_angle = encoder.angle_rad;
    for (;;)
    {
        MotorHwSample sample;
        uint32_t now;
        uint32_t elapsed;
        float fraction;
        float sine;
        float cosine;
        float alpha;
        float beta;
        float delta;
        App_Poll();
        if (calibration.result != MOTOR_CALIBRATION_OK) { return false; }
        now = HAL_GetTick();
        if (now == previous_ms) { continue; }
        previous_ms = now;
        elapsed = now - started;
        fraction = elapsed >= duration_ms ? 1.0f : (float)elapsed / (float)duration_ms;
        calibration.angle = start_angle + (angle - start_angle) * fraction;
        calibration.voltage_v = start_voltage + (voltage_v - start_voltage) * fraction;
        if (!MotorHw_PeekSample(calibration.axis, &sample) ||
            EncMt6826s_ReadLatest(calibration.axis, &encoder) != ENC_MT6826S_OK)
        {
            MotorCalibration_Fail(MOTOR_CALIBRATION_FEEDBACK_ERROR);
            return false;
        }
        delta = encoder.angle_rad - previous_angle;
        if (delta > MOTOR_CALIBRATION_TWO_PI * 0.5f) { delta -= MOTOR_CALIBRATION_TWO_PI; }
        else if (delta < -MOTOR_CALIBRATION_TWO_PI * 0.5f) { delta += MOTOR_CALIBRATION_TWO_PI; }
        movement += delta;
        previous_angle = encoder.angle_rad;
        arm_sin_cos_f32(calibration.angle * 57.29577951308232f, &sine, &cosine);
        alpha = calibration.voltage_v * cosine / sample.vbus_v;
        beta = calibration.voltage_v * sine / sample.vbus_v;
        if (!MotorHw_WriteDuty(calibration.axis, 0.5f + alpha,
                              0.5f - 0.5f * alpha + 0.8660254037844386f * beta,
                              0.5f - 0.5f * alpha - 0.8660254037844386f * beta))
        {
            MotorCalibration_Fail(MOTOR_CALIBRATION_OUTPUT_ERROR);
            return false;
        }
        if (elapsed >= duration_ms) { break; }
    }
    if (travel != 0) { *travel = movement; }
    return true;
}

/** @brief 对最终定位处的独立编码器帧做圆周平均。 */
static bool MotorCalibration_Measure(uint16_t pole_pairs, float *angle)
{
    uint32_t started = HAL_GetTick();
    uint32_t previous_time = 0U;
    unsigned int count = 0U;
    float sum_sine = 0.0f;
    float sum_cosine = 0.0f;
    while ((uint32_t)(HAL_GetTick() - started) < MOTOR_CALIBRATION_MEASURE_MS)
    {
        EncMt6826sSample encoder;
        float sine;
        float cosine;
        float phase;
        App_Poll();
        if (calibration.result != MOTOR_CALIBRATION_OK) { return false; }
        if (EncMt6826s_ReadLatest(calibration.axis, &encoder) != ENC_MT6826S_OK)
        {
            MotorCalibration_Fail(MOTOR_CALIBRATION_FEEDBACK_ERROR);
            return false;
        }
        if (count != 0U && encoder.time_us == previous_time) { continue; }
        previous_time = encoder.time_us;
        phase = fmodf(encoder.angle_rad * pole_pairs, MOTOR_CALIBRATION_TWO_PI);
        arm_sin_cos_f32(phase * 57.29577951308232f, &sine, &cosine);
        sum_sine += sine;
        sum_cosine += cosine;
        ++count;
    }
    if (count == 0U)
    {
        MotorCalibration_Fail(MOTOR_CALIBRATION_FEEDBACK_ERROR);
        return false;
    }
    *angle = atan2f(sum_sine, sum_cosine);
    return true;
}

MotorCalibrationResult MotorCalibration_Run(unsigned int axis, MotorConfig *config)
{
    MotorHwSample sample;
    int8_t direction;
    float zero_rad;
    uint32_t saved;
    uint32_t started;
    float movement[2];
    float expected;
    unsigned int i;
    bool enabled = false;
    MotorCalibrationResult result;
    saved = Board_EnterCritical();
    calibration.axis = axis;
    calibration.current_limit_a = fminf(config->current_limit_a, CAL_CURRENT_LIMIT_A);
    calibration.angle = calibration.voltage_v = 0.0f;
    calibration.result = MOTOR_CALIBRATION_NOT_READY;
    g_calibration_debug.feedback_available = false;
    g_calibration_debug.feedback.faults = 0U;
    g_calibration_debug.feedback.feedback_complete = false;
    g_calibration_debug.encoder_result = ENC_MT6826S_NOT_READY;
    g_calibration_debug.app_faults = 0U;
    g_calibration_debug.encoder_age_us = UINT32_MAX;
    g_calibration_debug.forward_deg = 0.0f;
    g_calibration_debug.backward_deg = 0.0f;
    g_calibration_debug.expected_deg = CAL_PROBE_ANGLE_RAD * 57.29577951308232f / config->pole_pairs;
    calibration.active = true;
    (void)MotorHw_SetOutputEnabled(axis, false);
    MotorControl_Reset(axis);
    Board_ExitCritical(saved);
    /* 让现有调度撤销本轴正常运行位，即使 ADC 中断停了也不会残留运行状态。 */
    App_Poll();
    if (calibration.result != MOTOR_CALIBRATION_NOT_READY) { goto done; }

    /* 输出关闭时装载中性占空比；在同一等待窗口内确认反馈和预装载更新。 */
    if (!MotorHw_WriteDuty(axis, 0.5f, 0.5f, 0.5f))
    {
        calibration.result = MOTOR_CALIBRATION_OUTPUT_ERROR;
        goto done;
    }
    started = HAL_GetTick();
    while (!enabled && (uint32_t)(HAL_GetTick() - started) < MOTOR_CALIBRATION_STARTUP_TIMEOUT_MS)
    {
        App_Poll();
        if (calibration.result == MOTOR_CALIBRATION_NOT_READY)
        {
            if (!MotorHw_PeekSample(axis, &sample) || !sample.feedback_complete || sample.faults != 0U ||
                sample.vbus_v < APP_BUS_MIN_V || sample.vbus_v > APP_BUS_MAX_V) { continue; }
            calibration.result = MOTOR_CALIBRATION_OK;
            (void)MotorCalibration_Monitor(0U);
        }
        if (calibration.result != MOTOR_CALIBRATION_OK) { goto done; }
        saved = Board_EnterCritical();
        if (calibration.result == MOTOR_CALIBRATION_OK)
        {
            enabled = MotorHw_SetOutputEnabled(axis, true);
        }
        Board_ExitCritical(saved);
    }
    if (!enabled)
    {
        if (calibration.result == MOTOR_CALIBRATION_OK) { MotorCalibration_Fail(MOTOR_CALIBRATION_OUTPUT_ERROR); }
        goto done;
    }
    /* 升压、正向扫描、反向返回；每步等待定位，最后测量零位。 */
    for (i = 0U; i < 3U; ++i)
    {
        float angle = i == 1U ? CAL_PROBE_ANGLE_RAD : 0.0f;
        uint32_t duration = i == 0U ? CAL_SETTLE_MS : CAL_SWEEP_MS;
        float settled_travel;
        float *travel = i == 0U ? 0 : &movement[i - 1U];
        if (!MotorCalibration_Move(angle, CAL_VOLTAGE_V, duration, travel) ||
            !MotorCalibration_Move(angle, CAL_VOLTAGE_V, CAL_SETTLE_MS, &settled_travel)) { goto done; }
        if (travel != 0) { *travel += settled_travel; }
        if (i == 1U) { g_calibration_debug.forward_deg = movement[0] * 57.29577951308232f; }
        else if (i == 2U) { g_calibration_debug.backward_deg = movement[1] * 57.29577951308232f; }
    }

    expected = CAL_PROBE_ANGLE_RAD / config->pole_pairs;
    if (fabsf(movement[0]) < expected * 0.1f || fabsf(movement[1]) < expected * 0.1f)
    {
        MotorCalibration_Fail(MOTOR_CALIBRATION_NO_MOVEMENT);
        goto done;
    }
    if (movement[0] * movement[1] >= 0.0f || fabsf(fabsf(movement[0]) - expected) > expected * 0.2f ||
        fabsf(fabsf(movement[1]) - expected) > expected * 0.2f)
    {
        MotorCalibration_Fail(MOTOR_CALIBRATION_TRACKING_ERROR);
        goto done;
    }
    if (!MotorCalibration_Measure(config->pole_pairs, &zero_rad)) { goto done; }
    direction = movement[0] > 0.0f ? 1 : -1;
    zero_rad *= direction;
    if (zero_rad < 0.0f) { zero_rad += MOTOR_CALIBRATION_TWO_PI; }

done:
    saved = Board_EnterCritical();
    result = calibration.result;
    (void)MotorHw_SetOutputEnabled(axis, false);
    if (result == MOTOR_CALIBRATION_OK)
    {
        config->encoder_direction = direction;
        config->electrical_zero_rad = zero_rad;
        MotorControl_Configure(axis, config);
    }
    else { MotorControl_Reset(axis); }
    calibration.active = false;
    Board_ExitCritical(saved);
    /* 保持输出关闭；command 层保存零位后继续齿槽标定或等待显式使能。 */
    /* 返回前发布最终故障状态并刷新指示灯，避免连续调用绕过故障锁存。 */
    App_Poll();
    return result;
}

/* 一次只标定一轴；工作区不放栈上，也不进入电流中断。 */
static float sums[COGGING_POINTS];
static uint16_t counts[COGGING_POINTS];
volatile struct
{
    unsigned int axis;
    unsigned int direction; /* 0=正向，1=反向。 */
    float revolutions;
    unsigned int min_count;
    float average_rpm; /* 按实际角位移计算的窗口平均转速。 */
    bool collecting;
    CoggingCalibrationResult result;
} g_cogging_debug;

static bool MotorCalibration_ReadCogging(unsigned int axis, MotorHwSample *sample, float *iq)
{
    float feedback[7];
    uint32_t saved = Board_EnterCritical();
    bool available = MotorHw_PeekSample(axis, sample);
    MotorControl_GetFeedback(axis, feedback);
    Board_ExitCritical(saved);
    *iq = feedback[6];
    return available && sample->feedback_complete && sample->faults == 0U && isfinite(*iq);
}

CoggingCalibrationResult MotorCalibration_RunCogging(unsigned int axis)
{
    int16_t *table = Cogging_Table(axis);
    const MotorConfig *config = MotorControl_GetConfig(axis);
    CoggingCalibrationResult result = COGGING_CAL_OK;
    unsigned int pass, i;
    float mean = 0.0f;
    g_cogging_debug.axis = axis;
    for (pass = 0U; pass < 2U; ++pass)
    {
        uint32_t started, previous_ms, window_started;
        float previous_angle = 0.0f, travel = 0.0f, window_travel = 0.0f;
        unsigned int stable_windows = 0U;
        bool have_angle = false, collecting = false;
        uint32_t saved;
        memset(sums, 0, sizeof(sums));
        memset(counts, 0, sizeof(counts));
        g_cogging_debug.direction = pass;
        g_cogging_debug.revolutions = 0.0f;
        g_cogging_debug.average_rpm = 0.0f;
        g_cogging_debug.collecting = false;
        g_cogging_debug.min_count = 0U;
        saved = Board_EnterCritical();
        MotorControl_SetSpeedTargetImmediate(axis, pass == 0U ? COG_CAL_RPM : -COG_CAL_RPM);
        Board_ExitCritical(saved);
        if (pass == 0U && (App_EnableCalibration(axis) & (1U << axis)) == 0U)
        {
            result = COGGING_CAL_OUTPUT; goto done;
        }
        started = HAL_GetTick();
        previous_ms = started;
        window_started = started;
        while (travel < COG_CAL_REVOLUTIONS * MOTOR_CALIBRATION_TWO_PI)
        {
            MotorHwSample sample;
            float iq, delta = 0.0f;
            uint32_t now;
            App_Poll();
            if ((App_RunningAxes() & (1U << axis)) == 0U)
            {
                result = COGGING_CAL_FEEDBACK; goto done;
            }
            now = HAL_GetTick();
            if ((uint32_t)(now - started) >=
                (collecting ? COG_CAL_TIMEOUT_MS : COG_CAL_SETTLE_TIMEOUT_MS))
            {
                result = COGGING_CAL_TIMEOUT; goto done;
            }
            if (now == previous_ms) { continue; }
            previous_ms = now;
            /* 换向后的加减速过程不参与学习。 */
            if (!MotorCalibration_ReadCogging(axis, &sample, &iq)) { result = COGGING_CAL_FEEDBACK; goto done; }
            if (have_angle)
            {
                delta = sample.mechanical_angle_rad - previous_angle;
                if (delta > MOTOR_CALIBRATION_TWO_PI * 0.5f) { delta -= MOTOR_CALIBRATION_TWO_PI; }
                if (delta < -MOTOR_CALIBRATION_TWO_PI * 0.5f) { delta += MOTOR_CALIBRATION_TWO_PI; }
            }
            previous_angle = sample.mechanical_angle_rad;
            have_angle = true;
            if (!collecting)
            {
                uint32_t elapsed = now - window_started;
                float target_rpm = pass == 0U ? COG_CAL_RPM : -COG_CAL_RPM;
                window_travel += delta * config->encoder_direction;
                if (elapsed < COG_CAL_SPEED_WINDOW_MS) { continue; }
                /* 低速齿槽波动不能按每毫秒的瞬时转速判稳。 */
                g_cogging_debug.average_rpm = window_travel * 60000.0f /
                                             (MOTOR_CALIBRATION_TWO_PI * elapsed);
                stable_windows = fabsf(g_cogging_debug.average_rpm - target_rpm) <=
                                 COG_CAL_SPEED_TOLERANCE_RPM ? stable_windows + 1U : 0U;
                window_started = now;
                window_travel = 0.0f;
                if (stable_windows < COG_CAL_STABLE_WINDOWS) { continue; }
                collecting = true;
                g_cogging_debug.collecting = true;
                started = now; /* 完整三圈拥有独立的扫描时间，不扣除启动/换向等待。 */
                delta = 0.0f;
            }
            travel += delta * config->encoder_direction * (pass == 0U ? 1.0f : -1.0f);
            i = (unsigned int)(sample.mechanical_angle_rad * (COGGING_POINTS / MOTOR_CALIBRATION_TWO_PI) + 0.5f) % COGGING_POINTS;
            sums[i] += iq;
            ++counts[i];
            g_cogging_debug.revolutions = travel / MOTOR_CALIBRATION_TWO_PI;
        }
        g_cogging_debug.min_count = UINT16_MAX;
        for (i = 0U; i < COGGING_POINTS; ++i)
        {
            float value;
            /* 整理表时闭环仍在运行，不能长时间中断前台编码器调度。 */
            if (i % 32U == 0U)
            {
                App_Poll();
                if ((App_RunningAxes() & (1U << axis)) == 0U)
                {
                    result = COGGING_CAL_FEEDBACK; goto done;
                }
            }
            if (counts[i] < g_cogging_debug.min_count) { g_cogging_debug.min_count = counts[i]; }
            if (counts[i] < 3U) { result = COGGING_CAL_COVERAGE; goto done; }
            value = sums[i] / counts[i];
            if (!isfinite(value) || fabsf(value) >= config->current_limit_a * 0.99f)
            {
                result = COGGING_CAL_CURRENT; goto done;
            }
            if (pass != 0U) { value = 0.5f * (table[i] * COGGING_SCALE_A + value); }
            table[i] = (int16_t)lroundf(value / COGGING_SCALE_A);
            if (pass != 0U) { mean += table[i] * COGGING_SCALE_A / COGGING_POINTS; }
        }
    }
done:
    /* 扫描已结束，先正常停机，再处理最终去均值和后续EEPROM保存。 */
    App_DisableOutputs((uint8_t)(1U << axis));
    if (result == COGGING_CAL_OK)
    {
        for (i = 0U; i < COGGING_POINTS; ++i)
        {
            float value = table[i] * COGGING_SCALE_A - mean;
            if (fabsf(value) > config->current_limit_a) { result = COGGING_CAL_CURRENT; break; }
            table[i] = (int16_t)lroundf(value / COGGING_SCALE_A);
        }
    }
    g_cogging_debug.result = result;
    return result;
}
