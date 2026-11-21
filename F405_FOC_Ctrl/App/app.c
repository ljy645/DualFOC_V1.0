/**
 * @file app.c
 * @brief 双轴独立启动、控制和故障关断。
 */
#include "app.h"
#include "motor_control.h"
#include "enc_mt6826s.h"
#include "adc_monitor.h"
#include "adc_current.h"
#include "usb_hostif.h"
#include "main.h"
#include "board_critical.h"
#include "motor_calibration.h"
#include "config_store.h"
#include <math.h>
#include <string.h>

#define APP_STARTUP_TIMEOUT_MS 10U
#define APP_BUS_RECOVER_MIN_V 9.0f
#define APP_BUS_RECOVER_MAX_V 27.0f
#define APP_BUS_STABLE_MS 1000U
#define APP_CURRENT_RETRY_MS 2000U
#define APP_CURRENT_ZERO_A 0.2f
#define APP_CURRENT_TRIP_RATIO 1.25f
#define APP_CURRENT_TRIP_MARGIN_A 0.2f
#define APP_ENCODER_PERIOD_US 200U /* 5 kHz 请求有效轴，SPI DMA 异步读取。 */

/* 运行请求与可用轴分开；可用轴在停机时也持续采集编码器。 */
static volatile uint8_t feedback_mask;
static volatile uint8_t control_running;
static volatile uint8_t fault_mask; /* 仅记录异常关断，初始化时跳过的轴不计入。 */
static uint8_t available_axes; /* 停机时仍采集编码器，供双轴上位机显示位置。 */
static uint32_t encoder_time_us;
static uint32_t startup_healthy_ms;
static bool startup_healthy;
static volatile uint8_t recovery_mask;
static struct
{
    volatile uint32_t reason;
    volatile uint32_t stopped_ms;
    uint32_t healthy_ms;
    volatile bool healthy;
    volatile bool current_retried;
} recovery[MOTOR_AXIS_COUNT];

/* 电源后接入或上电电压异常时先等待，不把一次初始化失败变成永久不可用。 */
static void App_ServiceStartup(void)
{
    float voltage;
    uint32_t now;
    if (available_axes != 0U || MotorCalibration_ActiveAxis() >= 0) { return; }
    if (!AdcMonitor_ReadBusVoltage(&voltage) ||
        voltage < APP_BUS_RECOVER_MIN_V || voltage > APP_BUS_RECOVER_MAX_V)
    {
        startup_healthy = false;
        return;
    }
    now = HAL_GetTick();
    if (!startup_healthy)
    {
        startup_healthy_ms = now;
        startup_healthy = true;
    }
    if ((uint32_t)(now - startup_healthy_ms) < APP_BUS_STABLE_MS) { return; }
    startup_healthy = false;
    /* 零偏校准会重配共享ADC，先结束母线DMA，完成后重新启动母线监测。 */
    if (AdcMonitor_Stop())
    {
        available_axes = MotorHw_Init();
        encoder_time_us = TIM5->CNT - APP_ENCODER_PERIOD_US;
    }
    AdcMonitor_Init();
}

static void App_ServiceFeedback(void)
{
    uint8_t requested = feedback_mask | available_axes;
    int calibration_axis = MotorCalibration_ActiveAxis();
    uint32_t now;
    if (calibration_axis >= 0) { requested |= (uint8_t)(1U << calibration_axis); }
    EncMt6826s_Service();
    now = TIM5->CNT;
    if (requested != 0U && (uint32_t)(now - encoder_time_us) >= APP_ENCODER_PERIOD_US)
    {
        /* BUSY 表示尚未受理，下一轮前台继续提交，不能漏掉这一周期的请求。 */
        if (EncMt6826s_RequestMask(requested) != ENC_MT6826S_BUSY) { encoder_time_us = now; }
    }
    AdcMonitor_Service();
}

/* 指令限幅与实测跳闸分开，留出电流环瞬态及采样噪声余量。 */
static uint32_t App_SampleFaults(unsigned int axis, const MotorHwSample *sample)
{
    float limit = MotorControl_GetConfig(axis)->current_limit_a * APP_CURRENT_TRIP_RATIO +
                  APP_CURRENT_TRIP_MARGIN_A;
    uint32_t faults = sample->faults;
    if (!sample->feedback_complete) { faults |= MOTOR_FAULT_ENCODER; }
    if (faults != 0U) { return faults; }
    if (sample->vbus_v < APP_BUS_MIN_V || sample->vbus_v > APP_BUS_MAX_V)
    {
        faults |= MOTOR_FAULT_BUS_VOLTAGE;
    }
    if (fabsf(sample->ia_a) >= limit || fabsf(sample->ib_a) >= limit ||
        fabsf(sample->ia_a + sample->ib_a) >= limit) { faults |= MOTOR_FAULT_OVERCURRENT; }
    return faults;
}

/** @brief 关断本轴并记录原因；仅运行中的电压/软件过流故障安排恢复。 */
static void App_StopAxis(unsigned int axis, uint32_t reason)
{
    uint8_t bit = (uint8_t)(1U << axis);
    uint32_t saved = Board_EnterCritical();
    /* ADC可能已先于前台关断，不能覆盖首次原因和重试计时。 */
    if ((control_running & bit) == 0U) { Board_ExitCritical(saved); return; }
    recovery[axis].reason = reason;
    recovery[axis].stopped_ms = HAL_GetTick();
    recovery[axis].healthy = false;
    if (reason == MOTOR_FAULT_BUS_VOLTAGE ||
        (reason == MOTOR_FAULT_OVERCURRENT && !recovery[axis].current_retried))
    {
        recovery_mask |= bit;
    }
    fault_mask |= (uint8_t)(1U << axis);
    control_running &= (uint8_t)~(1U << axis);
    feedback_mask &= (uint8_t)~(1U << axis);
    (void)MotorHw_SetOutputEnabled(axis, false);
    MotorControl_Reset(axis);
    Board_ExitCritical(saved);
}

/** @brief 各轴独立等待完整反馈与 PWM 更新，超时轴保持关闭。 */
static uint8_t App_StartOutputs(uint8_t axes, bool for_calibration)
{
    unsigned int axis;
    uint8_t requested = axes & ((1U << MOTOR_AXIS_COUNT) - 1U);
    uint8_t pending;
    uint8_t enabled;
    uint32_t started;
    uint32_t saved;
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        const ConfigState *state = ConfigStore_Get(axis);
        if (!state->saved || state->dirty || (!for_calibration && !state->params.calibrated))
        {
            requested &= (uint8_t)~(1U << axis);
        }
    }
    saved = Board_EnterCritical();
    pending = requested & (uint8_t)~control_running;
    feedback_mask |= pending;
    Board_ExitCritical(saved);
    started = HAL_GetTick();
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((pending & (1U << axis)) != 0U && !MotorHw_WriteDuty(axis, 0.5f, 0.5f, 0.5f))
        {
            pending &= (uint8_t)~(1U << axis);
        }
    }
    while (pending != 0U && (uint32_t)(HAL_GetTick() - started) < APP_STARTUP_TIMEOUT_MS)
    {
        App_ServiceFeedback();
        for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
        {
            MotorHwSample sample;
            uint32_t saved;
            if ((pending & (1U << axis)) == 0U) { continue; }
            if (!MotorHw_PeekSample(axis, &sample) || App_SampleFaults(axis, &sample) != 0U) { continue; }
            /* MOE 和运行位一起发布，避免 ADC 中断看见半完成的启动。 */
            saved = Board_EnterCritical();
            if (MotorHw_SetOutputEnabled(axis, true))
            {
                /* 反馈正常且硬件使能成功后解除本轴软件锁存；不清硬件Break。 */
                fault_mask &= (uint8_t)~(1U << axis);
                recovery[axis].reason = 0U;
                control_running |= (uint8_t)(1U << axis);
                pending &= (uint8_t)~(1U << axis);
            }
            Board_ExitCritical(saved);
        }
    }
    saved = Board_EnterCritical();
    feedback_mask = control_running;
    enabled = control_running & requested;
    Board_ExitCritical(saved);
    return enabled;
}

/* 只有手动使能重置过流重试次数，自动恢复不调用这个入口。 */
uint8_t App_EnableOutputs(uint8_t axes)
{
    unsigned int axis;
    uint32_t saved = Board_EnterCritical();
    recovery_mask &= (uint8_t)~axes;
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((axes & (1U << axis)) != 0U) { recovery[axis].current_retried = false; }
    }
    Board_ExitCritical(saved);
    return App_StartOutputs(axes, false);
}

uint8_t App_EnableCalibration(unsigned int axis)
{
    return App_StartOutputs((uint8_t)(1U << axis), true);
}

/* 前台检查恢复；不清硬件Break，不恢复旧目标，不循环尝试失败的启动。 */
static void App_ServiceRecovery(void)
{
    unsigned int axis;
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        uint8_t bit = (uint8_t)(1U << axis);
        MotorHwSample sample;
        uint32_t now, saved;
        bool ready;
        if ((recovery_mask & bit) == 0U) { continue; }
        ready = MotorHw_PeekSample(axis, &sample) && App_SampleFaults(axis, &sample) == 0U;
        now = HAL_GetTick();
        saved = Board_EnterCritical();
        if (recovery[axis].reason == MOTOR_FAULT_BUS_VOLTAGE)
        {
            ready = ready && sample.vbus_v >= APP_BUS_RECOVER_MIN_V && sample.vbus_v <= APP_BUS_RECOVER_MAX_V;
            if (!ready) { recovery[axis].healthy = false; }
            else if (!recovery[axis].healthy)
            {
                recovery[axis].healthy_ms = now;
                recovery[axis].healthy = true;
            }
            ready = ready && (uint32_t)(now - recovery[axis].healthy_ms) >= APP_BUS_STABLE_MS;
        }
        else
        {
            ready = ready && (uint32_t)(now - recovery[axis].stopped_ms) >= APP_CURRENT_RETRY_MS &&
                    fabsf(sample.ia_a) < APP_CURRENT_ZERO_A && fabsf(sample.ib_a) < APP_CURRENT_ZERO_A &&
                    fabsf(sample.ia_a + sample.ib_a) < APP_CURRENT_ZERO_A;
        }
        if (ready)
        {
            recovery_mask &= (uint8_t)~bit;
            if (recovery[axis].reason == MOTOR_FAULT_OVERCURRENT) { recovery[axis].current_retried = true; }
            MotorControl_Reset(axis);
        }
        Board_ExitCritical(saved);
        if (ready) { (void)App_StartOutputs(bit, false); }
    }
}

uint8_t App_RunningAxes(void) { return control_running; }
uint8_t App_FaultAxes(void) { return fault_mask; }
uint32_t App_FaultReason(unsigned int axis) { return recovery[axis].reason; }

void App_DisableOutputs(uint8_t axes)
{
    unsigned int axis;
    uint32_t saved = Board_EnterCritical();
    recovery_mask &= (uint8_t)~axes;
    control_running &= (uint8_t)~axes;
    feedback_mask &= (uint8_t)~axes;
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((axes & (1U << axis)) != 0U)
        {
            (void)MotorHw_SetOutputEnabled(axis, false);
            MotorControl_Reset(axis);
        }
    }
    Board_ExitCritical(saved);
}

void App_Init(void)
{
    control_running = 0U;
    fault_mask = 0U;
    feedback_mask = 0U;
    recovery_mask = 0U;
    memset(recovery, 0, sizeof(recovery));
    MotorControl_Init();
    ConfigStore_Init();
    available_axes = 0U;
    startup_healthy = false;
    encoder_time_us = 0U;
    /* 先测母线，前台在电压稳定后初始化驱动器和电流采样。 */
    AdcMonitor_Init();
    /* 上电只加载配置与测量，收到 enable 命令才开启功率输出。 */
}

/** @brief 20 kHz 回调只计算已运行轴，单轴故障只关闭该轴。 */
void AdcCurrent_OnSample(void)
{
    unsigned int axis;
    fault_mask |= MotorCalibration_Monitor(fault_mask);
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        MotorHwSample sample;
        FocOutput output;
        if (MotorCalibration_ActiveAxis() == (int)axis)
        {
            control_running &= (uint8_t)~(1U << axis);
            feedback_mask &= (uint8_t)~(1U << axis);
            continue;
        }
        if ((control_running & (1U << axis)) == 0U) { continue; }
        {
            uint32_t faults = MotorHw_ReadSample(axis, &sample) ?
                              App_SampleFaults(axis, &sample) : MOTOR_FAULT_MEASUREMENT;
            if (faults != 0U)
            {
                App_StopAxis(axis, faults);
                continue;
            }
        }
        MotorControl_Step(axis, &sample, &output);
        if (!MotorHw_WriteDuty(axis, output.duty_a, output.duty_b, output.duty_c))
        {
            App_StopAxis(axis, MOTOR_FAULT_OUTPUT);
        }
    }
}

void App_Poll(void)
{
    unsigned int axis;
    float bus_voltage;
    bool bus_ready;
    bool red;
    bool green;
    uint32_t saved;
    uint8_t calibration_faults;
    int calibration_axis;
    App_ServiceFeedback();
    App_ServiceStartup();
    calibration_faults = MotorCalibration_Monitor(fault_mask);
    saved = Board_EnterCritical();
    fault_mask |= calibration_faults;
    calibration_axis = MotorCalibration_ActiveAxis();
    if (calibration_axis >= 0)
    {
        uint8_t axis_bit = (uint8_t)(1U << calibration_axis);
        control_running &= (uint8_t)~axis_bit;
        feedback_mask &= (uint8_t)~axis_bit;
    }
    Board_ExitCritical(saved);
    /* ADC 中断停止时仍能从前台发现过期反馈并关闭对应输出。 */
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        MotorHwSample sample;
        if ((control_running & (1U << axis)) != 0U)
        {
            uint32_t faults = MotorHw_PeekSample(axis, &sample) ?
                              App_SampleFaults(axis, &sample) : MOTOR_FAULT_MEASUREMENT;
            if (faults != 0U) { App_StopAxis(axis, faults); }
        }
    }
    bus_ready = AdcMonitor_ReadBusVoltage(&bus_voltage);
    red = fault_mask == ((1U << MOTOR_AXIS_COUNT) - 1U) ||
          (bus_ready && (bus_voltage < APP_BUS_MIN_V || bus_voltage > APP_BUS_MAX_V));
    green = bus_ready && !red;
    if (green && MotorCalibration_ActiveAxis() >= 0)
    {
        green = (HAL_GetTick() / 250U) % 2U == 0U;
    }
    /* LED2 绿、LED1 红，低电平点亮；无有效母线数据时不显示正常。 */
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, green ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, red ? GPIO_PIN_RESET : GPIO_PIN_SET);
    UsbVofa_Poll();
    App_ServiceRecovery();
}
