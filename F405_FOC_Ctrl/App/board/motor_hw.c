/**
 * @file motor_hw.c
 * @brief 双轴测量快照、PWM 预装载和输出使能；运行保护策略由上层管理。
 */
#include "motor_hw.h"
#include "enc_mt6826s.h"
#include "board_critical.h"
#include "adc_current.h"
#include "adc_monitor.h"
#include "drv8303.h"
#include "tim.h"
#include <math.h>

#define MOTOR_ADC_V_PER_COUNT (3.3f / 4096.0f)
#define MOTOR_SHUNT_OHM 0.005f
#define MOTOR_CURRENT_GAIN 20U
#define MOTOR_TWO_PI 6.2831853071795865f
#define MOTOR_ENCODER_TIMEOUT_US 2000U

static float current_offset[MOTOR_AXIS_COUNT][2];
static float current_scale[MOTOR_AXIS_COUNT];
static uint8_t ready_mask;
static bool duty_preloaded[MOTOR_AXIS_COUNT];

/** @brief 定时器配置固定，只在初始化时检查 PWM2、极性和预装载。 */
static bool MotorHw_PwmConfigured(const TIM_TypeDef *timer)
{
    return timer->ARR > 0U && timer->ARR <= 65535U &&
        (timer->CR1 & TIM_CR1_CMS) == TIM_COUNTERMODE_CENTERALIGNED1 &&
        (timer->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S |
                        TIM_CCMR1_OC1M | TIM_CCMR1_OC2M |
                        TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE)) ==
            (TIM_OCMODE_PWM2 | (TIM_OCMODE_PWM2 << 8U) |
             TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE) &&
        (timer->CCMR2 & (TIM_CCMR2_CC3S | TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE)) ==
            (TIM_OCMODE_PWM2 | TIM_CCMR2_OC3PE) &&
        (timer->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P |
                        TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP)) == 0U;
}

/** @brief 仅控制 MOE，保持驱动器及电流放大器唤醒，正常停机后仍可持续测量。 */
bool MotorHw_SetOutputEnabled(unsigned int axis, bool enable)
{
    TIM_HandleTypeDef *timer;
    uint32_t saved;
    bool success = false;
    if (axis >= MOTOR_AXIS_COUNT) { return false; }
    timer = axis == MOTOR_AXIS_M0 ? &htim1 : &htim8;
    saved = Board_EnterCritical();
    if (!enable)
    {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer);
        duty_preloaded[axis] = false;
        success = true;
    }
    else if ((ready_mask & (1U << axis)) != 0U && duty_preloaded[axis] &&
             (timer->Instance->CR1 & TIM_CR1_CEN) != 0U &&
             (timer->Instance->SR & TIM_SR_UIF) != 0U)
    {
        __HAL_TIM_MOE_ENABLE(timer);
        success = (timer->Instance->BDTR & TIM_BDTR_MOE) != 0U;
    }
    Board_ExitCritical(saved);
    return success;
}

/** @brief 内部短接校准并开启测量链路，MOE 始终关闭；初始化允许阻塞。 */
uint8_t MotorHw_Init(void)
{
    unsigned int axis;
    uint8_t detected = 0U;
    TIM_TypeDef *timers[2] = {htim1.Instance, htim8.Instance};
    ready_mask = 0U;
    (void)MotorHw_SetOutputEnabled(MOTOR_AXIS_M0, false);
    (void)MotorHw_SetOutputEnabled(MOTOR_AXIS_M1, false);
    HAL_GPIO_WritePin(M0_EN_GATE_GPIO_Port, M0_EN_GATE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_EN_GATE_GPIO_Port, M1_EN_GATE_Pin, GPIO_PIN_RESET);
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        TIM_TypeDef *timer = timers[axis];
        if (!MotorHw_PwmConfigured(timer)) { goto failed; }
        CLEAR_BIT(timer->CR1, TIM_CR1_CEN);
        timer->CNT = 0U;
        /* EN_GATE 高时仅唤醒放大器。MOE=0、OSSI=1、两组空闲电平为低。 */
        CLEAR_BIT(timer->CR2, TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 |
                             TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N);
        CLEAR_BIT(timer->BDTR, TIM_BDTR_AOE);
        SET_BIT(timer->BDTR, TIM_BDTR_OSSI);
        SET_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                             TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
        if (!MotorHw_WriteDuty(axis, 0.5f, 0.5f, 0.5f)) { goto failed; }
        timer->EGR = TIM_EGR_UG;
    }
    if (EncMt6826s_Init() != ENC_MT6826S_OK)
    {
        goto failed;
    }
    /* DRV8303 的电流放大器需要 EN_GATE 高；PWM 输入保持全低，不启动功率输出。 */
    HAL_GPIO_WritePin(M0_EN_GATE_GPIO_Port, M0_EN_GATE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_EN_GATE_GPIO_Port, M1_EN_GATE_Pin, GPIO_PIN_SET);
    HAL_Delay(10U);
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if (!Drv8303_SetCurrentGain(axis, MOTOR_CURRENT_GAIN) ||
            !Drv8303_CheckStartup(axis))
        {
            HAL_GPIO_WritePin(axis == MOTOR_AXIS_M0 ? M0_EN_GATE_GPIO_Port : M1_EN_GATE_GPIO_Port,
                              axis == MOTOR_AXIS_M0 ? M0_EN_GATE_Pin : M1_EN_GATE_Pin, GPIO_PIN_RESET);
            continue;
        }
        detected |= (uint8_t)(1U << axis);
        /* EN_GATE 唤醒前可能留下 Break 标志；只在驱动器自检无故障后清一次。
         * 硬件 Break 始终使能，后续故障保持锁存，不在运行中清除。 */
        __HAL_TIM_CLEAR_FLAG(axis == MOTOR_AXIS_M0 ? &htim1 : &htim8, TIM_FLAG_BREAK);
        /* SetCurrentGain 已读回确认，换算直接使用确认后的配置值。 */
        current_scale[axis] = MOTOR_ADC_V_PER_COUNT / (MOTOR_SHUNT_OHM * MOTOR_CURRENT_GAIN);
    }
    detected = AdcCurrent_Init(detected, current_offset);
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((detected & (1U << axis)) == 0U)
        {
            HAL_GPIO_WritePin(axis == MOTOR_AXIS_M0 ? M0_EN_GATE_GPIO_Port : M1_EN_GATE_GPIO_Port,
                              axis == MOTOR_AXIS_M0 ? M0_EN_GATE_Pin : M1_EN_GATE_Pin, GPIO_PIN_RESET);
        }
    }
    if (detected == 0U) { goto failed; }

    /* 先用一次 TIM1 enable 边沿同步启动 TIM8，再切回 OC4REF 作为 ADC 触发。 */
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_ENABLE);
    MODIFY_REG(htim8.Instance->SMCR, TIM_SMCR_SMS | TIM_SMCR_TS,
               TIM_SLAVEMODE_TRIGGER | TIM_TS_ITR0);
    htim1.Instance->CNT = 0U;
    htim8.Instance->CNT = 0U;
    SET_BIT(htim1.Instance->CR1, TIM_CR1_CEN);
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_OC4REF);
    /* 默认的三相 50% 仅用于初始化，上层仍需显式装载启动占空比。 */
    duty_preloaded[0] = duty_preloaded[1] = false;
    ready_mask = detected;
    return ready_mask;

failed:
    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(htim8.Instance->CR1, TIM_CR1_CEN);
    (void)MotorHw_SetOutputEnabled(MOTOR_AXIS_M0, false);
    (void)MotorHw_SetOutputEnabled(MOTOR_AXIS_M1, false);
    HAL_GPIO_WritePin(M0_EN_GATE_GPIO_Port, M0_EN_GATE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_EN_GATE_GPIO_Port, M1_EN_GATE_Pin, GPIO_PIN_RESET);
    return 0U;
}

/** @brief 组装完整反馈，并用实测机械速度将编码器角度外推到该轴电流采样时刻。 */
static bool MotorHw_GetSample(unsigned int axis, MotorHwSample *sample, bool consume)
{
    AdcCurrentFrame frame;
    EncMt6826sSample encoder;
    MotorHwSample result = {0};
    bool bus_ready;
    if (axis >= MOTOR_AXIS_COUNT || sample == 0) { return false; }
    if ((ready_mask & (1U << axis)) == 0U) { return false; }
    if (!(consume ? AdcCurrent_Read(axis, &frame) : AdcCurrent_Peek(&frame)))
    {
        return false;
    }
    /* 原理图 SP 接分流电阻地端、SN 接低侧源极，流入电机为正时 SO 上升。 */
    result.ia_a = ((float)frame.current[axis][0] - current_offset[axis][0]) * current_scale[axis];
    result.ib_a = ((float)frame.current[axis][1] - current_offset[axis][1]) * current_scale[axis];
    result.current_time_us = frame.sample_time_us[axis];
    result.faults = frame.faults;
    if (((axis == MOTOR_AXIS_M0 ? htim1.Instance->SR : htim8.Instance->SR) & TIM_SR_BIF) != 0U)
    {
        result.faults |= MOTOR_FAULT_DRIVER;
    }
    bus_ready = AdcMonitor_ReadBusVoltage(&result.vbus_v);
    if (!bus_ready) { result.faults |= MOTOR_FAULT_BUS_SAMPLE; }
    if (EncMt6826s_ReadLatest(axis, &encoder) == ENC_MT6826S_OK &&
        (uint32_t)(TIM5->CNT - encoder.time_us) < MOTOR_ENCODER_TIMEOUT_US)
    {
        int32_t dt_us = (int32_t)(result.current_time_us - encoder.time_us);
        float angle = encoder.angle_rad + encoder.speed_rad_s * (float)dt_us * 1.0e-6f;
        angle = fmodf(angle, MOTOR_TWO_PI);
        result.mechanical_angle_rad = angle < 0.0f ? angle + MOTOR_TWO_PI : angle;
        if (result.mechanical_angle_rad >= MOTOR_TWO_PI) { result.mechanical_angle_rad = 0.0f; }
        result.mechanical_speed_rad_s = encoder.speed_rad_s;
        result.feedback_complete = bus_ready && encoder.speed_valid;
    }
    else
    {
        result.faults |= MOTOR_FAULT_ENCODER;
    }
    *sample = result;
    return true;
}

bool MotorHw_ReadSample(unsigned int axis, MotorHwSample *sample)
{
    return MotorHw_GetSample(axis, sample, true);
}

bool MotorHw_PeekSample(unsigned int axis, MotorHwSample *sample)
{
    return MotorHw_GetSample(axis, sample, false);
}

/** @brief 按原理图的 CH3N=A、CH2N=B、CH1N=C 写入高侧导通比例。 */
bool MotorHw_WriteDuty(unsigned int axis, float duty_a, float duty_b, float duty_c)
{
    TIM_HandleTypeDef *handle;
    TIM_TypeDef *timer;
    uint32_t period;
    uint32_t compare_a;
    uint32_t compare_b;
    uint32_t compare_c;
    uint32_t saved_mask;
    uint32_t saved_udis;

    if (axis >= MOTOR_AXIS_COUNT ||
        !(duty_a >= 0.0f && duty_a <= 1.0f) ||
        !(duty_b >= 0.0f && duty_b <= 1.0f) ||
        !(duty_c >= 0.0f && duty_c <= 1.0f))
    {
        return false;
    }
    handle = (axis == MOTOR_AXIS_M0) ? &htim1 : &htim8;
    timer = handle->Instance;
    /* 外设已初始化，固定的 ARR 和 PWM 配置已在 Init 中检查。 */
    period = timer->ARR;
    /* 高侧接 PWM2 的互补输出，理想导通比例为 CCR/ARR；死区由定时器插入。 */
    compare_a = (uint32_t)(duty_a * (float)period + 0.5f);
    compare_b = (uint32_t)(duty_b * (float)period + 0.5f);
    compare_c = (uint32_t)(duty_c * (float)period + 0.5f);
    saved_mask = Board_EnterCritical();
    saved_udis = timer->CR1 & TIM_CR1_UDIS;
    /* 关中断不能阻止定时器更新；短暂禁止更新，避免三相分属不同周期。 */
    timer->CR1 |= TIM_CR1_UDIS;
    __HAL_TIM_SET_COMPARE(handle, TIM_CHANNEL_1, compare_c);
    __HAL_TIM_SET_COMPARE(handle, TIM_CHANNEL_2, compare_b);
    __HAL_TIM_SET_COMPARE(handle, TIM_CHANNEL_3, compare_a);
    /* 开启前必须发生一次预装载更新；不产生 UG，不打乱双轴同步。 */
    if ((timer->BDTR & TIM_BDTR_MOE) == 0U) { __HAL_TIM_CLEAR_FLAG(handle, TIM_FLAG_UPDATE); }
    timer->CR1 = (timer->CR1 & ~TIM_CR1_UDIS) | saved_udis;
    duty_preloaded[axis] = true;
    Board_ExitCritical(saved_mask);
    return true;
}

