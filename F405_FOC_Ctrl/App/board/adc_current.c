/** @file adc_current.c
 *  @brief ADC1/ADC2 同步注入两 Rank，采集两轴相电流。
 */
#include "adc_current.h"
#include "board_critical.h"
#include "drv8303.h"
#include "adc.h"
#include "tim.h"
#include <string.h>

#define MOTOR_ADC_ZERO_SAMPLES 256U

static AdcCurrentFrame latest;
static uint32_t consumed[MOTOR_AXIS_COUNT];
static bool running;
static bool have_frame;
static volatile uint32_t adc_faults;
static uint32_t pwm_ticks_per_us;
volatile AdcCurrentInitResult g_adc_current_init_result = ADC_CURRENT_INIT_NOT_STARTED;
volatile AdcCurrentInitResult g_adc_current_axis_result[MOTOR_AXIS_COUNT];

/** @brief 初始化采样状态、内部短接校准零偏，随后进入持续采样。 */
uint8_t AdcCurrent_Init(uint8_t axis_mask, float offset[MOTOR_AXIS_COUNT][2])
{
    uint32_t sums[MOTOR_AXIS_COUNT][2] = {{0U, 0U}, {0U, 0U}};
    uint32_t trigger = hadc1.Instance->CR2 & ADC_CR2_JEXTEN;
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    unsigned int sample;
    unsigned int axis;
    unsigned int phase;
    uint8_t calibrated = axis_mask;
    bool ok = false;

    memset(offset, 0, sizeof(float) * MOTOR_AXIS_COUNT * 2U);
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        g_adc_current_axis_result[axis] = ADC_CURRENT_INIT_NOT_STARTED;
    }
    g_adc_current_init_result = ADC_CURRENT_INIT_NOT_STARTED;
    if (axis_mask == 0U) { return 0U; }

    g_adc_current_init_result = ADC_CURRENT_INIT_CLOCK_ERROR;

    /* 先确认微秒时基可配置，避免校准完成后才因时钟配置退出。 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) { timer_clock *= 2U; }
    if (timer_clock < 1000000U || timer_clock % 1000000U != 0U) { return 0U; }
    pwm_ticks_per_us = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) { pwm_ticks_per_us *= 2U; }
    pwm_ticks_per_us /= (htim1.Instance->PSC + 1U) * 1000000U;
    if (pwm_ticks_per_us == 0U) { return 0U; }
    g_adc_current_init_result = ADC_CURRENT_INIT_NOT_STARTED;

    /* 初始化时控制暂停；先关闭本模块中断源，再重置缓存。 */
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOC);
    running = have_frame = false;
    adc_faults = 0U;
    memset(&latest, 0, sizeof(latest));
    memset(consumed, 0, sizeof(consumed));
    (void)HAL_ADC_Stop(&hadc1);
    (void)HAL_ADC_Stop(&hadc2);

    /* 1. 仅对已装配、供电的驱动器内部短接，未启用轴不访问 SPI。 */
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((axis_mask & (1U << axis)) == 0U) { continue; }
        if (!Drv8303_SetCalibration(axis, true))
        {
            g_adc_current_axis_result[axis] = ADC_CURRENT_INIT_CALIBRATION_ENABLE_ERROR;
            calibrated &= (uint8_t)~(1U << axis);
        }
    }
    if (calibrated == 0U) { goto restore; }
    HAL_Delay(1U);

    /* 2. 暂用软件触发双 ADC，取 256 帧平均值作为各通道零偏。 */
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_JEXTEN);
    CLEAR_BIT(hadc1.Instance->CR1, ADC_CR1_JAUTO);
    CLEAR_BIT(hadc2.Instance->CR1, ADC_CR1_JAUTO);
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        g_adc_current_init_result = ADC_CURRENT_INIT_SLAVE_START_ERROR;
        goto restore;
    }
    for (sample = 0U; sample < MOTOR_ADC_ZERO_SAMPLES; ++sample)
    {
        if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
        {
            g_adc_current_init_result = ADC_CURRENT_INIT_MASTER_START_ERROR;
            goto restore;
        }
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 2U) != HAL_OK)
        {
            g_adc_current_init_result = ADC_CURRENT_INIT_MASTER_TIMEOUT;
            goto restore;
        }
        if (HAL_ADCEx_InjectedPollForConversion(&hadc2, 2U) != HAL_OK)
        {
            g_adc_current_init_result = ADC_CURRENT_INIT_SLAVE_TIMEOUT;
            goto restore;
        }
        sums[0][0] += hadc1.Instance->JDR1;
        sums[0][1] += hadc2.Instance->JDR1;
        sums[1][0] += hadc1.Instance->JDR2;
        sums[1][1] += hadc2.Instance->JDR2;
    }
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((calibrated & (1U << axis)) == 0U) { continue; }
        for (phase = 0U; phase < 2U; ++phase)
        {
            offset[axis][phase] = (float)sums[axis][phase] / MOTOR_ADC_ZERO_SAMPLES;
            /* REF 与 ADC 参考同为 A3V3，零偏应接近半量程。 */
            if (offset[axis][phase] < 1536.0f || offset[axis][phase] > 2560.0f)
            {
                g_adc_current_axis_result[axis] = ADC_CURRENT_INIT_OFFSET_ERROR;
                calibrated &= (uint8_t)~(1U << axis);
            }
        }
        if ((calibrated & (1U << axis)) != 0U)
        {
            g_adc_current_axis_result[axis] = ADC_CURRENT_INIT_OK;
        }
    }
    ok = true;
restore:
    /* 3. 成功或失败都尝试退出所有已启用轴的短接。 */
    (void)HAL_ADC_Stop(&hadc1);
    (void)HAL_ADC_Stop(&hadc2);
    MODIFY_REG(hadc1.Instance->CR2, ADC_CR2_JEXTEN, trigger);
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((axis_mask & (1U << axis)) == 0U) { continue; }
        if (!Drv8303_SetCalibration(axis, false))
        {
            if (g_adc_current_axis_result[axis] == ADC_CURRENT_INIT_OK ||
                g_adc_current_axis_result[axis] == ADC_CURRENT_INIT_NOT_STARTED)
            {
                g_adc_current_axis_result[axis] = ADC_CURRENT_INIT_CALIBRATION_RESTORE_ERROR;
            }
            calibrated &= (uint8_t)~(1U << axis);
        }
    }
    if (!ok) { return 0U; }
    if (calibrated == 0U)
    {
        g_adc_current_init_result = ADC_CURRENT_INIT_NOT_STARTED;
        return 0U;
    }
    HAL_Delay(1U);

    /* 4. 校准完成，恢复定时器触发电流采样。 */
    /* TIM5 在本工程未被其他功能使用，专用于 32 位单调微秒时基。 */
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM5_FORCE_RESET();
    __HAL_RCC_TIM5_RELEASE_RESET();
    TIM5->PSC = timer_clock / 1000000U - 1U;
    TIM5->ARR = 0xFFFFFFFFU;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CNT = 0U;
    TIM5->CR1 = TIM_CR1_CEN;

    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        g_adc_current_init_result = ADC_CURRENT_INIT_TRIGGER_START_ERROR;
        __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
        (void)HAL_ADC_Stop(&hadc1);
        (void)HAL_ADC_Stop(&hadc2);
        return 0U;
    }
    running = true;
    g_adc_current_init_result = ADC_CURRENT_INIT_OK;
    return calibrated;
}

/** @brief 发布完整电流帧，再通知应用执行本拍控制；不进行阻塞式外设操作。 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t now;
    uint32_t period;
    uint32_t phase;
    uint32_t elapsed;
    uint32_t trigger_us;
    if (hadc != &hadc1 || !running) { return; }
    /* 唯一发布者为 ADC IRQ（优先级 1）；读取者仅允许前台或优先级不高于它的中断。 */
    now = TIM5->CNT;
    /* 从中心对齐计数相位回推最近的 OC4REF 上升沿，消除大部分 ISR 入口延迟。 */
    phase = htim1.Instance->CNT;
    period = 2U * htim1.Instance->ARR;
    if ((htim1.Instance->CR1 & TIM_CR1_DIR) != 0U) { phase = period - phase; }
    elapsed = (phase + period - htim1.Instance->CCR4) % period;
    trigger_us = now - elapsed / pwm_ticks_per_us;
    if (!__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOC))
    {
        adc_faults |= MOTOR_FAULT_MEASUREMENT;
        return;
    }
    latest.current[0][0] = (uint16_t)hadc1.Instance->JDR1;
    latest.current[0][1] = (uint16_t)hadc2.Instance->JDR1;
    latest.current[1][0] = (uint16_t)hadc1.Instance->JDR2;
    latest.current[1][1] = (uint16_t)hadc2.Instance->JDR2;
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JSTRT);
    latest.completed_us = now;
    /* ADC 21 MHz、采样 28 周期、转换 12 周期：两 Rank 保持时刻约为
     * 触发后 1.33/3.24 us，按当前微秒时基取整。 */
    latest.sample_time_us[0] = trigger_us + 1U;
    latest.sample_time_us[1] = trigger_us + 3U;
    ++latest.sequence;
    have_frame = true;
    AdcCurrent_OnSample();
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1 || hadc == &hadc2)
    {
        /* 各错误路径只设置同一位；初始化清零只在控制暂停时进行。 */
        adc_faults |= MOTOR_FAULT_MEASUREMENT;
    }
}

bool AdcCurrent_Peek(AdcCurrentFrame *frame)
{
    uint32_t saved;
    if (frame == 0) { return false; }
    saved = Board_EnterCritical();
    if (!have_frame)
    {
        Board_ExitCritical(saved);
        return false;
    }
    *frame = latest;
    frame->faults |= adc_faults;
    Board_ExitCritical(saved);
    if ((uint32_t)(TIM5->CNT - frame->completed_us) > ADC_CURRENT_TIMEOUT_US)
    {
        frame->faults |= MOTOR_FAULT_STALE;
    }
    return true;
}

bool AdcCurrent_Read(unsigned int axis, AdcCurrentFrame *frame)
{
    if (axis >= MOTOR_AXIS_COUNT || !AdcCurrent_Peek(frame)) { return false; }
    if (consumed[axis] == frame->sequence) { frame->faults |= MOTOR_FAULT_STALE; }
    consumed[axis] = frame->sequence;
    return true;
}
