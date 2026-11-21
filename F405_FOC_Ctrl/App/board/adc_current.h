/** @file adc_current.h
 *  @brief 电机硬件层内部的双 ADC 注入采样后端。
 */
#ifndef ADC_CURRENT_H
#define ADC_CURRENT_H

#include "motor_hw.h"

#define ADC_CURRENT_TIMEOUT_US 150U

/** 初始化结果保留供调试器读取；失败后不被退出校准操作覆盖。 */
typedef enum
{
    ADC_CURRENT_INIT_NOT_STARTED = 0,
    ADC_CURRENT_INIT_OK,
    ADC_CURRENT_INIT_CLOCK_ERROR,
    ADC_CURRENT_INIT_CALIBRATION_ENABLE_ERROR,
    ADC_CURRENT_INIT_SLAVE_START_ERROR,
    ADC_CURRENT_INIT_MASTER_START_ERROR,
    ADC_CURRENT_INIT_MASTER_TIMEOUT,
    ADC_CURRENT_INIT_SLAVE_TIMEOUT,
    ADC_CURRENT_INIT_OFFSET_ERROR,
    ADC_CURRENT_INIT_CALIBRATION_RESTORE_ERROR,
    ADC_CURRENT_INIT_TRIGGER_START_ERROR
} AdcCurrentInitResult;

extern volatile AdcCurrentInitResult g_adc_current_init_result;
extern volatile AdcCurrentInitResult g_adc_current_axis_result[MOTOR_AXIS_COUNT];

typedef struct
{
    uint16_t current[MOTOR_AXIS_COUNT][2];
    uint32_t sample_time_us[MOTOR_AXIS_COUNT]; /* 按 TIM1 触发相位估计两 Rank 的采样保持时刻。 */
    uint32_t completed_us;
    uint32_t sequence;
    uint32_t faults;
} AdcCurrentFrame;

/**
 * @brief 按 axis_mask 校准候选轴，启动持续采样，返回校准成功的轴位掩码。
 * @note 仅在控制暂停、母线 DMA 未运行时从前台调用；DRV8303 已唤醒，PWM 计数器停止、MOE 关闭。
 *       不要求电机静止，offset 指向有效的两轴零偏数组。
 *       校准结束恢复正常电流输入；单轴失败仅剔除该轴，共享 ADC 失败返回 0。
 *       init_result 报告共享后端结果，axis_result 分别报告两轴校准结果。
 */
uint8_t AdcCurrent_Init(uint8_t axis_mask, float offset[MOTOR_AXIS_COUNT][2]);
/**
 * @brief 非阻塞复制完整电流帧；失败时不改输出，成功后检查 faults。
 * @note 每轴一个串行读取者，每个电流周期读取一次；同一帧再次读取会标记 STALE。
 *       从前台或优先级数值 >= ADC_IRQn（当前为 1）的中断调用，不能从 Break IRQ 调用。
 */
bool AdcCurrent_Read(unsigned int axis, AdcCurrentFrame *frame);
/** @brief 诊断读取，不修改控制读取的消费序号；仍检查采样错误和过期，调用优先级同 Read。 */
bool AdcCurrent_Peek(AdcCurrentFrame *frame);
/** @brief 应用实现的数据就绪回调，在 ADC IRQ 发布完整电流帧后调用，不得阻塞。 */
void AdcCurrent_OnSample(void);

#endif
