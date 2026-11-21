/**
 * @file adc_monitor.h
 * @brief 母线电压与两路 NTC 温度监测。
 */
#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/** @brief DMA 尚未启动时清空缓存并发起首次采样；在上电或电流 ADC 初始化之后调用。 */
void AdcMonitor_Init(void);
/* 仅在两轴均未运行、即将重新初始化电流ADC时调用，会停止共享ADC及母线DMA。 */
bool AdcMonitor_Stop(void);
/** @brief 前台周期服务：每 10 ms 发起一次单次 DMA，完成回调更新缓存，不等待转换完成。 */
void AdcMonitor_Service(void);
/** @brief 返回最近的母线电压，单位 V；无结果或超过 30 ms 未更新时返回 false，不改输出。 */
bool AdcMonitor_ReadBusVoltage(float *voltage_v);
/** @brief 预留两路 NTC 原始 ADC 读取，axis 为 0/1；尚未接入，返回 false 且不改输出。 */
bool AdcMonitor_ReadNtcRaw(unsigned int axis, uint16_t *raw);

#endif
