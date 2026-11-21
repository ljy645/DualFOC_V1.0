/**
 * @file drv8303.h
 * @brief 双路 DRV8303 配置与故障读取。
 */
#ifndef DRV8303_H
#define DRV8303_H

#include <stdbool.h>

/**
 * @brief 设置电流放大增益为 10/20/40/80 倍，保留其他配置并读回确认。
 * @note 驱动器已唤醒、PWM 输出关闭时从前台调用；设置后重新校准零偏。
 */
bool Drv8303_SetCurrentGain(unsigned int axis, unsigned int gain);

/**
 * @brief 启动检查：6-PWM 模式、两路校准关闭且状态寄存器 1 无故障。
 * @note SPI2 独占、16 位 mode 1；EN_GATE 已拉高至少 10 ms，PWM 输出关闭。
 *       仅初始化时阻塞调用，不在控制中断调用；不修改保护寄存器。
 */
bool Drv8303_CheckStartup(unsigned int axis);

/** @brief 同时设置两通道内部短接校准，保留其他寄存器位并读回确认；仅前台调用。 */
bool Drv8303_SetCalibration(unsigned int axis, bool enable);

#endif
