/** @file app.h */
#ifndef APP_H
#define APP_H

#include <stdint.h>

#define APP_BUS_MIN_V 8.0f
#define APP_BUS_MAX_V 28.0f

void App_Init(void);
void App_Poll(void);
/* 显式开启已保存且校准的轴；反馈正常且硬件使能成功后解除本轴软件锁存。
 * 不清除硬件Break或采样层故障，返回实际运行轴位掩码。
 */
uint8_t App_EnableOutputs(uint8_t axes);
/* 标定内部专用：找零结果已装入控制器、旧EEPROM记录已失效。
 * 允许尚未最终保存的零位参与齿槽闭环，仍检查反馈、电流、电压和硬件许可。
 */
uint8_t App_EnableCalibration(unsigned int axis);
/* 正常停机并取消待恢复请求，保持 ADC 和系统中断运行。 */
void App_DisableOutputs(uint8_t axes);
uint8_t App_RunningAxes(void);
uint8_t App_FaultAxes(void);
/* 已关断轴的应用故障原因，沿用 MOTOR_FAULT_* 位，成功使能后清零。 */
uint32_t App_FaultReason(unsigned int axis);

#endif
