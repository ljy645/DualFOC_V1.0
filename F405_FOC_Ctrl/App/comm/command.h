/** @file command.h */
#ifndef COMMAND_H
#define COMMAND_H

#include "motor_calibration.h"
#include <stddef.h>

/** @brief 自动标定指定轴的编码器方向和电角零位，仅在 App_Init 完成后从前台调用。
 * @note 使用该轴已提交的极对数和控制参数，保存结果前先持久化清零校准有效位。
 *       暂停两轴正常控制，等待期间持续执行 App_Poll，中断保持开启；不从中断调用。
 *       一次只标定一轴，电机须空载、可自由转动；不会自动探测电机允许电压/电流。
 *       调用前两轴须已停机；成功后保存方向/零位，保持关闭，等待显式 enable 命令。
 *       未保存的参数须先 param save；EEPROM 失败返回 STORAGE_ERROR，保持输出关闭。
 *       故障锁存轴须排除原因并重新初始化后才能标定；本函数不清硬件保护。
 */
MotorCalibrationResult Command_CalibrateMotor(unsigned int axis);
typedef enum { COMMAND_CAL_IDLE = 0, COMMAND_CAL_ZERO, COMMAND_CAL_COGGING } CommandCalibrationStage;
/* 完整校准：先持久化失效旧记录，找零结果留RAM，齿槽成功后统一保存零位及补偿表。
 * 回复包含失败阶段；开始前两轴已停止，结束保持关闭；中途失败不能作为完整校准使用。
 * USB calibrate <axis> 使用此入口；Command_CalibrateMotor 保留为仅找零的内部/调试入口。
 */
void Command_CalibrateFull(unsigned int axis, char *reply, size_t capacity);
/* 实际执行阶段，供前台 USB 发布进度，不是额外的运行许可。 */
CommandCalibrationStage Command_CalibrationStage(void);
/* USB 前台传入可修改的完整文本行；回复以换行结束，不在 USB 中断中执行。 */
void Command_Execute(char *line, char *reply, size_t capacity);

#endif
