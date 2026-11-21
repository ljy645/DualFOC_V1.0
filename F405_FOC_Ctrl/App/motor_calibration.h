/** @file motor_calibration.h */
#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include "motor_control.h"

typedef enum
{
    MOTOR_CALIBRATION_OK = 0,
    MOTOR_CALIBRATION_INVALID_ARGUMENT,
    MOTOR_CALIBRATION_BUSY,
    MOTOR_CALIBRATION_NOT_READY,
    MOTOR_CALIBRATION_FEEDBACK_ERROR,
    MOTOR_CALIBRATION_BUS_VOLTAGE,
    MOTOR_CALIBRATION_OVERCURRENT,
    MOTOR_CALIBRATION_OUTPUT_ERROR,
    MOTOR_CALIBRATION_NO_MOVEMENT,
    MOTOR_CALIBRATION_TRACKING_ERROR,
    MOTOR_CALIBRATION_STORAGE_ERROR
} MotorCalibrationResult;

/* 找零内部入口：自行关断本轴，成功后应用参数；返回时关闭输出，由 command 保存结果。 */
MotorCalibrationResult MotorCalibration_Run(unsigned int axis, MotorConfig *config);
/* ADC IRQ 与前台均须调用；传入应用故障掩码，返回标定故障掩码并立即关断故障轴。 */
uint8_t MotorCalibration_Monitor(uint8_t faults);
/* 空闲返回 -1，否则返回当前标定轴 M0/M1。 */
int MotorCalibration_ActiveAxis(void);

typedef enum
{
    COGGING_CAL_OK = 0,
    COGGING_CAL_OUTPUT,
    COGGING_CAL_FEEDBACK,
    COGGING_CAL_TIMEOUT,
    COGGING_CAL_COVERAGE,
    COGGING_CAL_CURRENT
} CoggingCalibrationResult;

/* 内部入口：零位已装入RAM、旧补偿记录已持久化失效，补偿关闭，两轴当前关闭。
 * 空载正反各10 rpm，实测转速稳定后各采3圈；持续 App_Poll，返回时关闭本轴。
 * 成功在 Cogging_Table 中留下1024点表；本函数不写 EEPROM。
 */
CoggingCalibrationResult MotorCalibration_RunCogging(unsigned int axis);

#endif
