/**
 * @file motor_control.h
 * @brief 固定双轴的电流环和速度环计算接口。
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "foc.h"
#include "motor_hw.h"

/** 每台电机的算法参数；保护阈值、校准许可和运行状态由应用及 BSP 管理。 */
typedef struct
{
    FocParams foc;             /* 电流 PI、电感、磁链和电流环周期。 */
    uint16_t pole_pairs;       /* 电机极对数。 */
    int8_t encoder_direction;  /* 编码器正方向，+1 或 -1。 */
    float electrical_zero_rad; /* 电角零位，rad。 */
    float current_limit_a;     /* 速度 PI 的 Iq 输出限幅，A。 */
    float speed_kp;            /* 速度 PI 比例增益，A/(rad/s)。 */
    float speed_ki;            /* 速度 PI 积分增益，A/rad。 */
} MotorConfig;

/* 调用约定：轴编号只能为 M0/M1，指针和参数有效，采样已校验。
 * 同一轴的配置、目标修改和计算由调用方串行执行，或用短临界区保护。
 * 本层不访问外设、不检查故障、不管理启停，也不发布运行信息。
 */
/** @brief 为固定两轴装入默认参数并清零算法数据，不操作功率输出。 */
void MotorControl_Init(void);
/** @brief 填入当前 5010 电机的算法初始参数，电角零位需另行校准。 */
void MotorControl_DefaultConfig(MotorConfig *config);
/** @brief 闭环暂停时装入一轴参数，同时复位目标和积分；参数由调用方预先校验。 */
void MotorControl_Configure(unsigned int axis, const MotorConfig *config);
/* 短临界区内热更新速度PI，保留目标、模式和积分，不重新校准或复位控制。 */
void MotorControl_SetSpeedGains(unsigned int axis, float kp, float ki);
/** @brief 读取该轴现有参数，只读指针在模块生命周期内有效，轴号由调用方校验。 */
const MotorConfig *MotorControl_GetConfig(unsigned int axis);
/* 读取最终指令目标（非斜坡中间值），直接电流模式返回0；单位rpm，前台用短临界区。 */
float MotorControl_GetSpeedTarget(unsigned int axis);
/** @brief 读取同拍 Id(A)、Iq(A)、Vd(V)、Vq(V)、限幅(0/1)、机械角度(rad)、Iq目标(A)。
 *  机械角度为采样时刻外推值，未修正方向；复位后为 NaN，调用方用短临界区读取。 */
void MotorControl_GetFeedback(unsigned int axis, float feedback[7]);
/** @brief 清零一轴目标和积分并选择电流控制；不关闭 PWM，由上层处理停机。 */
void MotorControl_Reset(unsigned int axis);
/** @brief 直接给定 d/q 电流并选择电流控制，单位 A；调用方负责目标电流限幅。 */
void MotorControl_SetCurrentTarget(unsigned int axis, float id_a, float iq_a);
/** @brief 给定最终机械转速rpm；经300 rpm/s加减速斜坡送入速度PI，Id固定为零。
 * 有效齿槽前馈照常加入并统一限流，重复下发目标不重置斜坡或积分。
 */
void MotorControl_SetSpeedTarget(unsigned int axis, float rpm);
/* 标定内部使用：直接设置速度给定，绕过用户指令斜坡。 */
void MotorControl_SetSpeedTargetImmediate(unsigned int axis, float rpm);
/**
 * @brief 单轴统一控制入口：更新测量、调度速度外环、执行 FOC 电流内环。
 * @note 上层下发目标后，按 foc.sample_period_s 固定周期调用本函数。
 *       速度模式下每五拍更新速度 PI，生成 Iq 目标；FOC 电流环每拍执行。
 *       直接电流模式下跳过速度 PI，FOC 使用 SetCurrentTarget 设置的目标。
 *       上层确认运行许可、feedback_complete 且 faults 为零后调用，再自行写入 PWM。
 */
void MotorControl_Step(unsigned int axis, const MotorHwSample *sample, FocOutput *output);
/**
 * @brief 两轴均需运行时调用，固定依次计算 M0、M1；数组按轴编号排列。
 * @note 每个电流周期调用一次，速度环五分频；默认分别为 20 kHz 和 4 kHz。
 *       仅运行一路时，上层直接调用该轴的 MotorControl_Step，不调用此双轴入口。
 */
void MotorControl_OnCurrentSample(const MotorHwSample samples[MOTOR_AXIS_COUNT],
                                 FocOutput outputs[MOTOR_AXIS_COUNT]);

#endif
