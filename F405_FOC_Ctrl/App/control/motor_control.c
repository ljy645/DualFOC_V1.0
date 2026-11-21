/**
 * @file motor_control.c
 * @brief 固定双轴控制计算：电流环每拍执行，速度环五分频执行。
 */
#include "motor_control.h"
#include "cogging.h"
#include <math.h>
#include <string.h>

#define MOTOR_TWO_PI 6.2831853071795865f
#define MOTOR_RPM_TO_RAD_S 0.1047197551196598f
#define MOTOR_SPEED_DIVIDER 5U
#define MOTOR_SPEED_ACCEL_RPM_S 5000.0f /* 速度给定的加、减速限制。 */

/** 每轴的算法数据；speed_control 只选择给定来源，不表示启停或故障状态。 */
typedef struct
{
    MotorConfig config;
    FocState foc;
    bool speed_control;      /* true：速度 PI 给定 Iq；false：直接电流给定。 */
    float id_ref;            /* d 轴目标，A。 */
    float iq_ref;            /* q 轴目标，A。 */
    float feedback[7];      /* 同拍 Id、Iq、Vd、Vq、限幅状态、机械角度、Iq 目标。 */
    float speed;             /* 最近一拍实测机械速度，rad/s。 */
    float speed_target;      /* 最终机械速度目标，rad/s，供通信回读。 */
    float speed_reference;   /* 斜坡输出，速度PI实际跟踪的给定，rad/s。 */
    float speed_integral;    /* 速度积分输出，A。 */
    unsigned int speed_tick; /* 速度环五分频计数，0～4。 */
} MotorAxis;

static MotorAxis motors[MOTOR_AXIS_COUNT];

/** @brief 填入当前 5010 电机的算法初始参数。 */
void MotorControl_DefaultConfig(MotorConfig *config)
{
    Foc_InitParams(&config->foc);
    config->foc.duty_margin = MOTOR_HW_DUTY_MARGIN;
    config->pole_pairs = 7;
    config->encoder_direction = 1;
    config->electrical_zero_rad = 0.0f;
    config->current_limit_a = 10.0f; /* 与 5 mΩ、20 倍采样增益配套的 Iq 指令上限。 */
    config->speed_kp = 0.2f;
    config->speed_ki = 1.0f;
}

/** @brief 清零该轴的目标、积分和分频计数，保留参数。 */
void MotorControl_Reset(unsigned int axis)
{
    MotorAxis *motor = &motors[axis];
    unsigned int i;
    Foc_Reset(&motor->foc);
    motor->speed_control = false;
    motor->id_ref = 0.0f;
    motor->iq_ref = 0.0f;
    for (i = 0U; i < 7U; ++i) { motor->feedback[i] = NAN; }
    motor->speed = 0.0f;
    motor->speed_target = 0.0f;
    motor->speed_reference = 0.0f;
    motor->speed_integral = 0.0f;
    motor->speed_tick = 0U;
}

/** @brief Initialize both axes with default parameters. */
void MotorControl_Init(void)
{
    memset(motors, 0, sizeof(motors));
    MotorControl_DefaultConfig(&motors[MOTOR_AXIS_M0].config);
    MotorControl_DefaultConfig(&motors[MOTOR_AXIS_M1].config);
    MotorControl_Reset(MOTOR_AXIS_M0);
    MotorControl_Reset(MOTOR_AXIS_M1);
}

/** @brief 装入上层已校验的参数，并清零该轴算法数据。 */
void MotorControl_Configure(unsigned int axis, const MotorConfig *config)
{
    motors[axis].config = *config;
    MotorControl_Reset(axis);
}

const MotorConfig *MotorControl_GetConfig(unsigned int axis)
{
    return &motors[axis].config;
}

void MotorControl_SetSpeedGains(unsigned int axis, float kp, float ki)
{
    motors[axis].config.speed_kp = kp;
    motors[axis].config.speed_ki = ki;
}

float MotorControl_GetSpeedTarget(unsigned int axis)
{
    return motors[axis].speed_control ? motors[axis].speed_target / MOTOR_RPM_TO_RAD_S : 0.0f;
}

void MotorControl_GetFeedback(unsigned int axis, float feedback[7])
{
    memcpy(feedback, motors[axis].feedback, sizeof(motors[axis].feedback));
}

/** @brief 选择直接电流给定，清除不再使用的速度积分。 */
void MotorControl_SetCurrentTarget(unsigned int axis, float id_a, float iq_a)
{
    motors[axis].speed_control = false;
    motors[axis].id_ref = id_a;
    motors[axis].iq_ref = iq_a;
    motors[axis].speed_integral = 0.0f;
}

/** @brief 更新斜坡最终目标；切入速度模式时从最近实测速度起步，承接当前Iq积分。 */
void MotorControl_SetSpeedTarget(unsigned int axis, float rpm)
{
    MotorAxis *motor = &motors[axis];
    if (!motor->speed_control)
    {
        motor->speed_integral = motor->iq_ref;
        motor->speed_reference = motor->speed;
        motor->speed_tick = 0U;
    }
    motor->speed_control = true;
    motor->speed_target = rpm * MOTOR_RPM_TO_RAD_S;
    motor->id_ref = 0.0f;
}

/* 标定使用原来的阶跃给定，不经过用户指令斜坡。调用方同样须串行/短临界区。 */
void MotorControl_SetSpeedTargetImmediate(unsigned int axis, float rpm)
{
    MotorControl_SetSpeedTarget(axis, rpm);
    motors[axis].speed_reference = motors[axis].speed_target;
}

/** @brief 对算法量做正负对称限幅，供 Iq 和积分共用。 */
static float MotorControl_Limit(float value, float limit)
{
    value = (value > limit) ? limit : value;
    return (value < -limit) ? -limit : value;
}

/** @brief 按最新速度目标计算 PI；饱和且误差继续推向饱和方向时暂停积分。 */
static void MotorControl_SpeedStep(MotorAxis *motor, float feedforward)
{
    float dt = motor->config.foc.sample_period_s * MOTOR_SPEED_DIVIDER;
    float ramp_step = MOTOR_SPEED_ACCEL_RPM_S * MOTOR_RPM_TO_RAD_S * dt;
    float error;
    float raw;
    float limit = motor->config.current_limit_a;

    /* 只整形给定；PI增益、积分和抗饱和算法保持原样。正反转连续经过零速。 */
    motor->speed_reference += MotorControl_Limit(motor->speed_target - motor->speed_reference, ramp_step);
    error = motor->speed_reference - motor->speed;
    raw = motor->config.speed_kp * error + motor->speed_integral + feedforward;
    /* iq_ref 保存 PI 部分；每个电流周期重新按实际角度加前馈，不重复累加。 */
    motor->iq_ref = motor->config.speed_kp * error + motor->speed_integral;
    /* 这是 PI 抗积分饱和条件，不涉及运行许可或故障处理。 */
    if (!((raw > limit && error > 0.0f) || (raw < -limit && error < 0.0f)))
    {
        motor->speed_integral = MotorControl_Limit(
            motor->speed_integral + motor->config.speed_ki * dt * error, limit);
    }
}

/** @brief 单轴周期调度：更新测量，按分频执行速度外环，每拍执行 FOC 电流内环。 */
void MotorControl_Step(unsigned int axis, const MotorHwSample *sample, FocOutput *output)
{
    MotorAxis *motor = &motors[axis];
    FocInput input;
    float angle;
    float feedforward = 0.0f;

    /* 1. 更新本拍反馈，统一机械速度方向。 */
    motor->speed = sample->mechanical_speed_rad_s * motor->config.encoder_direction;
    /* 2. 速度外环生成 Iq 目标；直接电流模式沿用上层给定。 */
    if (motor->speed_control)
    {
        feedforward = Cogging_Current(axis, sample->mechanical_angle_rad);
        /* 速度环仅在每组五拍中的首拍更新，Iq 在其余四拍保持。 */
        if (motor->speed_tick == 0U)
        {
            MotorControl_SpeedStep(motor, feedforward);
        }
        if (++motor->speed_tick >= MOTOR_SPEED_DIVIDER)
        {
            motor->speed_tick = 0U;
        }
    }
    /* 3. 组装本拍 FOC 输入，包括电角度、反馈和目标电流。 */
    angle = fmodf(sample->mechanical_angle_rad * motor->config.encoder_direction * motor->config.pole_pairs -
                  motor->config.electrical_zero_rad, MOTOR_TWO_PI);
    /* 负角度折回一周，满足 FOC 的 [0, 2*pi) 输入约定。 */
    if (angle < 0.0f)
    {
        angle += MOTOR_TWO_PI;
    }
    input.theta_sample_rad = (angle >= MOTOR_TWO_PI) ? 0.0f : angle;
    input.theta_apply_rad = input.theta_sample_rad;
    input.ia_a = sample->ia_a;
    input.ib_a = sample->ib_a;
    input.vbus_v = sample->vbus_v;
    input.id_ref_a = motor->id_ref;
    input.iq_ref_a = motor->speed_control ?
        MotorControl_Limit(motor->iq_ref + feedforward, motor->config.current_limit_a) : motor->iq_ref;
    input.omega_e_rad_per_s = motor->speed * motor->config.pole_pairs;
    /* 4. 电流内环每拍执行，跟踪 Id/Iq 目标并生成三相占空比。 */
    Foc_Step(&motor->config.foc, &motor->foc, &input, output);
    motor->feedback[0] = output->id_a;
    motor->feedback[1] = output->iq_a;
    motor->feedback[2] = output->vd_v;
    motor->feedback[3] = output->vq_v;
    motor->feedback[4] = output->voltage_limited ? 1.0f : 0.0f;
    motor->feedback[5] = sample->mechanical_angle_rad;
    motor->feedback[6] = input.iq_ref_a;
}

/** @brief 固定依次计算两轴，不循环、不判断运行状态、不操作 PWM。 */
void MotorControl_OnCurrentSample(const MotorHwSample samples[MOTOR_AXIS_COUNT],
                                 FocOutput outputs[MOTOR_AXIS_COUNT])
{
    MotorControl_Step(MOTOR_AXIS_M0, &samples[MOTOR_AXIS_M0], &outputs[MOTOR_AXIS_M0]);
    MotorControl_Step(MOTOR_AXIS_M1, &samples[MOTOR_AXIS_M1], &outputs[MOTOR_AXIS_M1]);
}
