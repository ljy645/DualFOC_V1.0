/**
 * @file foc.h
 * @brief 双轴共用的单精度 FOC 电流环接口，每台电机独立保存参数和积分状态。
 */

#ifndef FOC_H
#define FOC_H

#include <stdbool.h>

/** 单轴 PI 参数，采用并联形式，积分增量为 ki_v_per_as * sample_period_s * 电流误差。 */
typedef struct
{
    float kp_v_per_a;  /* 比例增益，V/A。 */
    float ki_v_per_as; /* 积分增益，V/(A·s)。 */
} FocPiParams;

/** 电流环配置，在进入快环前校验。两轴可使用不同的 PI 增益。 */
typedef struct
{
    FocPiParams d;
    FocPiParams q;
    float sample_period_s;       /* 控制周期，单位 s。本工程初值为 50 µs。 */
    float duty_margin;           /* 对称占空比余量，范围 [0, 0.5)。 */
    float anti_windup_gain;      /* 每拍回算系数，范围 (0, 1]。 */
    float ld_h;                  /* d 轴电感，单位 H，用于解耦前馈。 */
    float lq_h;                  /* q 轴电感，单位 H，用于解耦前馈。 */
    float flux_linkage_wb;       /* 每相永磁磁链，单位 Wb，用于反电动势前馈。 */
} FocParams;

/** 每台电机独有的 PI 积分状态，单位 V。 */
typedef struct
{
    float integral_d_v;
    float integral_q_v;
} FocState;

/** 同一拍的已校准测量值与目标值，由电机控制层组装。 */
typedef struct
{
    float ia_a;                  /* A 相瞬时电流，流入电机为正，单位 A。 */
    float ib_a;                  /* B 相瞬时电流，与 A 相同时采样，单位 A。 */
    float theta_sample_rad;      /* 电流采样时刻的电角度，用于 Park。 */
    float theta_apply_rad;       /* PWM 输出预计生效时刻的电角度，用于反 Park。 */
    float vbus_v;                /* 最近一次有效的母线电压，单位 V，必须大于零。 */
    float id_ref_a;              /* 已限幅的 d 轴电流目标，单位 A。 */
    float iq_ref_a;              /* 已限幅的 q 轴电流目标，单位 A。 */
    float omega_e_rad_per_s;     /* 带符号的实际电角速度，单位 rad/s，等于机械速度乘极对数。 */
} FocInput;

/** 本拍的调制结果与用于仿真对照的电流、电压量。 */
typedef struct
{
    float duty_a;                /* A 相高侧导通比例，归一化到 [0, 1]。 */
    float duty_b;                /* B 相高侧导通比例，归一化到 [0, 1]。 */
    float duty_c;                /* C 相高侧导通比例，归一化到 [0, 1]。 */
    float id_a;                  /* 测得的 d 轴电流，单位 A。 */
    float iq_a;                  /* 测得的 q 轴电流，单位 A。 */
    float vd_v;                  /* 限幅后的 d 轴指令电压，单位 V。 */
    float vq_v;                  /* 限幅后的 q 轴指令电压，单位 V。 */
    bool voltage_limited;       /* 本拍电压矢量是否被缩放。 */
} FocOutput;

/**
 * @brief 写入当前 5010 正弦仿真模型的电流环默认参数，上电后可由有效配置覆盖。
 * @param params 有效配置指针；模型参数仍需实板辨识和时序验证。
 */
void Foc_InitParams(FocParams *params);

/**
 * @brief 在初始化或参数提交时检查配置范围和有限性，成功返回 true。
 * @param params 待校验配置，允许传入空指针并返回 false。
 */
bool Foc_ValidateParams(const FocParams *params);

/**
 * @brief 清零指定电机的两路积分。启动闭环前、退出闭环后调用。
 * @param state 该电机的有效状态指针。
 */
void Foc_Reset(FocState *state);

/**
 * @brief 执行一次电流闭环计算，生成三相占空比并更新积分状态。
 *
 * 输入和积分状态为有限数值，指针有效，参数须由调用方预先校验。电流、电压、
 * 角度的有效性及运行许可由调用方管理。每轴每个固定控制周期调用一次。
 *
 * 坐标约定为 A-B-C 相序，A 相正方向与 alpha 轴重合。
 * 两相采样按 Ia + Ib + Ic = 0 重构，采用幅值不变 Clarke 变换：
 * i_alpha = Ia，i_beta = (Ia + 2*Ib) / sqrt(3)。
 * Park 变换为 id = i_alpha*cos(theta) + i_beta*sin(theta)，
 * iq = -i_alpha*sin(theta) + i_beta*cos(theta)。
 * 电角度以 alpha 轴为零，向 beta 轴为正，输入范围 [0, 2*pi)。
 * 极对数、方向和零位由调用方处理。初期可给两种角度传同一个值。
 *
 * 每拍先求试积分 I_try = I_prev + Ki*Ts*e，再将 PI 与前馈电压相加。
 * 前馈为 Vd_ff = -omega_e*Lq*iq，Vq_ff = omega_e*(Ld*id + flux)。
 * 电感或磁链为零时，对应前馈项为零。速度须使用实测或估计值。
 * 对 dq 电压等比例限幅，矢量模长上限为
 * (1 - 2*duty_margin)*Vbus/sqrt(3)，再用限幅差回算积分：
 * I_next = I_try + anti_windup_gain*(V_limited - V_raw)。
 * 反 Park 后用三相 min/max 共模注入计算占空比。duty_margin 是静态
 * 调制余量，板级代码负责确认低边采样窗口并将高侧导通比例换算为 CCR。
 * 该函数不包含电流给定限幅、过流锁存、运行状态或 PWM 输出使能。
 */
void Foc_Step(const FocParams *params, FocState *state,
              const FocInput *input, FocOutput *output);

#endif
