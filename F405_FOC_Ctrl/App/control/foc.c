/**
 * @file foc.c
 * @brief 单精度 FOC 电流环，按 Simulink 电流控制器移植，各轴独立传入积分状态。
 */

#include "foc.h"

#include "dsp/controller_functions.h"
#include "dsp/fast_math_functions.h"

#define FOC_INV_SQRT_3   0.5773502691896258f
#define FOC_RAD_TO_DEG   57.29577951308232f

/**
 * @brief 加载与当前 Simulink 模型一致的默认参数，全部使用单精度存储。
 */
void Foc_InitParams(FocParams *params)
{
    params->d.kp_v_per_a = 0.15f;
    params->d.ki_v_per_as = 377.0f;
    params->q = params->d;
    params->sample_period_s = 50.0e-6f;
    params->duty_margin = 0.05f;
    params->anti_windup_gain = 0.1f;
    params->ld_h = 50.0e-6f;
    params->lq_h = 50.0e-6f;
    params->flux_linkage_wb = 0.0021f;
}

/**
 * @brief 检查初始化配置，避免将非法周期、增益或电机参数交给快环。
 */
bool Foc_ValidateParams(const FocParams *params)
{
    if (params == 0)
    {
        return false;
    }

    return isfinite(params->d.kp_v_per_a) && params->d.kp_v_per_a >= 0.0f &&
           isfinite(params->d.ki_v_per_as) && params->d.ki_v_per_as >= 0.0f &&
           isfinite(params->q.kp_v_per_a) && params->q.kp_v_per_a >= 0.0f &&
           isfinite(params->q.ki_v_per_as) && params->q.ki_v_per_as >= 0.0f &&
           isfinite(params->sample_period_s) && params->sample_period_s > 0.0f &&
           isfinite(params->duty_margin) && params->duty_margin >= 0.0f && params->duty_margin < 0.5f &&
           isfinite(params->anti_windup_gain) && params->anti_windup_gain > 0.0f && params->anti_windup_gain <= 1.0f &&
           isfinite(params->ld_h) && params->ld_h >= 0.0f &&
           isfinite(params->lq_h) && params->lq_h >= 0.0f &&
           isfinite(params->flux_linkage_wb) && params->flux_linkage_wb >= 0.0f;
}

/**
 * @brief 清除当前电机的 d/q 积分电压，在进入或退出电流闭环时调用。
 */
void Foc_Reset(FocState *state)
{
    state->integral_d_v = 0.0f;
    state->integral_q_v = 0.0f;
}

/**
 * @brief 将浮点舍入造成的边界越界收回占空比余量内，保留低边采样的静态余量。
 */
static float Foc_ClampDuty(float duty, float margin)
{
    if (duty < margin)
    {
        return margin;
    }
    if (duty > 1.0f - margin)
    {
        return 1.0f - margin;
    }
    return duty;
}

/**
 * @brief 计算单拍 Clarke/Park、PI 与前馈、矢量限幅、反 Park 和 SVPWM。
 * @details 所有中间量为 float，不分配动态内存，不保存跨电机共享的可变状态。
 */
void Foc_Step(const FocParams *params, FocState *state,
              const FocInput *input, FocOutput *output)
{
    float sin_sample;
    float cos_sample;
    float sin_apply;
    float cos_apply;
    float beta;
    float id;
    float iq;
    float error_d;
    float error_q;
    float trial_d;
    float trial_q;
    float raw_d;
    float raw_q;
    float limit = (1.0f - 2.0f * params->duty_margin) * input->vbus_v * FOC_INV_SQRT_3;
    float magnitude_squared;
    float vd;
    float vq;
    float alpha;
    float phase_a;
    float phase_b;
    float phase_c;
    float common;
    float phase_max;
    float phase_min;
    float inv_vbus = 1.0f / input->vbus_v;

    /* CMSIS 成对计算正余弦，接口单位为度，电流环对外仍统一使用弧度。 */
    arm_sin_cos_f32(input->theta_sample_rad * FOC_RAD_TO_DEG, &sin_sample, &cos_sample);
    sin_apply = sin_sample;
    cos_apply = cos_sample;
    arm_clarke_f32(input->ia_a, input->ib_a, &alpha, &beta);
    arm_park_f32(alpha, beta, &id, &iq, sin_sample, cos_sample);

    error_d = input->id_ref_a - id;
    error_q = input->iq_ref_a - iq;
    trial_d = state->integral_d_v + params->d.ki_v_per_as * params->sample_period_s * error_d;
    trial_q = state->integral_q_v + params->q.ki_v_per_as * params->sample_period_s * error_q;
    raw_d = params->d.kp_v_per_a * error_d + trial_d - input->omega_e_rad_per_s * params->lq_h * iq;
    raw_q = params->q.kp_v_per_a * error_q + trial_q + input->omega_e_rad_per_s * (params->ld_h * id + params->flux_linkage_wb);
    magnitude_squared = raw_d * raw_d + raw_q * raw_q;
    vd = raw_d;
    vq = raw_q;

    /* 只在矢量超限时开平方，前馈与 PI 共同参与限幅。 */
    output->voltage_limited = magnitude_squared > limit * limit;
    if (output->voltage_limited)
    {
        float magnitude;
        float scale;

        /* 非负模长通过 CMSIS 内联接口，在 ARMCC5/FPU 目标上使用 VSQRT.F32。 */
        (void)arm_sqrt_f32(magnitude_squared, &magnitude);
        scale = limit / magnitude;
        vd *= scale;
        vq *= scale;
    }

    /* 用总电压的限幅差回算积分，避免高速或低母线电压时积分持续累积。 */
    state->integral_d_v = trial_d + params->anti_windup_gain * (vd - raw_d);
    state->integral_q_v = trial_q + params->anti_windup_gain * (vq - raw_q);

    /* 有输出延迟补偿时使用预计生效角，否则复用采样角的正余弦。 */
    if (input->theta_apply_rad != input->theta_sample_rad)
    {
        arm_sin_cos_f32(input->theta_apply_rad * FOC_RAD_TO_DEG, &sin_apply, &cos_apply);
    }

    arm_inv_park_f32(vd, vq, &alpha, &beta, sin_apply, cos_apply);
    arm_inv_clarke_f32(alpha, beta, &phase_a, &phase_b);
    phase_c = -phase_a - phase_b;
    
    /* 有限电压用内联比较求极值，避免通用浮点最大最小值函数调用。 */
    phase_max = (phase_a > phase_b) ? phase_a : phase_b;
    phase_max = (phase_c > phase_max) ? phase_c : phase_max;

    phase_min = (phase_a < phase_b) ? phase_a : phase_b;
    phase_min = (phase_c < phase_min) ? phase_c : phase_min;
    common = (phase_max + phase_min) * 0.5f;

    output->duty_a = Foc_ClampDuty(0.5f + (phase_a - common) * inv_vbus, params->duty_margin);
    output->duty_b = Foc_ClampDuty(0.5f + (phase_b - common) * inv_vbus, params->duty_margin);
    output->duty_c = Foc_ClampDuty(0.5f + (phase_c - common) * inv_vbus, params->duty_margin);
    output->id_a = id;
    output->iq_a = iq;
    output->vd_v = vd;
    output->vq_v = vq;
}
