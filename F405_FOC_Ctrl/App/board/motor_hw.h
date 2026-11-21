/**
 * @file motor_hw.h
 * @brief 双轴硬件映射、快环测量快照及功率输出接口。
 */
#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_AXIS_M0    0  /* TIM1，ADC1/ADC2 注入 Rank 1 */
#define MOTOR_AXIS_M1    1  /* TIM8，ADC1/ADC2 注入 Rank 2 */
#define MOTOR_AXIS_COUNT 2  /* 板上固定有两路电机 */
#define MOTOR_HW_DUTY_MARGIN 0.05f /* 恢复原 FOC 调制余量，占空比范围 5%～95%。 */

/** 共享故障定义：采样层报告数据问题，运行阈值和停机/恢复由上层管理。 */
#define MOTOR_FAULT_MEASUREMENT 0x01U /* 电流 ADC 采样/外设错误。 */
#define MOTOR_FAULT_STALE       0x02U /* 电流采样过期或重复，由电流采样层报告。 */
#define MOTOR_FAULT_OVERCURRENT 0x04U /* 上层：实际相电流超过运行阈值。 */
#define MOTOR_FAULT_BUS_VOLTAGE 0x08U /* 上层：实际母线电压超过欠压/过压阈值。 */
#define MOTOR_FAULT_DRIVER      0x10U /* 电流放大器未唤醒或驱动器故障。 */
#define MOTOR_FAULT_OUTPUT      0x20U /* 上层：占空比写入/功率输出操作失败。 */
#define MOTOR_FAULT_BUS_SAMPLE  0x40U /* 母线采样尚未取得、失败或过期。 */
#define MOTOR_FAULT_ENCODER     0x80U /* 编码器帧缺失、告警、通信错误或过期。 */

/**
 * 同一轴的一致测量快照，电流流入电机为正，角度和速度沿编码器原始正方向。
 * BSP 负责提供与电流采样时刻对应的角度，并校验编码器、速度和低频母线缓存时效。
 */
typedef struct
{
    float ia_a;                    /* 同时采样的 A 相电流，A。 */
    float ib_a;                    /* 同时采样的 B 相电流，A。 */
    float mechanical_angle_rad;    /* 编码器单圈机械角，[0, 2*pi)。 */
    float mechanical_speed_rad_s;  /* 实际机械角速度，rad/s，保留符号。 */
    float vbus_v;                  /* 实测母线电压，V。 */
    uint32_t current_time_us;      /* TIM5 微秒时基，按触发相位和 ADC 配置估计的本轴采样时刻。 */
    uint32_t faults;              /* 本次快照的数据故障位；控制还须检查 feedback_complete。 */
    bool feedback_complete;        /* 电流、母线和可用的角度/速度均已取得并完成时间配准。 */
} MotorHwSample;

/**
 * @brief 初始化 ADC/PWM 后端，设置并读回确认电流放大增益，校准零偏并启动采样。
 * @note 在 CubeMX GPIO、ADC、TIM、SPI 和 DMA 初始化后调用，不要求电机静止。
 *       包含 10 ms 驱动器唤醒及 256 次零偏采样，仅从前台调用。
 *       返回通过驱动器检查和电流校准的轴位掩码：bit0=M0、bit1=M1，0=无可用轴。
 *       等待母线和编码器正常帧后可取得完整反馈；不自动开启输出。
 *       EN_GATE 会拉高以唤醒放大器，但两轴 MOE 保持关闭、PWM 输入保持全低。
 *       单轴失败仅关闭该轴驱动器，共享后端失败关闭两轴；成功后 TIM5 为 1 MHz 时间基准。
 */
uint8_t MotorHw_Init(void);
/**
 * @brief 无阻塞读取一致测量快照，返回 false 时输出参数不更新。
 * @note 返回 true 仅表示取得快照；交给控制算法前须满足 feedback_complete 且 faults 为零。
 * @note 本层检查采样完整性和时效；电流、母线的运行保护阈值由后续保护层管理。
 *       采样停止的独立监测与关断留给上层，不能依赖电流 ISR 持续运行。
 *       不调用阻塞 SPI；复用最新编码器帧，用机械速度外推到电流采样时刻。
 *       首帧速度未建立时 feedback_complete 为 false；编码器无结果或过期时报告故障。
 *       同一轴的 ReadSample/WriteDuty/SetOutputEnabled 由上层串行调用。
 */
bool MotorHw_ReadSample(unsigned int axis, MotorHwSample *sample);
/** @brief 诊断快照，数据约定同 ReadSample，但不消费控制帧，不因重复查看而报告 STALE。 */
bool MotorHw_PeekSample(unsigned int axis, MotorHwSample *sample);
/**
 * @brief 将有限的 [0,1] 高侧导通比例写入三相 CCR 预装载，不开启输出。
 * @note 按原理图 CH3N=A、CH2N=B、CH1N=C，PWM2 高有效换算。
 *       错误参数返回 false；固定 PWM 配置在 Init 中校验，运行中不得修改。
 *       不修改 CH4 采样触发点，不主动产生更新事件；三相在后续更新事件生效。
 *       不按固定低侧采样窗口拒绝占空比；高占空比采样有效性须另行验证。
 *       运行时定时器配置须保持固定，实际波形仍须实板确认。
 */
bool MotorHw_WriteDuty(unsigned int axis, float duty_a, float duty_b, float duty_c);
/**
 * @brief 执行指定轴的硬件输出使能或关断，不管理应用启停状态和故障恢复策略。
 * @return 有效轴关闭成功返回 true；无效轴返回 false 且不操作硬件。
 * @note 上层先读取完整且无故障的反馈、装载占空比，跨过一次 PWM 更新事件后开启。
 *       本接口仅检查初始化、计数器运行和占空比预装载更新，不检查反馈或运行许可。
 *       上层负责反馈时效和运行保护；开启失败不主动修改已开启的输出。
 *       关闭仅清 MOE，保持 EN_GATE 高及电流采样运行；重新开启前须重新装载占空比。
 *       调用前须已完成 GPIO 和定时器初始化。
 */
bool MotorHw_SetOutputEnabled(unsigned int axis, bool enable);

#endif
