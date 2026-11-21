/**
 * @file enc_mt6826s.h
 * @brief 双轴 MT6826S 非阻塞 SPI DMA 请求和结果读取接口。
 */
#ifndef ENC_MT6826S_H
#define ENC_MT6826S_H

#include <stdint.h>
#include "motor_hw.h"

#define ENC_MT6826S_STATUS_OVERSPEED    0x01U /* 转速超过允许范围。 */
#define ENC_MT6826S_STATUS_WEAK_FIELD   0x02U /* 磁场过弱。 */
#define ENC_MT6826S_STATUS_UNDERVOLTAGE 0x04U /* 芯片供电欠压。 */

/** 请求与最近一帧结果，PENDING 仅表示已受理，不代表测量完成。 */
typedef enum
{
    ENC_MT6826S_OK = 0,
    ENC_MT6826S_INVALID_ARG,
    ENC_MT6826S_NOT_READY,
    ENC_MT6826S_BUSY,
    ENC_MT6826S_SPI_ERROR,
    ENC_MT6826S_TIMEOUT,
    ENC_MT6826S_CRC_ERROR,
    ENC_MT6826S_WARNING,
    ENC_MT6826S_PENDING
} EncMt6826sResult;

/** 通过 CRC 校验的机械角度，WARNING 数据仅供诊断，尚未作电角零位补偿。 */
typedef struct
{
    uint16_t angle_raw; /* 15 位角度，0～32767。 */
    float angle_rad;    /* 单圈机械角，[0, 2*pi)。 */
    uint8_t status;     /* 芯片告警低三位。 */
    uint32_t time_us;   /* SPI 请求时刻，来自 ADC 初始化的 TIM5 微秒时基。 */
    float speed_rad_s; /* 沿编码器原始正方向、经一阶低通的机械角速度，rad/s。 */
    bool speed_valid;  /* 首帧、错误后首帧或无有效时间间隔时为 false。 */
} EncMt6826sSample;

/**
 * @brief 初始化缓存及片选，在 CubeMX 的 GPIO、SPI1 和 DMA 初始化后调用。
 * @note SPI1 独占，固定为 8 位 Mode 3、MSB，RX/TX 使用单次字节 DMA。
 *       配置由 CubeMX 统一管理，运行中不重复校验；忙时不打断事务。
 *       SPI/DMA/超时故障后由上层调用本函数恢复；CRC 错误仅报告对应轴。
 *       DMA 尚未退出时返回 BUSY；成功后重新建立速度基准。
 */
EncMt6826sResult EncMt6826s_Init(void);
/** @brief 按 bit0=M0、bit1=M1 提交非阻塞 DMA 读取，只接受非零的两轴位掩码。
 *         两轴按顺序读取、片选互斥；成功返回 PENDING，可从前台或中断调用。
 */
EncMt6826sResult EncMt6826s_RequestMask(uint8_t mask);
/**
 * @brief 无等待地读取最近一次请求的结果，不再发起 SPI 传输。
 * @return OK/WARNING 时更新 sample，其他结果保持调用者原值；请求中返回 PENDING。
 * @note 先 RequestMask，后读取缓存；SPI1 由本驱动独占。
 *       速度由相邻正常帧角度差分得到，跨圈使用最短角差，必须保证间隔内转动小于半圈。
 *       差分速度经 20 Hz 一阶低通，按实际帧间隔更新；角度不滤波，不改变调用频率。
 *       TIM5 须已由 AdcCurrent_Init 启动；否则角度可读，speed_valid 为 false。
 *       两帧间隔达到 2 ms 超时窗口时重新建立速度基准，不输出旧基准的速度。
 */
EncMt6826sResult EncMt6826s_Read(unsigned int axis, EncMt6826sSample *sample);
/**
 * @brief 控制反馈读取：下一帧 DMA 进行中仍可读取上一帧结果，时间有效性由调用方检查。
 * @note 共享 SPI 故障时立即返回故障，不继续提供旧的正常结果；不发起新传输。
 */
EncMt6826sResult EncMt6826s_ReadLatest(unsigned int axis, EncMt6826sSample *sample);
/**
 * @brief 推进正常的第二轴请求并检查超时，不自动恢复故障，无轮询等待。
 * @note 已接入 App_Poll 及 SPI1 DMA 中断尾部；需保持调用才能监测丢失中断。
 *       仅从前台或 SPI1 DMA 中断尾部调用，不从更高优先级的控制中断调用。
 *       time_us 是请求时间；motor_hw 用速度外推到电流采样时刻，芯片内部延迟须实测确认。
 *       SPI/DMA/超时故障取消本批请求，直到 Init 成功；CRC 错误不取消另一轴请求。
 */
void EncMt6826s_Service(void);

#endif
