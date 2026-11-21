/**
 * @file usb_hostif.h
 * @brief USB CDC JustFloat 波形、文本配置与双轴 HOST 遥测接口。
 */
#ifndef USB_HOSTIF_H
#define USB_HOSTIF_H

#include <stdint.h>

/**
 * @brief 从 App_Poll 调用，处理目标命令并按固定间隔尝试上传一次；仅从前台调用。
 * @note 通道顺序为 Ia(A)、Ib(A)、Ic(A)、转速(rpm)、母线电压(V)、实际 Id(A)、实际 Iq(A)、
 *       指令 Vd(V)、指令 Vq(V)、电压限幅状态(0/1)、机械角度(deg)、Iq目标(A)，
 *       追加 M0/M1 实际使能状态、最近使能命令轴号和结果（1成功/-1失败/0未收到）。
 *       轴和上传间隔在 usb_hostif.c 选择，不影响双轴独立控制；无效数据为 NaN。
 *       未连接、未到期或发送忙时直接返回，不等待、不排队补发。
 *       本模块独占 CDC 发送接口，传输完成前不改写发送缓冲区。
 *       speed <rpm> 默认更新 M0；speed <axis> <rpm> 可指定轴，不自动启停。
 *       iq <A> 切换为 Id=0 的电流模式；超过该轴 current_limit_a 的命令忽略，不自动启停。
 *       id <A> 切换为 Iq=0 的电流模式，限幅约定与 iq 相同，也支持显式轴号。
 *       config on 暂停波形并启用文本命令/应答；config off 应答后恢复波形。
 *       host on 返回 OK HOST 1，启用双轴50 Hz文本遥测和命令应答；host off 恢复波形。
 *       enable <axis> <0/1> 在所有模式均可执行，波形模式通过附加通道回传结果。
 *       配置命令逐条发送，等待回复后再发送下一条；param show 以 END 行结束。
 *       calibrate <axis> 为完整校准，P CALIBRATION ZERO/COGGING 是进度帧，
 *       仅最终 OK CALIBRATED_FULL 表示两个阶段均已保存成功。
 */
void UsbVofa_Poll(void);

/** @brief CDC 回调拼行，支持分包/CRLF，最多 127 字符；忙时拒绝新行，不覆盖待处理命令。 */
void UsbVofa_Receive(const uint8_t *data, uint32_t length);
/** @brief CDC 初始化/退出时清除未完成和待处理命令，仅由 USB 回调调用。 */
void UsbVofa_ResetRx(void);

#endif
