/**
 * @file usb_hostif.c
 * @brief USB CDC 波形与参数配置，共用发送口，指令只在前台执行。
 */
#include "usb_hostif.h"
#include "command.h"
#include "app.h"
#include "config_store.h"
#include <stdio.h>
#include "motor_hw.h"
#include "motor_control.h"
#include "adc_monitor.h"
#include "board_critical.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>
#include <ctype.h>

#define USB_VOFA_PERIOD_MS 1U       /* 默认最多 1 kHz，实际速率受主循环和 USB 影响。 */
#define USB_VOFA_RAD_S_TO_RPM 9.54929658551372f
#define USB_VOFA_AXIS MOTOR_AXIS_M0 /* 只选择波形对象，命令可显式指定轴。 */

extern USBD_HandleTypeDef hUsbDeviceFS;

static struct
{
    float channel[16];
    uint8_t tail[4];
} tx_frame = {{0}, {0x00U, 0x00U, 0x80U, 0x7FU}};
static uint32_t last_send_ms;
#define USB_COMMAND_BYTES 128U
#define USB_REPLY_BYTES 1024U
static char rx_line[USB_COMMAND_BYTES];
static char pending_line[USB_COMMAND_BYTES];
static char reply[USB_REPLY_BYTES]; /* CDC 发送完成前保持不变。 */
static unsigned int rx_length;
static bool rx_discard;
static volatile bool command_pending;
static volatile bool executing;
static volatile unsigned int reply_length;
static volatile uint8_t rx_error; /* 1：忙，2：行过长或包含 NUL。 */
static volatile bool config_mode;
static volatile bool host_mode;
static float command_axis = -1.0f;
static float command_result; /* VOFA：0=尚无使能命令，1=成功，-1=失败。 */
static char telemetry[512];
static CommandCalibrationStage progress_sent;

void UsbVofa_ResetRx(void)
{
    rx_length = 0U;
    rx_discard = false;
    command_pending = false;
    reply_length = 0U;
    rx_error = 0U;
    config_mode = false;
    host_mode = false;
    command_axis = -1.0f;
    command_result = 0.0f;
    progress_sent = COMMAND_CAL_IDLE;
}

/** @brief 回调只拼行；待处理命令不被后来的行覆盖。 */
void UsbVofa_Receive(const uint8_t *data, uint32_t length)
{
    uint32_t i;
    for (i = 0U; i < length; ++i)
    {
        char ch = (char)data[i];
        if (ch == '\n' || ch == '\r')
        {
            if (!rx_discard && rx_length != 0U)
            {
                if (command_pending || executing || reply_length != 0U) { rx_error = 1U; }
                else
                {
                    rx_line[rx_length] = '\0';
                    memcpy(pending_line, rx_line, rx_length + 1U);
                    command_pending = true;
                }
            }
            rx_length = 0U;
            rx_discard = false;
        }
        else if (!rx_discard)
        {
            if (ch != '\0' && rx_length < sizeof(rx_line) - 1U) { rx_line[rx_length++] = ch; }
            else { rx_discard = true; rx_error = 2U; }
        }
    }
}

/* 返回 true 时暂停波形，校准内部递归 App_Poll 时也不重入命令。 */
static bool UsbVofa_ServiceCommand(void)
{
    char line[USB_COMMAND_BYTES];
    uint32_t saved;
    USBD_CDC_HandleTypeDef *cdc;
    uint8_t error;
    bool have_command;
    bool enable_command;
    unsigned int enable_axis;
    if (executing)
    {
        CommandCalibrationStage stage = Command_CalibrationStage();
        /* 不可变帧可保持到 USB 传输完成，校准循环只发布阶段，不重入命令。 */
        static const char *const progress[] = {"", "P CALIBRATION ZERO\r\n", "P CALIBRATION COGGING\r\n"};
        if ((host_mode || config_mode) && stage != COMMAND_CAL_IDLE && stage != progress_sent)
        {
            saved = Board_EnterCritical();
            cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
            if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != 0 && cdc->TxState == 0U &&
                CDC_Transmit_FS((uint8_t *)progress[stage], (uint16_t)strlen(progress[stage])) == USBD_OK)
            {
                progress_sent = stage;
            }
            Board_ExitCritical(saved);
        }
        return true;
    }
    saved = Board_EnterCritical();
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || cdc == 0 || cdc->TxState != 0U)
    {
        Board_ExitCritical(saved);
        return true;
    }
    if (reply_length != 0U)
    {
        if (CDC_Transmit_FS((uint8_t *)reply, (uint16_t)reply_length) == USBD_OK) { reply_length = 0U; }
        Board_ExitCritical(saved);
        return true;
    }
    have_command = command_pending;
    error = rx_error;
    rx_error = 0U;
    if (have_command)
    {
        memcpy(line, pending_line, sizeof(line));
        command_pending = false;
        executing = true;
        progress_sent = COMMAND_CAL_IDLE;
        /* 错误单独应答，不能覆盖已接收指令的执行结果。 */
        rx_error = error;
    }
    Board_ExitCritical(saved);
    if (have_command)
    {
        enable_command = strncmp(line, "enable ", 7U) == 0;
        if (enable_command)
        {
            command_axis = sscanf(line + 7U, "%u", &enable_axis) == 1 && enable_axis < MOTOR_AXIS_COUNT ? (float)enable_axis : -1.0f;
        }
        if (strcmp(line, "config on") == 0)
        {
            config_mode = true;
            host_mode = false;
            snprintf(reply, sizeof(reply), "OK CONFIG\r\n");
        }
        else if (strcmp(line, "host on") == 0)
        {
            config_mode = false;
            host_mode = true;
            snprintf(reply, sizeof(reply), "OK HOST 1\r\n");
        }
        else if (strcmp(line, "config off") == 0 || strcmp(line, "host off") == 0)
        {
            config_mode = false;
            host_mode = false;
            snprintf(reply, sizeof(reply), "OK WAVEFORM\r\n");
        }
        else if (config_mode || host_mode) { Command_Execute(line, reply, sizeof(reply)); }
        else
        {
            /* VOFA 期间允许 enable，结果由附加通道回传，不混入 ASCII。 */
            if ((strncmp(line, "speed", 5U) == 0 && isspace((unsigned char)line[5])) ||
                (strncmp(line, "enable", 6U) == 0 && isspace((unsigned char)line[6])) ||
                ((strncmp(line, "id", 2U) == 0 || strncmp(line, "iq", 2U) == 0) &&
                 isspace((unsigned char)line[2]))) { Command_Execute(line, reply, sizeof(reply)); }
            else { reply[0] = '\0'; }
        }
        if (enable_command)
        {
            command_result = strncmp(reply, "OK", 2U) == 0 ? 1.0f : -1.0f;
        }
        if (!config_mode && !host_mode && strcmp(line, "config off") != 0 && strcmp(line, "host off") != 0)
        {
            reply[0] = '\0';
        }
        saved = Board_EnterCritical();
        reply_length = (unsigned int)strlen(reply);
        executing = false;
        Board_ExitCritical(saved);
        return true;
    }
    if (error != 0U && (config_mode || host_mode))
    {
        snprintf(reply, sizeof(reply), error == 1U ? "ERR BUSY\r\n" : "ERR LINE_TOO_LONG\r\n");
        reply_length = (unsigned int)strlen(reply);
        return true;
    }
    return config_mode;
}

/* 文本上位机：两轴一拍快照，50 Hz；无有效测量时发 nan，不填虚假零值。 */
static unsigned int UsbVofa_Telemetry(void)
{
    unsigned int axis;
    unsigned int length = 0U;
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        MotorHwSample sample;
        float feedback[7];
        float rpm = NAN, position = NAN, bus = NAN, target;
        uint32_t faults = 0xFFFFFFFFU;
        uint32_t saved;
        const ConfigState *state = ConfigStore_Get(axis);
        if (MotorHw_PeekSample(axis, &sample))
        {
            faults = sample.faults;
            if ((sample.faults & (MOTOR_FAULT_ENCODER | MOTOR_FAULT_STALE)) == 0U)
            {
                rpm = sample.mechanical_speed_rad_s * MotorControl_GetConfig(axis)->encoder_direction * USB_VOFA_RAD_S_TO_RPM;
                position = sample.mechanical_angle_rad * 57.29577951308232f;
            }
            if ((sample.faults & MOTOR_FAULT_BUS_SAMPLE) == 0U) { bus = sample.vbus_v; }
        }
        if ((App_FaultAxes() & (1U << axis)) != 0U) { faults |= 0x100U | App_FaultReason(axis); }
        saved = Board_EnterCritical();
        target = MotorControl_GetSpeedTarget(axis);
        MotorControl_GetFeedback(axis, feedback);
        Board_ExitCritical(saved);
        length += (unsigned int)snprintf(telemetry + length, sizeof(telemetry) - length,
            "T %u %u %u %lu %.7g %.7g %.7g %.7g %.7g %.7g\r\n", axis,
            (unsigned int)((App_RunningAxes() >> axis) & 1U),
            (unsigned int)(state->saved && !state->dirty && state->params.calibrated),
            (unsigned long)faults, (double)rpm, (double)target, (double)position,
            (double)bus, (double)feedback[0], (double)feedback[1]);
    }
    return length;
}

void UsbVofa_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t saved;
    MotorHwSample sample;
    USBD_CDC_HandleTypeDef *cdc;
    float channel[16] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};

    if (UsbVofa_ServiceCommand()) { return; }
    if ((uint32_t)(now - last_send_ms) < (host_mode ? 20U : USB_VOFA_PERIOD_MS)) { return; }
    last_send_ms = now;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return; }
    if (host_mode)
    {
        unsigned int length = UsbVofa_Telemetry();
        saved = Board_EnterCritical();
        cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != 0 && cdc->TxState == 0U)
        {
            (void)CDC_Transmit_FS((uint8_t *)telemetry, (uint16_t)length);
        }
        Board_ExitCritical(saved);
        return;
    }

    /* 诊断读取不消费控制帧，避免让电流环误报重复采样。 */
    if (MotorHw_PeekSample(USB_VOFA_AXIS, &sample))
    {
        if ((sample.faults & (MOTOR_FAULT_MEASUREMENT | MOTOR_FAULT_STALE)) == 0U)
        {
            channel[0] = sample.ia_a;
            channel[1] = sample.ib_a;
            channel[2] = -sample.ia_a - sample.ib_a;
        }
        if (sample.feedback_complete && (sample.faults & MOTOR_FAULT_ENCODER) == 0U)
        {
            channel[3] = sample.mechanical_speed_rad_s * USB_VOFA_RAD_S_TO_RPM;
        }
        if ((sample.faults & MOTOR_FAULT_BUS_SAMPLE) == 0U) { channel[4] = sample.vbus_v; }
    }
    else
    {
        /* 电机后端未就绪时，仍允许单独调试母线 ADC。 */
        (void)AdcMonitor_ReadBusVoltage(&channel[4]);
    }

    /* 同一拍的电流、电压和限幅结果，不在前台重复计算或滤波。 */
    saved = Board_EnterCritical();
    MotorControl_GetFeedback(USB_VOFA_AXIS, &channel[5]);
    Board_ExitCritical(saved);
    channel[10] *= 57.29577951308232f; /* 机械角度 rad 转 deg，前十个通道顺序不变。 */
    channel[12] = (float)(App_RunningAxes() & 1U);
    channel[13] = (float)((App_RunningAxes() >> 1U) & 1U);
    channel[14] = command_axis;
    channel[15] = command_result;

    /* 防止 USB 断开中断释放类句柄；仅在空闲时改写持久发送缓冲区。 */
    saved = Board_EnterCritical();
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != 0 && cdc->TxState == 0U)
    {
        memcpy(tx_frame.channel, channel, sizeof(channel));
        (void)CDC_Transmit_FS((uint8_t *)&tx_frame, sizeof(tx_frame));
    }
    Board_ExitCritical(saved);
}
