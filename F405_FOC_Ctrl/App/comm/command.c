/** @file command.c
 *  @brief 统一命令校验和电机操作协调。
 */
#include "command.h"
#include "main.h"
#include "app.h"
#include "config_store.h"
#include "board_critical.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define COMMAND_ALL_AXES ((1U << MOTOR_AXIS_COUNT) - 1U)
static CommandCalibrationStage calibration_stage;

CommandCalibrationStage Command_CalibrationStage(void) { return calibration_stage; }

static MotorCalibrationResult Command_FindZero(unsigned int axis, MotorSettings *params)
{
    MotorConfig config;
    MotorCalibrationResult result;
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U || axis >= MOTOR_AXIS_COUNT)
    {
        return MOTOR_CALIBRATION_INVALID_ARGUMENT;
    }
    if (calibration_stage != COMMAND_CAL_IDLE || MotorCalibration_ActiveAxis() >= 0) { return MOTOR_CALIBRATION_BUSY; }
    if (App_RunningAxes() != 0U) { return MOTOR_CALIBRATION_BUSY; }
    if (ConfigStore_Get(axis)->dirty) { return MOTOR_CALIBRATION_NOT_READY; }
    calibration_stage = COMMAND_CAL_ZERO;
    App_DisableOutputs(COMMAND_ALL_AXES);
    *params = ConfigStore_Get(axis)->params;
    params->calibrated = params->cogging_valid = 0U;
    params->cogging_crc = 0U;
    result = MOTOR_CALIBRATION_STORAGE_ERROR;
    if (ConfigStore_Save(axis, params) != CONFIG_OK) { goto done; }
    ConfigStore_Apply(axis);
    config = *MotorControl_GetConfig(axis);
    result = MotorCalibration_Run(axis, &config);
    if (result != MOTOR_CALIBRATION_OK) { goto done; }
    params->encoder_direction = config.encoder_direction;
    params->electrical_zero_rad = config.electrical_zero_rad;
    params->calibrated = 1U;
done:
    calibration_stage = COMMAND_CAL_IDLE;
    return result;
}

MotorCalibrationResult Command_CalibrateMotor(unsigned int axis)
{
    MotorSettings params;
    MotorCalibrationResult result = Command_FindZero(axis, &params);
    if (result == MOTOR_CALIBRATION_OK && ConfigStore_Save(axis, &params) != CONFIG_OK)
    {
        result = MOTOR_CALIBRATION_STORAGE_ERROR;
    }
    return result;
}

static const char *StoreReply(ConfigResult result)
{
    switch (result)
    {
        case CONFIG_OK: return "OK\r\n";
        case CONFIG_INVALID: return "ERR INVALID_CONFIG\r\n";
        case CONFIG_IO_ERROR: return "ERR EEPROM_IO\r\n";
        default: return "ERR EEPROM_VERIFY\r\n";
    }
}

/* 调用方已确认两轴停止、参数已提交、零位有效。联合入口和单独重标定共用此流程。 */
static void Command_CalibrateCogging(unsigned int axis, const MotorSettings *zero, char *reply, size_t capacity)
{
    bool full = zero != NULL;
    MotorSettings params = full ? *zero : ConfigStore_Get(axis)->params;
    ConfigResult stored;
    CoggingCalibrationResult result;
    const char *error = full ? "CALIBRATION_COGGING" : "COGGING_CALIBRATION";
    calibration_stage = COMMAND_CAL_COGGING;
    if (!full)
    {
        params.cogging_valid = 0U;
        params.cogging_crc = 0U;
        stored = ConfigStore_Save(axis, &params);
        if (stored != CONFIG_OK) { goto storage_error; }
        ConfigStore_Apply(axis);
    }
    result = MotorCalibration_RunCogging(axis);
    if (result != COGGING_CAL_OK)
    {
        snprintf(reply, capacity, "ERR %s %u\r\n", error, (unsigned int)result);
        goto done;
    }
    stored = ConfigStore_SaveCogging(axis, &params);
    ConfigStore_Apply(axis);
    if (stored != CONFIG_OK) { goto storage_error; }
    snprintf(reply, capacity, "%s", full ? "OK CALIBRATED_FULL\r\n" : "OK COGGING_CALIBRATED\r\n");
    goto done;
storage_error:
    snprintf(reply, capacity, "ERR %s_STORAGE %s", error, StoreReply(stored) + 4U);
done:
    calibration_stage = COMMAND_CAL_IDLE;
}

void Command_CalibrateFull(unsigned int axis, char *reply, size_t capacity)
{
    MotorSettings params;
    MotorCalibrationResult result = Command_FindZero(axis, &params);
    if (result != MOTOR_CALIBRATION_OK)
    {
        snprintf(reply, capacity, "ERR CALIBRATION_ZERO %u\r\n", (unsigned int)result);
        return;
    }
    Command_CalibrateCogging(axis, &params, reply, capacity);
}

void Command_Execute(char *line, char *reply, size_t capacity)
{
    char *args[6];
    unsigned int count = 0U;
    unsigned int axis;
    unsigned long number;
    char *cursor = line;
    char *end;
    const char *message = "ERR COMMAND\r\n";
    const ConfigState *state;
    ConfigResult store_result;
    MotorSettings params;
    float value;
    uint32_t saved;
    while (*cursor != '\0')
    {
        while (isspace((unsigned char)*cursor)) { ++cursor; }
        if (*cursor == '\0') { break; }
        if (count == sizeof(args) / sizeof(args[0])) { goto respond; }
        args[count++] = cursor;
        while (*cursor != '\0' && !isspace((unsigned char)*cursor)) { ++cursor; }
        if (*cursor != '\0') { *cursor++ = '\0'; }
    }
    if (count == 0U) { goto respond; }
    if (calibration_stage != COMMAND_CAL_IDLE || MotorCalibration_ActiveAxis() >= 0)
    {
        message = "ERR BUSY\r\n";
        goto respond;
    }
    if (strcmp(args[0], "param") == 0 || strcmp(args[0], "cogging") == 0)
    {
        if (count < 3U) { goto respond; }
        cursor = args[2];
    }
    else
    {
        if (count < 2U) { goto respond; }
        /* 保留原 speed/id/iq <值>，默认 M0；新格式可显式选择轴。 */
        if (count == 2U && (strcmp(args[0], "speed") == 0 || strcmp(args[0], "id") == 0 ||
                            strcmp(args[0], "iq") == 0)) { cursor = "0"; }
        else { cursor = args[1]; }
    }
    number = strtoul(cursor, &end, 10);
    if (*cursor == '\0' || *end != '\0' || number >= MOTOR_AXIS_COUNT)
    {
        message = "ERR AXIS\r\n";
        goto respond;
    }
    axis = (unsigned int)number;
    state = ConfigStore_Get(axis);
    if (strcmp(args[0], "param") == 0)
    {
        if (strcmp(args[1], "show") == 0 && count == 3U)
        {
            const MotorSettings *p = &state->params;
            snprintf(reply, capacity,
                "OK axis=%u saved=%u dirty=%u result=%u running=%u tuning_saved=%u tuning_result=%u\r\n"
                "phase_resistance_ohm=%.9g\r\nld_h=%.9g\r\nlq_h=%.9g\r\n"
                "flux_linkage_wb=%.9g\r\npole_pairs=%u\r\ncurrent_limit_a=%.9g\r\n"
                "speed_kp=%.9g\r\nspeed_ki=%.9g\r\ncalibrated=%u\r\n"
                "encoder_direction=%d\r\nelectrical_zero_rad=%.9g\r\n"
                "cogging_gain=%.9g\r\ncogging_valid=%u\r\n"
                "cogging_points=%u\r\ncogging_scale_a=0.001\r\nEND\r\n",
                axis, (unsigned int)state->saved, (unsigned int)state->dirty,
                (unsigned int)state->result, (unsigned int)((App_RunningAxes() >> axis) & 1U),
                (unsigned int)state->tuning_saved, (unsigned int)state->tuning_result,
                (double)p->phase_resistance_ohm, (double)p->ld_h, (double)p->lq_h,
                (double)p->flux_linkage_wb, p->pole_pairs, (double)p->current_limit_a,
                (double)p->speed_kp, (double)p->speed_ki, p->calibrated, p->encoder_direction,
                (double)p->electrical_zero_rad, (double)p->cogging_gain, p->cogging_valid, p->cogging_points);
            return;
        }
        if (strcmp(args[1], "set") == 0 && count == 5U)
        {
            value = strtof(args[4], &end);
            message = end != args[4] && *end == '\0' && ConfigStore_Set(axis, args[3], value) ?
                      "OK STAGED\r\n" : "ERR PARAMETER\r\n";
            goto respond;
        }
        if (count != 3U) { goto respond; }
        if (strcmp(args[1], "save") != 0 && strcmp(args[1], "reload") != 0) { goto respond; }
        if (App_RunningAxes() != 0U) { message = "ERR STOP_BOTH_AXES\r\n"; goto respond; }
        if (strcmp(args[1], "save") == 0)
        {
            params = state->params;
            params.calibrated = params.cogging_valid = 0U;
            params.cogging_crc = 0U;
            store_result = ConfigStore_Save(axis, &params);
            if (store_result == CONFIG_OK) { ConfigStore_Apply(axis); }
        }
        else
        {
            store_result = ConfigStore_Load(axis);
            /* 读取失败时应用未校准初值，仍报告原始错误。 */
            ConfigStore_Apply(axis);
        }
        message = StoreReply(store_result);
    }
    else if (strcmp(args[0], "tune") == 0 && count == 5U)
    {
        float values[3];
        unsigned int i;
        for (i = 0U; i < 3U; ++i)
        {
            values[i] = strtof(args[i + 2U], &end);
            if (end == args[i + 2U] || *end != '\0') { message = "ERR VALUE\r\n"; goto respond; }
        }
        store_result = ConfigStore_StartTuning(axis, values[0], values[1], values[2]);
        while (store_result == CONFIG_PENDING)
        {
            App_Poll(); /* I2C中断传输和芯片写周期期间继续编码器/控制服务。 */
            store_result = ConfigStore_PollTuning();
        }
        message = StoreReply(store_result);
    }
    else if (strcmp(args[0], "cogging") == 0 && strcmp(args[1], "calibrate") == 0 && count == 3U)
    {
        if (App_RunningAxes() != 0U) { message = "ERR STOP_BOTH_AXES\r\n"; goto respond; }
        if (!state->saved || state->dirty || !state->params.calibrated)
        {
            message = "ERR SAVE_AND_CALIBRATE\r\n"; goto respond;
        }
        Command_CalibrateCogging(axis, NULL, reply, capacity);
        return;
    }
    else if (strcmp(args[0], "enable") == 0 && count == 3U)
    {
        unsigned int enabled;
        const char *reason;
        if (strcmp(args[2], "0") != 0 && strcmp(args[2], "1") != 0)
        {
            message = "ERR ENABLE_VALUE\r\n"; goto respond;
        }
        if (args[2][0] == '0') { App_DisableOutputs((uint8_t)(1U << axis)); }
        else { (void)App_EnableOutputs((uint8_t)(1U << axis)); }
        enabled = (App_RunningAxes() >> axis) & 1U;
        if (enabled == (unsigned int)(args[2][0] - '0'))
        {
            snprintf(reply, capacity, "OK ENABLE axis=%u state=%u\r\n", axis, enabled);
        }
        else
        {
            reason = !state->saved ? "UNSAVED" : state->dirty ? "UNSAVED_CHANGES" :
                     !state->params.calibrated ? "UNCALIBRATED" :
                     (App_FaultAxes() & (1U << axis)) != 0U ? "FAULT" : "NOT_READY";
            snprintf(reply, capacity, "ERR ENABLE axis=%u state=%u reason=%s\r\n", axis, enabled, reason);
        }
        return;
    }
    else if (strcmp(args[0], "position") == 0 && count == 3U)
    {
        message = "ERR POSITION_NOT_IMPLEMENTED\r\n";
    }
    else if (strcmp(args[0], "stop") == 0 && count == 2U)
    {
        App_DisableOutputs((uint8_t)(1U << axis));
        message = "OK\r\n";
    }
    else if (strcmp(args[0], "start") == 0 && count == 2U)
    {
        message = (App_EnableOutputs((uint8_t)(1U << axis)) & (1U << axis)) != 0U ?
                  "OK\r\n" : "ERR NOT_READY\r\n";
    }
    else if (strcmp(args[0], "calibrate") == 0 && count == 2U)
    {
        Command_CalibrateFull(axis, reply, capacity);
        return;
    }
    else if ((strcmp(args[0], "speed") == 0 || strcmp(args[0], "id") == 0 ||
              strcmp(args[0], "iq") == 0) && (count == 2U || count == 3U))
    {
        value = strtof(args[count - 1U], &end);
        if (end == args[count - 1U] || *end != '\0' || !isfinite(value))
        {
            message = "ERR VALUE\r\n"; goto respond;
        }
        if ((App_RunningAxes() & (1U << axis)) == 0U)
        {
            message = "ERR STOPPED\r\n"; goto respond;
        }
        if (strcmp(args[0], "speed") != 0 && fabsf(value) > MotorControl_GetConfig(axis)->current_limit_a)
        {
            message = "ERR CURRENT_LIMIT\r\n"; goto respond;
        }
        saved = Board_EnterCritical();
        if (strcmp(args[0], "speed") == 0) { MotorControl_SetSpeedTarget(axis, value); }
        else { MotorControl_SetCurrentTarget(axis, args[0][1] == 'd' ? value : 0.0f,
                                             args[0][1] == 'q' ? value : 0.0f); }
        Board_ExitCritical(saved);
        message = "OK\r\n";
    }
respond:
    snprintf(reply, capacity, "%s", message);
}
