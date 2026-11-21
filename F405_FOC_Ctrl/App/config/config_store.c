/** @file config_store.c
 *  @brief 固定小端格式，不把 C 结构体布局写入 EEPROM。
 */
#include "config_store.h"
#include "eeprom.h"
#include "board_critical.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define CONFIG_MAGIC 0x3143464DU /* 字节 "MFC1" */
#define CONFIG_VERSION 1U
#define CONFIG_CRC_OFFSET 252U
#define CONFIG_FLOAT_COUNT 9U
static ConfigState states[MOTOR_AXIS_COUNT];
static struct { unsigned int axis; uint8_t data[EEPROM_PAGE_SIZE]; } tuning_write;

/* 顺序也是磁盘上 float 区（偏移 16）的顺序，调整时须升级版本。 */
static const struct { const char *name; size_t offset; } fields[] = {
    {"phase_resistance_ohm", offsetof(MotorSettings, phase_resistance_ohm)},
    {"ld_h", offsetof(MotorSettings, ld_h)},
    {"lq_h", offsetof(MotorSettings, lq_h)},
    {"flux_linkage_wb", offsetof(MotorSettings, flux_linkage_wb)},
    {"current_limit_a", offsetof(MotorSettings, current_limit_a)},
    {"speed_kp", offsetof(MotorSettings, speed_kp)},
    {"speed_ki", offsetof(MotorSettings, speed_ki)},
    {"electrical_zero_rad", offsetof(MotorSettings, electrical_zero_rad)},
    {"cogging_gain", offsetof(MotorSettings, cogging_gain)}
};

static void Put32(uint8_t *p, uint32_t value)
{
    unsigned int i;
    for (i = 0U; i < 4U; ++i) { p[i] = (uint8_t)(value >> (8U * i)); }
}
static uint32_t Get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}
/* CRC-32/ISO-HDLC：初值/末异或 0xFFFFFFFF，反射多项式 0xEDB88320。 */
static uint32_t ConfigCrc(const uint8_t *data, unsigned int size)
{
    uint32_t crc = 0xFFFFFFFFU;
    unsigned int i;
    while (size-- != 0U)
    {
        crc ^= *data++;
        for (i = 0U; i < 8U; ++i) { crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U); }
    }
    return crc ^ 0xFFFFFFFFU;
}
static void Defaults(MotorSettings *p)
{
    MotorConfig config;
    MotorControl_DefaultConfig(&config);
    memset(p, 0, sizeof(*p));
    p->phase_resistance_ohm = 0.12f;
    p->ld_h = config.foc.ld_h;
    p->lq_h = config.foc.lq_h;
    p->flux_linkage_wb = config.foc.flux_linkage_wb;
    p->current_limit_a = config.current_limit_a;
    p->speed_kp = config.speed_kp;
    p->speed_ki = config.speed_ki;
    p->pole_pairs = config.pole_pairs;
    p->encoder_direction = 1;
    p->cogging_gain = 1.0f;
    p->cogging_points = CONFIG_COGGING_POINTS;
}
static bool Valid(const MotorSettings *p)
{
    unsigned int i;
    for (i = 0U; i < CONFIG_FLOAT_COUNT; ++i)
    {
        const float *value = (const float *)((const uint8_t *)p + fields[i].offset);
        if (!isfinite(*value)) { return false; }
    }
    return p->phase_resistance_ohm > 0.0f && p->ld_h > 0.0f && p->lq_h > 0.0f &&
           p->flux_linkage_wb >= 0.0f && p->current_limit_a > 0.0f && p->current_limit_a <= 10.0f &&
           p->speed_kp >= 0.0f && p->speed_ki >= 0.0f && p->pole_pairs != 0U &&
           (p->encoder_direction == 1 || p->encoder_direction == -1) &&
           p->electrical_zero_rad >= 0.0f && p->electrical_zero_rad < 6.2831853071795865f &&
           p->cogging_gain >= 0.0f && p->calibrated <= 1U &&
           p->cogging_valid <= 1U && p->cogging_points == CONFIG_COGGING_POINTS &&
           (!p->cogging_valid || p->calibrated);
}
static void Encode(const MotorSettings *p, uint8_t data[CONFIG_HEADER_BYTES])
{
    unsigned int i;
    uint32_t bits;
    float scale = CONFIG_COGGING_SCALE_A;
    memset(data, 0, CONFIG_HEADER_BYTES);
    Put32(data, CONFIG_MAGIC);
    Put32(data + 4U, CONFIG_VERSION);
    Put32(data + 8U, CONFIG_HEADER_BYTES);
    for (i = 0U; i < CONFIG_FLOAT_COUNT; ++i)
    {
        memcpy(&bits, (const uint8_t *)p + fields[i].offset, sizeof(bits));
        Put32(data + 16U + 4U * i, bits);
    }
    data[52] = (uint8_t)p->pole_pairs;
    data[53] = (uint8_t)(p->pole_pairs >> 8U);
    data[54] = p->encoder_direction == 1 ? 1U : 0xFFU;
    data[55] = p->calibrated;
    data[56] = p->cogging_valid;
    /* 偏移57保留为0；读取旧版记录时忽略旧补偿开关，强度0即关闭。 */
    data[58] = (uint8_t)p->cogging_points;
    data[59] = (uint8_t)(p->cogging_points >> 8U);
    Put32(data + 60U, 1U); /* 表格式1：小端 int16，原始机械角一圈，线性插值。 */
    memcpy(&bits, &scale, sizeof(bits));
    Put32(data + 64U, bits);
    Put32(data + 68U, p->cogging_crc);
    Put32(data + CONFIG_CRC_OFFSET, ConfigCrc(data, CONFIG_CRC_OFFSET));
}
static bool Decode(const uint8_t data[CONFIG_HEADER_BYTES], MotorSettings *p)
{
    unsigned int i;
    uint32_t bits;
    float scale = CONFIG_COGGING_SCALE_A;
    memcpy(&bits, &scale, sizeof(bits));
    if (Get32(data) != CONFIG_MAGIC || Get32(data + 4U) != CONFIG_VERSION ||
        Get32(data + 8U) != CONFIG_HEADER_BYTES || Get32(data + 60U) != 1U ||
        Get32(data + 64U) != bits ||
        Get32(data + CONFIG_CRC_OFFSET) != ConfigCrc(data, CONFIG_CRC_OFFSET)) { return false; }
    memset(p, 0, sizeof(*p));
    for (i = 0U; i < CONFIG_FLOAT_COUNT; ++i)
    {
        bits = Get32(data + 16U + 4U * i);
        memcpy((uint8_t *)p + fields[i].offset, &bits, sizeof(bits));
    }
    p->pole_pairs = (uint16_t)(data[52] | ((uint16_t)data[53] << 8U));
    if (data[54] != 1U && data[54] != 0xFFU) { return false; }
    p->encoder_direction = data[54] == 1U ? 1 : -1;
    p->calibrated = data[55];
    p->cogging_valid = data[56];
    p->cogging_points = (uint16_t)(data[58] | ((uint16_t)data[59] << 8U));
    p->cogging_crc = Get32(data + 68U);
    return Valid(p);
}
const ConfigState *ConfigStore_Get(unsigned int axis) { return &states[axis]; }

static bool TuningValid(float kp, float ki, float gain)
{
    return isfinite(kp) && kp >= 0.0f && isfinite(ki) && ki >= 0.0f && isfinite(gain) && gain >= 0.0f;
}

static void LoadTuning(unsigned int axis)
{
    uint8_t data[EEPROM_PAGE_SIZE];
    float values[3];
    uint32_t bits;
    unsigned int i;
    ConfigState *state = &states[axis];
    state->tuning_saved = false;
    state->tuning_result = CONFIG_INVALID;
    if (!Eeprom_Read((uint16_t)(axis * CONFIG_AXIS_BYTES + CONFIG_TUNING_OFFSET), data, sizeof(data)))
    {
        state->tuning_result = CONFIG_IO_ERROR;
        return;
    }
    if (Get32(data) != 0x314E5554U || Get32(data + 4U) != 1U ||
        Get32(data + 28U) != ConfigCrc(data, 28U)) { return; }
    for (i = 0U; i < 3U; ++i)
    {
        bits = Get32(data + 8U + i * 4U);
        memcpy(&values[i], &bits, sizeof(bits));
    }
    if (!TuningValid(values[0], values[1], values[2])) { return; }
    state->params.speed_kp = values[0];
    state->params.speed_ki = values[1];
    state->params.cogging_gain = values[2];
    state->tuning_saved = true;
    state->tuning_result = CONFIG_OK;
}
ConfigResult ConfigStore_Load(unsigned int axis)
{
    uint8_t data[CONFIG_HEADER_BYTES];
    ConfigState *state = &states[axis];
    state->saved = false;
    state->dirty = false;
    Defaults(&state->params);
    if (!Eeprom_Read((uint16_t)(axis * CONFIG_AXIS_BYTES), data, sizeof(data)))
    {
        state->result = CONFIG_IO_ERROR;
        goto tuning;
    }
    if (!Decode(data, &state->params))
    {
        Defaults(&state->params);
        state->result = CONFIG_INVALID;
        goto tuning;
    }
    state->saved = true;
    if (state->params.cogging_valid)
    {
        int16_t *table = Cogging_Table(axis);
        ConfigResult result = CONFIG_OK;
        if (!Eeprom_Read((uint16_t)(axis * CONFIG_AXIS_BYTES + CONFIG_COGGING_OFFSET),
                         table, CONFIG_COGGING_POINTS * sizeof(*table))) { result = CONFIG_IO_ERROR; }
        else if (ConfigCrc((const uint8_t *)table, CONFIG_COGGING_POINTS * sizeof(*table)) !=
                 state->params.cogging_crc) { result = CONFIG_VERIFY_ERROR; }
        if (result != CONFIG_OK)
        {
            /* 表损坏只关闭补偿，仍保留已校准的普通电机参数并报告错误。 */
            state->params.cogging_valid = 0U;
            state->result = result;
            goto tuning;
        }
    }
    state->result = CONFIG_OK;
tuning:
    LoadTuning(axis);
    return state->result;
}
void ConfigStore_Init(void)
{
    unsigned int axis;
    for (axis = 0U; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        (void)ConfigStore_Load(axis);
        ConfigStore_Apply(axis);
    }
}
bool ConfigStore_Set(unsigned int axis, const char *name, float value)
{
    MotorSettings next = states[axis].params;
    unsigned int i;
    if (!isfinite(value)) { return false; }
    if (strcmp(name, "pole_pairs") == 0)
    {
        if (value < 1.0f || value > 65535.0f || floorf(value) != value) { return false; }
        next.pole_pairs = (uint16_t)value;
    }
    else
    {
        /* param set 仅修改电机固有参数；调试参数使用独立tune命令。 */
        for (i = 0U; i < 5U; ++i)
        {
            if (strcmp(name, fields[i].name) == 0) { break; }
        }
        if (i == 5U) { return false; }
        *(float *)((uint8_t *)&next + fields[i].offset) = value;
    }
    if (!Valid(&next)) { return false; }
    states[axis].params = next;
    states[axis].dirty = true;
    return true;
}
ConfigResult ConfigStore_Save(unsigned int axis, const MotorSettings *params)
{
    uint8_t data[CONFIG_HEADER_BYTES];
    uint8_t verify[CONFIG_HEADER_BYTES];
    uint8_t invalid[4] = {0};
    uint16_t address = (uint16_t)(axis * CONFIG_AXIS_BYTES);
    ConfigState *state = &states[axis];
    if (!Valid(params)) { return CONFIG_INVALID; }
    Encode(params, data);
    state->saved = false;
    /* 先失效旧记录，再写正文，最后提交标记，半记录不作为已校准数据。 */
    if (!Eeprom_Write(address, invalid, sizeof(invalid)) ||
        !Eeprom_Write(address + 4U, data + 4U, sizeof(data) - 4U) ||
        !Eeprom_Write(address, data, 4U) || !Eeprom_Read(address, verify, sizeof(verify)))
    {
        return state->result = CONFIG_IO_ERROR;
    }
    if (memcmp(data, verify, sizeof(data)) != 0) { return state->result = CONFIG_VERIFY_ERROR; }
    state->params = *params;
    state->dirty = false;
    state->saved = true;
    return state->result = CONFIG_OK;
}
void ConfigStore_Apply(unsigned int axis)
{
    MotorConfig config;
    const MotorSettings *p = &states[axis].params;
    MotorControl_DefaultConfig(&config);
    config.foc.ld_h = p->ld_h;
    config.foc.lq_h = p->lq_h;
    config.foc.flux_linkage_wb = p->flux_linkage_wb;
    config.current_limit_a = p->current_limit_a;
    config.speed_kp = p->speed_kp;
    config.speed_ki = p->speed_ki;
    config.pole_pairs = p->pole_pairs;
    config.encoder_direction = p->encoder_direction;
    config.electrical_zero_rad = p->electrical_zero_rad;
    MotorControl_Configure(axis, &config);
    Cogging_SetGain(axis, p->calibrated && p->cogging_valid ? p->cogging_gain : 0.0f);
}

ConfigResult ConfigStore_SaveCogging(unsigned int axis, const MotorSettings *zero)
{
    MotorSettings params = *zero;
    int16_t *table = Cogging_Table(axis);
    uint16_t bytes = CONFIG_COGGING_POINTS * sizeof(*table);
    uint16_t address = (uint16_t)(axis * CONFIG_AXIS_BYTES + CONFIG_COGGING_OFFSET);
    uint32_t crc = ConfigCrc((const uint8_t *)table, bytes);
    if (!Eeprom_Write(address, table, bytes) || !Eeprom_Read(address, table, bytes))
    {
        return states[axis].result = CONFIG_IO_ERROR;
    }
    if (ConfigCrc((const uint8_t *)table, bytes) != crc)
    {
        return states[axis].result = CONFIG_VERIFY_ERROR;
    }
    params.cogging_crc = crc;
    params.cogging_valid = 1U;
    return ConfigStore_Save(axis, &params);
}

ConfigResult ConfigStore_StartTuning(unsigned int axis, float kp, float ki, float gain)
{
    float values[3];
    uint32_t bits;
    unsigned int i;
    ConfigState *state = &states[axis];
    if (!TuningValid(kp, ki, gain)) { return CONFIG_INVALID; }
    values[0] = kp; values[1] = ki; values[2] = gain;
    memset(tuning_write.data, 0, sizeof(tuning_write.data));
    Put32(tuning_write.data, 0x314E5554U); /* TUN1 */
    Put32(tuning_write.data + 4U, 1U);
    for (i = 0U; i < 3U; ++i)
    {
        memcpy(&bits, &values[i], sizeof(bits));
        Put32(tuning_write.data + 8U + i * 4U, bits);
    }
    Put32(tuning_write.data + 28U, ConfigCrc(tuning_write.data, 28U));
    if (!Eeprom_WritePageStart((uint16_t)(axis * CONFIG_AXIS_BYTES + CONFIG_TUNING_OFFSET), tuning_write.data))
    {
        return state->tuning_result = CONFIG_IO_ERROR;
    }
    tuning_write.axis = axis;
    state->tuning_saved = false;
    return state->tuning_result = CONFIG_PENDING;
}

ConfigResult ConfigStore_PollTuning(void)
{
    EepromResult result = Eeprom_WritePagePoll();
    ConfigState *state = &states[tuning_write.axis];
    uint32_t bits, saved;
    float values[3];
    unsigned int i;
    if (result == EEPROM_PENDING) { return CONFIG_PENDING; }
    if (result != EEPROM_COMPLETE)
    {
        return state->tuning_result = result == EEPROM_VERIFY_ERROR ? CONFIG_VERIFY_ERROR : CONFIG_IO_ERROR;
    }
    for (i = 0U; i < 3U; ++i)
    {
        bits = Get32(tuning_write.data + 8U + i * 4U);
        memcpy(&values[i], &bits, sizeof(bits));
    }
    /* 仅提交三个调试值，保持当前目标、模式、积分及已保存的校准信息。 */
    saved = Board_EnterCritical();
    state->params.speed_kp = values[0];
    state->params.speed_ki = values[1];
    state->params.cogging_gain = values[2];
    MotorControl_SetSpeedGains(tuning_write.axis, values[0], values[1]);
    Cogging_SetGain(tuning_write.axis, state->params.calibrated && state->params.cogging_valid ? values[2] : 0.0f);
    Board_ExitCritical(saved);
    state->tuning_saved = true;
    return state->tuning_result = CONFIG_OK;
}
