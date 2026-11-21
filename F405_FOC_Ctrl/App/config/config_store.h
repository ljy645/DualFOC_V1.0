/** @file config_store.h
 *  @brief 每轴参数与 24LC64 固定格式，仅由前台串行调用。
 */
#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H
#include "motor_control.h"
#include "cogging.h"

#define CONFIG_AXIS_BYTES 4096U
#define CONFIG_HEADER_BYTES 256U
#define CONFIG_COGGING_POINTS COGGING_POINTS
#define CONFIG_COGGING_OFFSET 256U
#define CONFIG_COGGING_SCALE_A COGGING_SCALE_A
#define CONFIG_TUNING_OFFSET 0x900U /* 每轴独立32字节页，不与零位/补偿表共写。 */

typedef struct
{
    float phase_resistance_ohm;
    float ld_h;
    float lq_h;
    float flux_linkage_wb;
    float current_limit_a;
    float speed_kp;
    float speed_ki;
    float electrical_zero_rad;
    float cogging_gain;
    uint16_t pole_pairs;
    int8_t encoder_direction;
    uint8_t calibrated;
    uint8_t cogging_valid;
    uint16_t cogging_points;
    uint32_t cogging_crc;
} MotorSettings;

typedef enum { CONFIG_OK = 0, CONFIG_INVALID, CONFIG_IO_ERROR, CONFIG_VERIFY_ERROR, CONFIG_PENDING } ConfigResult;
typedef struct
{
    MotorSettings params; /* 待保存配置，与控制环使用的配置分开。 */
    bool saved;
    bool dirty;
    ConfigResult result;
    bool tuning_saved;
    ConfigResult tuning_result;
} ConfigState;

/* 轴号由 command 层检查。Load/Save/SaveCogging 期间两轴停止，中断保持开启。
 * Load 失败装入明确标记为未保存、未校准的初值，不写 EEPROM。
 */
void ConfigStore_Init(void);
const ConfigState *ConfigStore_Get(unsigned int axis);
ConfigResult ConfigStore_Load(unsigned int axis);
bool ConfigStore_Set(unsigned int axis, const char *name, float value);
/* 写入并读回核对；失败后 RAM 不再认为该轴有可用的已保存配置。 */
ConfigResult ConfigStore_Save(unsigned int axis, const MotorSettings *params);
/* 两轴停止时写表并核对CRC，再统一保存传入的零位和补偿有效位。
 * 调用前必须已持久化清除旧补偿有效位；本函数不重复写失效记录。
 */
ConfigResult ConfigStore_SaveCogging(unsigned int axis, const MotorSettings *zero);
/* 前台串行开始/轮询单页写入。两次 Poll 之间继续服务 App_Poll，可运行中调用。
 * 成功核对后热更新Kp/Ki/补偿强度，保留目标、积分、零位和表；失败保留运行值。
 */
ConfigResult ConfigStore_StartTuning(unsigned int axis, float kp, float ki, float gain);
ConfigResult ConfigStore_PollTuning(void);
/* 闭环暂停时装入参数，电流 PI 仍来自程序默认值。 */
void ConfigStore_Apply(unsigned int axis);
#endif
