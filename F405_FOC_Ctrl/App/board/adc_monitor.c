/**
 * @file adc_monitor.c
 * @brief 母线电压与两路 NTC 温度监测。
 */
#include "adc_monitor.h"
#include "board_critical.h"
#include "adc.h"

#define BUS_SAMPLE_PERIOD_MS 10U
#define BUS_SAMPLE_TIMEOUT_MS (3U * BUS_SAMPLE_PERIOD_MS)
#define BUS_VOLTS_PER_COUNT (3.3f / 4096.0f * 21.0f) /* PA4，200 kΩ / 10 kΩ 分压。 */

static float bus_voltage_v;
static uint32_t bus_raw; /* 常规 SRAM，DMA 为半字传输；使用 32 位变量保证地址对齐。 */
static uint32_t bus_time_ms;
static uint32_t started_ms;
static volatile bool bus_valid;

void AdcMonitor_Init(void)
{
    /* 在 DMA 采样启动前初始化；此时没有缓存发布者。 */
    bus_valid = false;
    started_ms = HAL_GetTick() - BUS_SAMPLE_PERIOD_MS;
    AdcMonitor_Service();
}

bool AdcMonitor_Stop(void)
{
    HAL_StatusTypeDef result = HAL_ADC_Stop_DMA(&hadc1);
    bus_valid = false;
    return result == HAL_OK;
}

void AdcMonitor_Service(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - started_ms) >= BUS_SAMPLE_PERIOD_MS &&
        HAL_DMA_GetState(hadc1.DMA_Handle) == HAL_DMA_STATE_READY)
    {
        started_ms = now; /* 先记录时间，避免 DMA 完成回调先于启动函数返回。 */
        if (HAL_ADC_Start_DMA(&hadc1, &bus_raw, 1U) != HAL_OK)
        {
            /* 单字节失效标记，无需保护多字段快照。 */
            bus_valid = false;
        }
    }
}

/** @brief ADC1 规则组单次 DMA 完成；保持 ADC 运行，不影响注入组电流采样。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        float voltage = (float)bus_raw * BUS_VOLTS_PER_COUNT;
        uint32_t saved = Board_EnterCritical();
        bus_voltage_v = voltage;
        bus_time_ms = started_ms;
        bus_valid = true;
        Board_ExitCritical(saved);
    }
}

bool AdcMonitor_ReadBusVoltage(float *voltage_v)
{
    uint32_t saved;
    uint32_t sample_time;
    float voltage;
    bool valid;
    if (voltage_v == 0) { return false; }
    saved = Board_EnterCritical();
    valid = bus_valid;
    sample_time = bus_time_ms;
    voltage = bus_voltage_v;
    Board_ExitCritical(saved);
    if (!valid || (uint32_t)(HAL_GetTick() - sample_time) > BUS_SAMPLE_TIMEOUT_MS) { return false; }
    *voltage_v = voltage;
    return true;
}

bool AdcMonitor_ReadNtcRaw(unsigned int axis, uint16_t *raw)
{
    (void)axis;
    (void)raw;
    return false;
}
