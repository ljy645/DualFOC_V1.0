/**
 * @file drv8303.c
 * @brief 双路 DRV8303 配置与故障读取。
 */
#include "drv8303.h"
#include "motor_hw.h"
#include "spi.h"

static bool Drv8303_Transfer(unsigned int axis, uint16_t command, uint16_t *response)
{
    GPIO_TypeDef *port = (axis == MOTOR_AXIS_M0) ? M0_SPI_CS_GPIO_Port : M1_SPI_CS_GPIO_Port;
    uint16_t pin = (axis == MOTOR_AXIS_M0) ? M0_SPI_CS_Pin : M1_SPI_CS_Pin;
    unsigned int delay;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    for (delay = 0U; delay < 16U; ++delay) { __NOP(); }
    status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&command,
                                    (uint8_t *)response, 1U, 2U);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    for (delay = 0U; delay < 16U; ++delay) { __NOP(); }
    return status == HAL_OK;
}

static bool Drv8303_ReadRegister(unsigned int axis, uint16_t address, uint16_t *value)
{
    uint16_t command = (uint16_t)(0x8000U | (address << 11U));
    uint16_t response;
    /* 读取响应滞后一帧。 */
    if (!Drv8303_Transfer(axis, command, &response) ||
        !Drv8303_Transfer(axis, command, &response) ||
        (response & 0xF800U) != (uint16_t)(address << 11U))
    {
        return false;
    }
    *value = response & 0x07FFU;
    return true;
}

/** @brief 写入寄存器并读回确认，供增益和校准设置共用。 */
static bool Drv8303_WriteRegister(unsigned int axis, uint16_t address, uint16_t value)
{
    uint16_t response;
    uint16_t readback;
    return Drv8303_Transfer(axis, (uint16_t)((address << 11U) | value), &response) &&
           Drv8303_ReadRegister(axis, address, &readback) && readback == value;
}

bool Drv8303_SetCurrentGain(unsigned int axis, unsigned int gain)
{
    uint16_t bits;
    uint16_t control;
    switch (gain)
    {
        case 10U: bits = 0U; break;
        case 20U: bits = 1U; break;
        case 40U: bits = 2U; break;
        case 80U: bits = 3U; break;
        default: return false;
    }
    if (axis >= MOTOR_AXIS_COUNT || !Drv8303_ReadRegister(axis, 3U, &control))
    {
        return false;
    }
    /* 控制寄存器 2 的 D3:D2 为增益，其余位保持原值。 */
    control = (uint16_t)((control & ~0x000CU) | (bits << 2U));
    return Drv8303_WriteRegister(axis, 3U, control);
}

bool Drv8303_SetCalibration(unsigned int axis, bool enable)
{
    uint16_t control;
    if (axis >= MOTOR_AXIS_COUNT || !Drv8303_ReadRegister(axis, 3U, &control))
    {
        return false;
    }
    /* 控制寄存器 2 的 D4/D5：同时短接两路放大器输入，保留增益及其他配置。 */
    control = (uint16_t)((control & ~0x0030U) | (enable ? 0x0030U : 0U));
    return Drv8303_WriteRegister(axis, 3U, control);
}

bool Drv8303_CheckStartup(unsigned int axis)
{
    uint16_t control1;
    uint16_t control2;
    uint16_t status;
    /* SPI2 的 16 位 mode 1 配置统一由 CubeMX 初始化。 */
    if (axis >= MOTOR_AXIS_COUNT)
    {
        return false;
    }
    if (!Drv8303_ReadRegister(axis, 2U, &control1) ||
        !Drv8303_ReadRegister(axis, 3U, &control2) ||
        !Drv8303_ReadRegister(axis, 0U, &status) || status != 0U ||
        (control1 & (1U << 3U)) != 0U || (control2 & (3U << 4U)) != 0U)
    {
        return false;
    }
    return true;
}
