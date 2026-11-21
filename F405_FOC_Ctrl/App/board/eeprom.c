/** @file eeprom.c
 *  @brief 24LC64 / STM32F4 HAL 移植：连续读、跨页写和 ACK 写完成轮询。
 *  @see https://github.com/patsaoglou/st-m24c64
 *  @see Microchip DS21189，固定为本板 I2C1 / 8 KB / 32 字节页。
 *
 * MIT License
 * Copyright (c) 2026 Patsaoglou Pantelis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "eeprom.h"
#include "i2c.h"
#include <stddef.h>
#include <string.h>

#define EEPROM_HAL_ADDRESS (0x50U << 1U)
#define EEPROM_TRANSFER_TIMEOUT_MS 1000U
#define EEPROM_WRITE_TIMEOUT_MS 6U /* 芯片写周期最大 5 ms，另留时基量化余量。 */

/** @brief 写周期中的 NACK 是芯片忙，不是重写请求；仅轮询地址应答。 */
static bool Eeprom_WaitWriteComplete(void)
{
    uint32_t started = HAL_GetTick();
    do
    {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_HAL_ADDRESS, 1U, 2U);
        if (status == HAL_OK) { return true; }
        /* 本工程 HAL 的地址 NACK 返回 HAL_ERROR，清除 AF 后 ErrorCode 保持 NONE。 */
        if (status != HAL_ERROR || HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) { return false; }
    }
    while ((uint32_t)(HAL_GetTick() - started) < EEPROM_WRITE_TIMEOUT_MS);
    return false;
}

bool Eeprom_Read(uint16_t address, void *data, uint16_t size)
{
    if (address > EEPROM_SIZE_BYTES || size > EEPROM_SIZE_BYTES - address ||
        (size != 0U && data == NULL)) { return false; }
    if (size == 0U) { return true; }
    return HAL_I2C_Mem_Read(&hi2c1, EEPROM_HAL_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                            (uint8_t *)data, size, EEPROM_TRANSFER_TIMEOUT_MS) == HAL_OK;
}

bool Eeprom_Write(uint16_t address, const void *data, uint16_t size)
{
    const uint8_t *source = (const uint8_t *)data;
    if (address > EEPROM_SIZE_BYTES || size > EEPROM_SIZE_BYTES - address ||
        (size != 0U && data == NULL)) { return false; }
    while (size != 0U)
    {
        uint16_t chunk = (uint16_t)(EEPROM_PAGE_SIZE - address % EEPROM_PAGE_SIZE);
        if (chunk > size) { chunk = size; }
        if (HAL_I2C_Mem_Write(&hi2c1, EEPROM_HAL_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                             (uint8_t *)source, chunk, EEPROM_TRANSFER_TIMEOUT_MS) != HAL_OK ||
            !Eeprom_WaitWriteComplete()) { return false; }
        address += chunk;
        source += chunk;
        size -= chunk;
    }
    return true;
}

/* 运行中只使用单页异步事务，缓冲区保持到中断传输真正结束。 */
static struct
{
    uint8_t data[EEPROM_PAGE_SIZE];
    uint8_t verify[EEPROM_PAGE_SIZE];
    uint16_t address;
    volatile enum { PAGE_IDLE, PAGE_TX, PAGE_WAIT, PAGE_RX, PAGE_DONE, PAGE_ERROR } phase;
    volatile uint32_t started;
} page;

bool Eeprom_WritePageStart(uint16_t address, const uint8_t data[EEPROM_PAGE_SIZE])
{
    if (address % EEPROM_PAGE_SIZE != 0U || address > EEPROM_SIZE_BYTES - EEPROM_PAGE_SIZE ||
        (page.phase != PAGE_IDLE && page.phase != PAGE_DONE && page.phase != PAGE_ERROR) ||
        HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ||
        __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET) { return false; }
    memcpy(page.data, data, sizeof(page.data));
    page.address = address;
    page.started = HAL_GetTick();
    page.phase = PAGE_TX;
    /* 已检查总线空闲，避免本工程 HAL _IT 入口内的 BUSY 轮询。 */
    if (HAL_I2C_Mem_Write_IT(&hi2c1, EEPROM_HAL_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                             page.data, sizeof(page.data)) != HAL_OK)
    {
        page.phase = PAGE_ERROR;
        return false;
    }
    return true;
}

EepromResult Eeprom_WritePagePoll(void)
{
    if ((page.phase == PAGE_TX || page.phase == PAGE_WAIT || page.phase == PAGE_RX) &&
        (uint32_t)(HAL_GetTick() - page.started) >= 25U) { page.phase = PAGE_ERROR; }
    if (page.phase == PAGE_WAIT && (uint32_t)(HAL_GetTick() - page.started) >= EEPROM_WRITE_TIMEOUT_MS &&
        HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY &&
        __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET)
    {
        page.started = HAL_GetTick();
        page.phase = PAGE_RX;
        if (HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_HAL_ADDRESS, page.address, I2C_MEMADD_SIZE_16BIT,
                              page.verify, sizeof(page.verify)) != HAL_OK) { page.phase = PAGE_ERROR; }
    }
    if (page.phase == PAGE_DONE)
    {
        return memcmp(page.data, page.verify, sizeof(page.data)) == 0 ? EEPROM_COMPLETE : EEPROM_VERIFY_ERROR;
    }
    return page.phase == PAGE_ERROR || page.phase == PAGE_IDLE ? EEPROM_ERROR : EEPROM_PENDING;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *i2c)
{
    if (i2c == &hi2c1 && page.phase == PAGE_TX)
    {
        page.started = HAL_GetTick();
        page.phase = PAGE_WAIT;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *i2c)
{
    if (i2c == &hi2c1 && page.phase == PAGE_RX) { page.phase = PAGE_DONE; }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c)
{
    if (i2c == &hi2c1 && (page.phase == PAGE_TX || page.phase == PAGE_WAIT || page.phase == PAGE_RX))
    {
        page.phase = PAGE_ERROR;
    }
}
