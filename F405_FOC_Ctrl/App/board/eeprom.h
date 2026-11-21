/** @file eeprom.h
 *  @brief I2C1 上的 24LC64 字节读写，A0/A1/A2/WP 均接 GND。
 */
#ifndef EEPROM_H
#define EEPROM_H

#include <stdbool.h>
#include <stdint.h>

#define EEPROM_SIZE_BYTES 8192U
#define EEPROM_PAGE_SIZE 32U

/* 调用约定：MX_I2C1_Init() 已完成，I2C1 独占，系统时基和中断正常。
 * 以下为阻塞接口，仅在前台初始化或两路电机均停止时调用，不放入控制中断。
 * 地址按字节计，范围 0x0000～0x1FFF；越界、空缓冲区或通信失败返回 false。
 * 长度为 0 时不访问总线。失败时读缓冲区或 EEPROM 可能已完成前缀部分。
 */
bool Eeprom_Read(uint16_t address, void *data, uint16_t size);

/** @brief 自动按 32 字节页边界拆分，逐页等待写完成后返回，不自动重写失败页。
 *  @note WP 必须保持低电平；true 表示通信及写周期等待成功，不包含读回校验。
 */
bool Eeprom_Write(uint16_t address, const void *data, uint16_t size);

typedef enum { EEPROM_PENDING, EEPROM_COMPLETE, EEPROM_ERROR, EEPROM_VERIFY_ERROR } EepromResult;
/* 一次写入并核对一个32字节对齐页。调用后反复 Poll，期间继续执行前台服务。
 * I2C1独占，回调由本驱动使用；失败不重试。此接口可在电机运行时使用。
 */
bool Eeprom_WritePageStart(uint16_t address, const uint8_t data[EEPROM_PAGE_SIZE]);
EepromResult Eeprom_WritePagePoll(void);

#endif
