/** @file board_critical.h
 *  @brief 板级共享数据的短临界区工具，不属于电机硬件接口。
 */
#ifndef BOARD_CRITICAL_H
#define BOARD_CRITICAL_H

#include "main.h"

/** @brief 保存 PRIMASK 后关闭中断；临界区内不得阻塞等待。 */
static inline uint32_t Board_EnterCritical(void)
{
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return saved_mask;
}
/** @brief 恢复原 PRIMASK，支持嵌套，不无条件开启中断。 */
static inline void Board_ExitCritical(uint32_t saved_mask)
{
    __DMB();
    __set_PRIMASK(saved_mask);
}

#endif
