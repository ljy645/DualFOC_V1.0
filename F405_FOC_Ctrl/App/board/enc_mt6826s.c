/**
 * @file enc_mt6826s.c
 * @brief SPI1 DMA 读取双轴编码器，短临界区保护共享总线及结果缓存。
 */
#include "enc_mt6826s.h"
#include "board_critical.h"
#include "spi.h"

#define ENC_FRAME_SIZE 6U
#define ENC_TIMEOUT_MS 2U
#define ENC_RAD_PER_COUNT (6.28318530718f / 32768.0f)
#define ENC_SPEED_FILTER_HZ 20.0f

/** 异常后停在 FAULT，由上层显式初始化；DMA 未退出前不复用缓冲区。 */
typedef enum { ENC_OFF, ENC_IDLE, ENC_TRANSFER, ENC_FAULT } EncBusState;
/** 每轴最近一次请求的结果，错误帧不覆盖之前正确的角度数据。 */
typedef struct
{
    EncMt6826sSample sample;
    EncMt6826sResult result;
    EncMt6826sResult last_result;
    bool speed_reference_valid;
} EncCache;

static EncCache cache[MOTOR_AXIS_COUNT];
static uint8_t tx[ENC_FRAME_SIZE] = {0xA0U, 0x03U, 0U, 0U, 0U, 0U};
static uint8_t rx[ENC_FRAME_SIZE]; /* DMA 缓冲区位于常规 SRAM，生命周期覆盖异步传输。 */
static EncBusState bus_state;
static uint8_t pending_mask;
static unsigned int active_axis;
static uint32_t started_ms;
static uint32_t started_us;
static bool time_valid;

/** @brief 转速一阶低通，使用本帧实际间隔；调用方保证 dt_us 有效。 */
static float EncMt6826s_FilterSpeed(float previous, float speed, uint32_t dt_us)
{
    float dt = (float)dt_us * 1.0e-6f;
    float tau = 1.0f / (6.28318530718f * ENC_SPEED_FILTER_HZ);
    return previous + dt / (tau + dt) * (speed - previous);
}

/** @brief DMA RX/TX 中断次序不固定，必须分别确认两路状态。 */
static bool EncMt6826s_DmaReady(void)
{
    return HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_READY &&
           HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_READY;
}

/** @brief 操作独立片选，调用方保证 axis 有效。 */
static void EncMt6826s_SetCs(unsigned int axis, GPIO_PinState level)
{
    HAL_GPIO_WritePin(axis == MOTOR_AXIS_M0 ? M0_ENC_CS_GPIO_Port : M1_ENC_CS_GPIO_Port,
                      axis == MOTOR_AXIS_M0 ? M0_ENC_CS_Pin : M1_ENC_CS_Pin, level);
}

/** @brief 保留片选建立/保持的短延时，64 NOP 在 168 MHz 下至少约 381 ns。 */
static void EncMt6826s_CsDelay(void)
{
    unsigned int cycle;
    __DSB();
    for (cycle = 0U; cycle < 64U; ++cycle)
    {
        __NOP();
    }
}

/** @brief 校验原始数据三字节，CRC-8 多项式 0x07、初值 0、无反射。 */
static uint8_t EncMt6826s_CalculateCrc(const uint8_t *data)
{
    uint8_t crc = 0U;
    unsigned int byte;
    unsigned int bit;
    for (byte = 0U; byte < 3U; ++byte)
    {
        crc ^= data[byte];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/** @brief 异常时停止 SPI 时钟并异步中止 DMA，未退出的 DMA 不得访问下一帧缓冲区。 */
static void EncMt6826s_StopTransfer(EncMt6826sResult reason)
{
    unsigned int axis;
    uint8_t affected = (uint8_t)(pending_mask | (1U << active_axis));
    DMA_HandleTypeDef *dma[2] = {hspi1.hdmarx, hspi1.hdmatx};

    bus_state = ENC_FAULT;
    for (axis = 0; axis < MOTOR_AXIS_COUNT; ++axis)
    {
        if ((affected & (1U << axis)) != 0U)
        {
            cache[axis].result = reason;
            cache[axis].last_result = reason;
            cache[axis].speed_reference_valid = false;
        }
    }
    pending_mask = 0U;
    __HAL_SPI_DISABLE_IT(&hspi1, SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR);
    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    __HAL_SPI_DISABLE(&hspi1);
    EncMt6826s_CsDelay();
    EncMt6826s_SetCs(MOTOR_AXIS_M0, GPIO_PIN_SET);
    EncMt6826s_SetCs(MOTOR_AXIS_M1, GPIO_PIN_SET);
    for (axis = 0; axis < 2U; ++axis)
    {
        dma[axis]->XferAbortCallback = 0;
        if (HAL_DMA_GetState(dma[axis]) == HAL_DMA_STATE_BUSY)
        {
            (void)HAL_DMA_Abort_IT(dma[axis]);
        }
    }
}

/** @brief 在调用方临界区内启动待处理轴，仅提交 DMA，不等待收发结束。 */
static EncMt6826sResult EncMt6826s_StartNext(void)
{
    active_axis = (pending_mask & 1U) ? MOTOR_AXIS_M0 : MOTOR_AXIS_M1;
    pending_mask &= (uint8_t)~(1U << active_axis);
    bus_state = ENC_TRANSFER;
    started_ms = HAL_GetTick();
    EncMt6826s_SetCs(active_axis, GPIO_PIN_RESET);
    EncMt6826s_CsDelay();
    time_valid = (TIM5->CR1 & TIM_CR1_CEN) != 0U;
    started_us = time_valid ? TIM5->CNT : 0U;
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, ENC_FRAME_SIZE) != HAL_OK)
    {
        EncMt6826s_StopTransfer(ENC_MT6826S_SPI_ERROR);
        return ENC_MT6826S_SPI_ERROR;
    }
    return ENC_MT6826S_PENDING;
}

/** @brief 初始化协议缓存，不中断已提交请求或仍在退出的 DMA。 */
EncMt6826sResult EncMt6826s_Init(void)
{
    unsigned int axis;
    uint32_t saved_mask = Board_EnterCritical();
    EncMt6826sResult result = ENC_MT6826S_NOT_READY;

    if (bus_state == ENC_TRANSFER || pending_mask != 0U || !EncMt6826s_DmaReady())
    {
        result = ENC_MT6826S_BUSY;
        goto done;
    }
    /* 仅上层主动初始化时复位 SPI；到这里两路 DMA 均已退出。 */
    if (bus_state == ENC_FAULT &&
        (HAL_SPI_Abort(&hspi1) != HAL_OK || HAL_SPI_Init(&hspi1) != HAL_OK))
    {
        goto done;
    }
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
    {
        EncMt6826s_SetCs(MOTOR_AXIS_M0, GPIO_PIN_SET);
        EncMt6826s_SetCs(MOTOR_AXIS_M1, GPIO_PIN_SET);
        for (axis = 0; axis < MOTOR_AXIS_COUNT; ++axis)
        {
            cache[axis].result = ENC_MT6826S_NOT_READY;
            cache[axis].last_result = ENC_MT6826S_NOT_READY;
            cache[axis].speed_reference_valid = false;
        }
        pending_mask = 0U;
        bus_state = ENC_IDLE;
        result = ENC_MT6826S_OK;
    }
done:
    Board_ExitCritical(saved_mask);
    return result;
}

/** @brief 原子提交单轴或双轴请求，已有请求时不排入额外队列。 */
EncMt6826sResult EncMt6826s_RequestMask(uint8_t mask)
{
    unsigned int axis;
    uint32_t saved_mask;
    EncMt6826sResult result;
    if (mask == 0U || (mask & ~3U) != 0U) { return ENC_MT6826S_INVALID_ARG; }
    saved_mask = Board_EnterCritical();

    if (bus_state == ENC_OFF || bus_state == ENC_FAULT)
    {
        result = ENC_MT6826S_NOT_READY;
    }
    else if (bus_state != ENC_IDLE || pending_mask != 0U || !EncMt6826s_DmaReady())
    {
        result = ENC_MT6826S_BUSY;
    }
    else
    {
        pending_mask = mask;
        for (axis = 0; axis < MOTOR_AXIS_COUNT; ++axis)
        {
            if ((mask & (1U << axis)) != 0U)
            {
                cache[axis].result = ENC_MT6826S_PENDING;
            }
        }
        result = EncMt6826s_StartNext();
    }
    Board_ExitCritical(saved_mask);
    return result;
}

/** @brief 读取缓存，挂起请求及错误帧不会修改调用者原有样本。 */
static EncMt6826sResult EncMt6826s_ReadCache(unsigned int axis, EncMt6826sSample *sample, bool latest)
{
    uint32_t saved_mask;
    EncMt6826sResult result;
    if (axis >= MOTOR_AXIS_COUNT || sample == 0)
    {
        return ENC_MT6826S_INVALID_ARG;
    }
    saved_mask = Board_EnterCritical();
    if (bus_state == ENC_OFF) { result = ENC_MT6826S_NOT_READY; }
    else if (latest && bus_state == ENC_FAULT) { result = cache[active_axis].result; }
    else { result = latest ? cache[axis].last_result : cache[axis].result; }
    if (result == ENC_MT6826S_OK || result == ENC_MT6826S_WARNING)
    {
        *sample = cache[axis].sample;
    }
    Board_ExitCritical(saved_mask);
    return result;
}

EncMt6826sResult EncMt6826s_Read(unsigned int axis, EncMt6826sSample *sample)
{
    return EncMt6826s_ReadCache(axis, sample, false);
}

EncMt6826sResult EncMt6826s_ReadLatest(unsigned int axis, EncMt6826sSample *sample)
{
    return EncMt6826s_ReadCache(axis, sample, true);
}

/** @brief 仅检查超时、推进正常的第二轴请求；故障时不自动恢复或重试。 */
void EncMt6826s_Service(void)
{
    uint32_t saved_mask = Board_EnterCritical();
    if (bus_state == ENC_TRANSFER && (uint32_t)(HAL_GetTick() - started_ms) >= ENC_TIMEOUT_MS)
    {
        EncMt6826s_StopTransfer(ENC_MT6826S_TIMEOUT);
    }
    else if (bus_state == ENC_IDLE && pending_mask != 0U && EncMt6826s_DmaReady())
    {
        (void)EncMt6826s_StartNext();
    }
    Board_ExitCritical(saved_mask);
}

/** @brief SPI1 RX DMA 完成后结束片选并校验帧；SPI2 不属于本驱动。 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint32_t saved_mask = Board_EnterCritical();
    if (hspi == &hspi1 && bus_state == ENC_TRANSFER)
    {
        EncMt6826s_CsDelay();
        EncMt6826s_SetCs(active_axis, GPIO_PIN_SET);
        if (EncMt6826s_CalculateCrc(&rx[2]) != rx[5])
        {
            /* 一帧数据错误只影响本轴；总线和 DMA 已正常完成，继续服务另一轴。 */
            cache[active_axis].result = ENC_MT6826S_CRC_ERROR;
            cache[active_axis].last_result = ENC_MT6826S_CRC_ERROR;
            cache[active_axis].speed_reference_valid = false;
            bus_state = ENC_IDLE;
        }
        else
        {
            EncCache *encoder = &cache[active_axis];
            uint16_t angle = (uint16_t)(((uint16_t)rx[2] << 7) | (rx[3] >> 1));
            uint8_t status = rx[4] & 0x07U;
            uint32_t dt_us = started_us - encoder->sample.time_us;

            encoder->sample.speed_valid = encoder->speed_reference_valid && time_valid && status == 0U &&
                                          dt_us > 0U && dt_us < ENC_TIMEOUT_MS * 1000U;
            if (encoder->sample.speed_valid)
            {
                /* 直接对 15 位计数求最短差，等价于将角差折回 [-pi, pi]。 */
                int32_t delta = (int32_t)angle - encoder->sample.angle_raw;
                float speed;
                if (delta > 16384) { delta -= 32768; }
                else if (delta < -16384) { delta += 32768; }
                speed = (float)delta * ENC_RAD_PER_COUNT * 1000000.0f / (float)dt_us;
                encoder->sample.speed_rad_s = EncMt6826s_FilterSpeed(encoder->sample.speed_rad_s, speed, dt_us);
            }
            else
            {
                /* 首帧或数据间断时复位滤波值，重新建立速度基准。 */
                encoder->sample.speed_rad_s = 0.0f;
            }
            encoder->sample.angle_raw = angle;
            encoder->sample.angle_rad = angle * ENC_RAD_PER_COUNT;
            encoder->sample.status = status;
            encoder->sample.time_us = started_us;
            encoder->speed_reference_valid = time_valid && status == 0U;
            encoder->result = status ? ENC_MT6826S_WARNING : ENC_MT6826S_OK;
            encoder->last_result = encoder->result;
            bus_state = ENC_IDLE;
        }
    }
    Board_ExitCritical(saved_mask);
}

/** @brief SPI/DMA 失败后取消本批请求并释放片选，等待异步 DMA 退出。 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    uint32_t saved_mask = Board_EnterCritical();
    if (hspi == &hspi1 && bus_state == ENC_TRANSFER)
    {
        EncMt6826s_StopTransfer(ENC_MT6826S_SPI_ERROR);
    }
    Board_ExitCritical(saved_mask);
}
