#include "ibus.h"

/* ================= STATIC ================= */

static uint8_t uart_dma_rx_buf[IBUS_DMA_BUF_LEN];
static uint8_t ibus_frame_buf[IBUS_FRAME_LEN];

static uint16_t dma_last_pos = 0;
static uint8_t fail_safe_flag = 0;

/* strumień do parsera */
static uint8_t stream_buf[IBUS_FRAME_LEN * 2];
static uint16_t stream_len = 0;

/* ================= LOCAL ================= */

static void ibus_process_bytes(uint8_t *data, uint16_t len);
static void ibus_try_parse_frame(void);
static bool ibus_is_valid(void);
static bool ibus_checksum(void);
static void ibus_update(uint16_t *ibus_data);

/* ================= PUBLIC ================= */

void ibus_init(void)
{
    HAL_UART_Receive_DMA(IBUS_UART, uart_dma_rx_buf, IBUS_DMA_BUF_LEN);
}

/**
 * @brief Wywoływać w USARTx_IRQHandler po wykryciu UART_FLAG_IDLE
 */
void ibus_uart_idle_callback(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(IBUS_UART);

    uint16_t pos = IBUS_DMA_BUF_LEN -
                   __HAL_DMA_GET_COUNTER(IBUS_UART->hdmarx);

    if (pos != dma_last_pos)
    {
        /* cache fix dla STM32F7 */
        SCB_InvalidateDCache_by_Addr(
            (uint32_t*)uart_dma_rx_buf,
            IBUS_DMA_BUF_LEN
        );

        if (pos > dma_last_pos)
        {
            ibus_process_bytes(&uart_dma_rx_buf[dma_last_pos],
                               pos - dma_last_pos);
        }
        else
        {
            /* zawinięcie DMA */
            ibus_process_bytes(&uart_dma_rx_buf[dma_last_pos],
                               IBUS_DMA_BUF_LEN - dma_last_pos);
            ibus_process_bytes(&uart_dma_rx_buf[0], pos);
        }

        dma_last_pos = pos;
    }
}

bool ibus_read(uint16_t *ibus_data)
{
    if (!ibus_is_valid())
        return false;

    if (!ibus_checksum())
        return false;

    ibus_update(ibus_data);
    return true;
}

/* ================= FAIL SAFE ================= */

void ibus_soft_failsafe(uint16_t *ibus_data, uint8_t max)
{
    if (++fail_safe_flag < max)
        return;

    for (uint8_t i = 0; i < IBUS_USER_CHANNELS; i++)
        ibus_data[i] = 0;

    fail_safe_flag = 0;
}

void ibus_reset_failsafe(void)
{
    fail_safe_flag = 0;
}

/* ================= LOCAL ================= */

static void ibus_process_bytes(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (stream_len < sizeof(stream_buf))
            stream_buf[stream_len++] = data[i];

        ibus_try_parse_frame();
    }
}

/**
 * @brief Parser strumieniowy + RESYNC
 */
static void ibus_try_parse_frame(void)
{
    if (stream_len < 2)
        return;

    for (uint16_t i = 0; i <= stream_len - 2; i++)
    {
        if (stream_buf[i]     == IBUS_HEADER_0 &&
            stream_buf[i + 1] == IBUS_HEADER_1)
        {
            if (stream_len - i < IBUS_FRAME_LEN)
                return;

            memcpy(ibus_frame_buf,
                   &stream_buf[i],
                   IBUS_FRAME_LEN);

            memmove(stream_buf,
                    &stream_buf[i + IBUS_FRAME_LEN],
                    stream_len - (i + IBUS_FRAME_LEN));

            stream_len -= (i + IBUS_FRAME_LEN);

            ibus_reset_failsafe();
            return;
        }
    }

    /* brak nagłówka – zachowaj ostatni bajt */
    stream_buf[0] = stream_buf[stream_len - 1];
    stream_len = 1;
}

static bool ibus_is_valid(void)
{
    return (ibus_frame_buf[0] == IBUS_HEADER_0 &&
            ibus_frame_buf[1] == IBUS_HEADER_1);
}

static bool ibus_checksum(void)
{
    uint16_t checksum_cal = 0xFFFF;
    uint16_t checksum_ibus;

    for (uint8_t i = 0; i < 30; i++)
        checksum_cal -= ibus_frame_buf[i];

    checksum_ibus =
        (ibus_frame_buf[31] << 8) |
         ibus_frame_buf[30];

    return (checksum_ibus == checksum_cal);
}

static void ibus_update(uint16_t *ibus_data)
{
    for (uint8_t ch = 0, i = 2;
         ch < IBUS_USER_CHANNELS;
         ch++, i += 2)
    {
        ibus_data[ch] =
            (ibus_frame_buf[i + 1] << 8) |
             ibus_frame_buf[i];
    }
}
