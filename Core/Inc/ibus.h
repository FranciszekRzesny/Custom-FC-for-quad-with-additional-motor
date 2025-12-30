#ifndef __IBUS_H
#define __IBUS_H

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <string.h>

/* ================= CONFIG ================= */

#define IBUS_FRAME_LEN        32
#define IBUS_HEADER_0         0x20
#define IBUS_HEADER_1         0x40
#define IBUS_DMA_BUF_LEN      64      // >= 2 * IBUS_FRAME_LEN
#define IBUS_USER_CHANNELS    10

/* UART handle z CubeMX */
extern UART_HandleTypeDef huart4;
#define IBUS_UART (&huart4)

/* ================= API ================= */

void ibus_init(void);

/* wywoływać w USARTx_IRQHandler po IDLE */
void ibus_uart_idle_callback(void);

bool ibus_read(uint16_t *ibus_data);

void ibus_soft_failsafe(uint16_t *ibus_data, uint8_t fail_safe_max);
void ibus_reset_failsafe(void);

#endif
