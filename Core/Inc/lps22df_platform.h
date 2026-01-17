/*
 * lps22df_platform.h
 *
 *  Created on: Jan 17, 2026
 *      Author: franu
 */

#ifndef LPS22DF_PLATFORM_H_
#define LPS22DF_PLATFORM_H_

#endif /* LPS22DF_PLATFORM_H_ */
#include "stm32f7xx_hal.h"
#include "lps22df_reg.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} lps22df_spi_ctx_t;


int32_t lps22df_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

int32_t lps22df_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

int32_t lps22df_init(stmdev_ctx_t *dev_ctx);

void platform_delay(uint32_t ms);
