/*
 * lps22df_platform.c
 *
 *  Created on: Jan 17, 2026
 *      Author: franu
 */
#include "lps22df_platform.h"
#include "stm32f7xx_hal.h"
#include "spi.h"
#include "gpio.h"
#include "lps22df_reg.h"

extern SPI_HandleTypeDef hspi2;  // Twój handle SPI
#define LPS22DF_CS_GPIO_Port BARO_CS_GPIO_Port
#define LPS22DF_CS_Pin       BARO_CS_Pin

int32_t lps22df_platform_write(void *handle, uint8_t reg,
                               const uint8_t *bufp, uint16_t len)
{
    reg &= 0x7F; // write
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, (uint8_t*)bufp, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    return 0;
}

//int32_t lps22df_platform_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
//{
//    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
//
//    if(data == NULL || len == 0)
//        return -1;
//
//    uint8_t tx = reg & 0x7F;        // MSB=0 -> write
//    if(len > 1)
//        tx |= 0x40;                 // Bit6 = auto-increment dla multi-byte
//
//    // CS LOW
//    HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_RESET);
//
//    // Wyślij rejestr do zapisu
//    if(HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY) != HAL_OK)
//    {
//        HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_SET);
//        return -1;
//    }
//
//    // Wyślij dane
//    if(HAL_SPI_Transmit(hspi, (uint8_t *)data, len, HAL_MAX_DELAY) != HAL_OK)
//    {
//        HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_SET);
//        return -1;
//    }
//
//    // CS HIGH
//    HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_SET);
//
//    return 0;
//}
//

int32_t lps22df_platform_read(void *handle, uint8_t reg,
                              uint8_t *bufp, uint16_t len)
{
    reg |= 0x80; // read
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, bufp, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    return 0;
}

//int32_t lps22df_platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
//{
//    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
//
//    if(data == NULL || len == 0)
//        return -1;
//
//    uint8_t tx = reg | 0x80;        // MSB=1 -> read
//    if(len > 1)
//        tx |= 0x40;                 // Bit6 = auto-increment dla multi-byte
//
//    // CS LOW
//    HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_RESET);
//
//    // Wyślij rejestr do odczytu
//    if(HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY) != HAL_OK)
//    {
//        HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_SET);
//        return -1;
//    }
//
//    // Odczyt danych
//    if(HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY) != HAL_OK)
//    {
//        HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_SET);
//        return -1;
//    }
//
//    // CS HIGH
//    HAL_GPIO_WritePin(LPS22DF_CS_GPIO_Port, LPS22DF_CS_Pin, GPIO_PIN_SET);
//
//    return 0;
//}


int32_t lps22df_init(stmdev_ctx_t *dev_ctx)
{
    lps22df_id_t whoami;
    lps22df_bus_mode_t bus_mode;
    lps22df_md_t md;

    /* 1. Sprawdzenie WHO_AM_I */
    if (lps22df_id_get(dev_ctx, &whoami) != 0)
        return -1;

    if (whoami.whoami != LPS22DF_ID)
        return -2;

    /* 2. Reset urządzenia */
    lps22df_init_set(dev_ctx, LPS22DF_RESET);

    /* czekaj aż reset się skończy */
    lps22df_stat_t stat;
    do {
        lps22df_status_get(dev_ctx, &stat);
    } while (stat.sw_reset);

    /* 3. Konfiguracja magistrali */
    bus_mode.interface = LPS22DF_SPI_4W;
    bus_mode.filter = LPS22DF_FILTER_ALWAYS_ON;
    bus_mode.i3c_ibi_time = LPS22DF_IBI_50us;
    lps22df_bus_mode_set(dev_ctx, &bus_mode);

    /* 4. Konfiguracja trybu pracy */
    md.odr = LPS22DF_50Hz;          // 50 Hz
    md.avg = LPS22DF_32_AVG;        // uśrednianie
    md.lpf = LPS22DF_LPF_ODR_DIV_4; // filtr LPF
    lps22df_mode_set(dev_ctx, &md);

    return 0;
}

void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}
