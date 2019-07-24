/*-----------------------------------------------------------------------------/
 * Module       : board.h
 * Create       : 2019-05-23
 * Copyright    : hamelec.taobao.com
 * Author       : huanglong
 * Brief        : 
/-----------------------------------------------------------------------------*/
#ifndef _BOARD_H
#define _BOARD_H

/* 公用 */
#include "stm32f1xx_hal.h"
#include "main.h"

#define LED1_ON         HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
#define LED1_OFF        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
#define LED1_TOG        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

#define AIC_RESET_H     HAL_GPIO_WritePin(AIC_RST_GPIO_Port, AIC_RST_Pin, GPIO_PIN_SET);
#define AIC_RESET_L     HAL_GPIO_WritePin(AIC_RST_GPIO_Port, AIC_RST_Pin, GPIO_PIN_RESET);

#define SDIO_CD_IN()    HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin)

#define LCD_RST_H()     HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
#define LCD_RST_L()     HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

#define CTP_RST_H()     // HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_SET);
#define CTP_RST_L()     // HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_RESET);
#define CTP_INT_IN()    HAL_GPIO_ReadPin(TP_IRQ_GPIO_Port, TP_IRQ_Pin)

#define RTP_CS_L()      HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
#define RTP_CS_H()      HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);

#define FLASH_CS_L()    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_RESET);
#define FLASH_CS_H()    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_SET);

#endif
