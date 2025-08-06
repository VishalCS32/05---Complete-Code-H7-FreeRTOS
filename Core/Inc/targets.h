/*
 * target.h
 *
 *  Created on: Mar 14, 2025
 *      Author: vishal
 */


#ifndef TARGET_H_
#define TARGET_H_

#include "stm32h7xx_hal.h"

/* Buzzer Configuration ------------------------------------------------------*/
#define BUZZER_TIM               TIM2
#define BUZZER_TIM_CHANNEL       LL_TIM_CHANNEL_CH1
#define BUZZER_DEFAULT_DUTY      25 // Percentage (25% duty cycle, i.e., TIM->ARR / 4)

// LED Pin Configuration
#define WS2812_LED_GPIO_PORT GPIOA
#define WS2812_LED_PIN GPIO_PIN_7 // PA7

#define AIRCRAFTLIGHTS_LED_GPIO_PORT GPIOB
#define AIRCRAFTLIGHTS_LED_PIN GPIO_PIN_0 // PA7

// Timer and DMA Configuration (for WS2812 PWM output)
#define WS2812_TIMER htim3
#define WS2812_DMA hdma_tim3_ch2
#define WS2812_TIMER_CHANNEL TIM_CHANNEL_2
#define WS2812_DMA_STREAM DMA1_Stream4
#define WS2812_DMA_IRQn DMA1_Stream4_IRQn

#define AIRCRAFTLIGHTS_TIMER htim3
#define AIRCRAFTLIGHTS_DMA AIRCRAFTLIGHTS_TIMER_DMA
#define AIRCRAFTLIGHTS_TIMER_DMA hdma_tim3_ch3
#define AIRCRAFTLIGHTS_TIMER_TIMER_CHANNEL TIM_CHANNEL_3
#define AIRCRAFTLIGHTS_TIMER_DMA_STREAM DMA1_Stream5
#define AIRCRAFTLIGHTS_TIMER_DMA_IRQn DMA1_Stream5_IRQn

#define SPI2_CS_Pin GPIO_PIN_8
#define SPI2_CS_GPIO_Port GPIOE

// I2C1 for HMC5883L
#define I2C1_SDA_PIN         GPIO_PIN_7
#define I2C1_SDA_PORT        GPIOB
#define I2C1_SCL_PIN         GPIO_PIN_6
#define I2C1_SCL_PORT        GPIOB

// USART6 for FTDI debugger
#define USART6_TX_PIN        GPIO_PIN_6
#define USART6_TX_PORT       GPIOC
#define USART6_RX_PIN        GPIO_PIN_7
#define USART6_RX_PORT       GPIOC

// Peripheral handles
//extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1; // Added for HMC5883L
//extern UART_HandleTypeDef huart6;

#endif /* TARGET_H_ */
