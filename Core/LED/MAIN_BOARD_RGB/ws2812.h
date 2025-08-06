/*
 * ws2812.h
 *
 *  Created on: May 18, 2025
 *      Author: vishal
 */

#ifndef WS2812_H
#define WS2812_H

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define LED_COUNT 1
#define BITS_PER_LED 24
#define BUFFER_SIZE (LED_COUNT * BITS_PER_LED + 50)
#define DUTY_0 80
#define DUTY_1 160
#define DUTY_RESET 0

extern SemaphoreHandle_t ws2812_dma_semaphore;

void WS2812_Init(TIM_HandleTypeDef *htim);
void WS2812_SetColor(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness);
void WS2812_SetGlobalBrightness(float brightness);
void WS2812_Send(void);
void main_led(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness, uint32_t main_led_time);

#endif
