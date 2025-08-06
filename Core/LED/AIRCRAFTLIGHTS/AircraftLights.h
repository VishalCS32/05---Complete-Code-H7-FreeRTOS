/*
 * AircraftLights.h
 *
 *  Created on: Aug 5, 2025
 *      Author: vishal
 */

#ifndef AIRCRAFTLIGHTS_H
#define AIRCRAFTLIGHTS_H

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define AIRCRAFT_LED_COUNT 4
#define AIRCRAFT_BITS_PER_LED 24
#define AIRCRAFT_BUFFER_SIZE (AIRCRAFT_LED_COUNT * AIRCRAFT_BITS_PER_LED + 50)
#define DUTY_0 80
#define DUTY_1 160
#define DUTY_RESET 0

extern SemaphoreHandle_t aircraftlights_dma_semaphore;

void AIRCRAFTLIGHTS_Init(TIM_HandleTypeDef *htim);
void AIRCRAFTLIGHTS_SetColor(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness);
void AIRCRAFTLIGHTS_SetGlobalBrightness(float brightness);
void AIRCRAFTLIGHTS_Send(void);
void aircraftlights(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness);
void aircraftlights_update(void);
void AircraftLights_PWM_Callback(TIM_HandleTypeDef *htim);

#endif

