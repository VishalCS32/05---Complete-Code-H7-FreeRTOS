/*
 * buzzer.h
 *
 *  Created on: Jun 1, 2025
 *      Author: vishal
 */

/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   Buzzer control functions for STM32
  ******************************************************************************
  */

#ifndef __BUZZER_H
#define __BUZZER_H

/* Includes ------------------------------------------------------------------*/
#include "targets.h"

/* Function Prototypes -------------------------------------------------------*/
void Buzzer_PlayTone(uint16_t *prescalers, uint16_t *delays, uint8_t numTones);
void Buzzer_On(uint16_t prescaler);
void Buzzer_Off(void);
void StartupTone(void); // Added prototype for StartupTone

#endif /* __BUZZER_H */
