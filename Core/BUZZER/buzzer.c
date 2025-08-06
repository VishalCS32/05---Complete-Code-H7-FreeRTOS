/*
 * buzzer.c
 *
 *  Created on: Jun 1, 2025
 *      Author: vishal
 */


/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   Buzzer control implementation
  ******************************************************************************
  */

#include "buzzer.h"
#include "tim.h" // For timer HAL/LL functions

/**
  * @brief  Play a sequence of tones on the buzzer
  * @param  prescalers: Array of prescaler values for the timer
  * @param  delays: Array of delay durations in milliseconds for each tone
  * @param  numTones: Number of tones to play
  * @retval None
  */
void Buzzer_PlayTone(uint16_t *prescalers, uint16_t *delays, uint8_t numTones)
{
    // Enable the timer and channel
    LL_TIM_EnableCounter(BUZZER_TIM);
    LL_TIM_CC_EnableChannel(BUZZER_TIM, BUZZER_TIM_CHANNEL);

    // Set duty cycle (e.g., 25% as in original code)
    BUZZER_TIM->CCR1 = BUZZER_TIM->ARR * BUZZER_DEFAULT_DUTY / 100;

    // Play each tone
    for (uint8_t i = 0; i < numTones; i++)
    {
        BUZZER_TIM->PSC = prescalers[i];
        HAL_Delay(delays[i]);
    }

    // Disable the timer to stop the buzzer
    LL_TIM_DisableCounter(BUZZER_TIM);
}

/**
  * @brief  Turn the buzzer on with a specific prescaler
  * @param  prescaler: Timer prescaler value
  * @retval None
  */
void Buzzer_On(uint16_t prescaler)
{
    BUZZER_TIM->PSC = prescaler;
    BUZZER_TIM->CCR1 = BUZZER_TIM->ARR * BUZZER_DEFAULT_DUTY / 100;
    LL_TIM_EnableCounter(BUZZER_TIM);
    LL_TIM_CC_EnableChannel(BUZZER_TIM, BUZZER_TIM_CHANNEL);
}

/**
  * @brief  Turn the buzzer off
  * @retval None
  */
void Buzzer_Off(void)
{
//    LL_TIM_DisableCounter(BUZZER_TIM);
    LL_TIM_CC_DisableChannel(BUZZER_TIM, BUZZER_TIM_CHANNEL);
}

/**
  * @brief  Play the startup tone sequence
  * @retval None
  */
void StartupTone(void)
{
    // Define the startup tone sequence
    uint16_t prescalers[] = {1092, 592, 292};
    uint16_t delays[] = {100, 100, 100};
    uint8_t numTones = 3;

    // Play the sequence using Buzzer_PlayTone
    Buzzer_PlayTone(prescalers, delays, numTones);
}
