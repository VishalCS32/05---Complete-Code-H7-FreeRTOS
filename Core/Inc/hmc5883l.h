/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hmc5883l.h
  * @brief   HMC5883L Magnetometer Driver Header
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Your Name.
  * Licensed under your chosen license.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "stm32h7xx_hal.h"

#define HMC5883L_ADDR        0x1E << 1
#define HMC5883L_ID_A        0x0A
#define HMC5883L_CONFIG_A    0x00
#define HMC5883L_CONFIG_B    0x01
#define HMC5883L_MODE        0x02
#define HMC5883L_DATA_X_H    0x03

typedef struct {
    float mag_x, mag_y, mag_z; // Gauss
} HMC5883L_Data_t;

typedef struct {
    float heading; // Heading in degrees
    float declination; // Magnetic declination in degrees
} Compass_Data_t;

void HMC5883L_Init(void);
void HMC5883L_ReadMag(HMC5883L_Data_t *data);
void HMC5883L_ReadMag_DMA(HMC5883L_Data_t *data);
void HMC5883L_ProcessMagData(void); // New function to handle all processing
void HMC5883L_DMA_Complete_Callback(I2C_HandleTypeDef *hi2c);
uint8_t HMC5883L_ReadReg(uint8_t reg);
void HMC5883L_SetOffsets(HMC5883L_Data_t *offsets);
Compass_Data_t* HMC5883L_GetCompassData(void);

HMC5883L_Data_t* HMC5883L_GetRawMagData(void);
extern HMC5883L_Data_t hmc_data;

#endif /* HMC5883L_H_ */
