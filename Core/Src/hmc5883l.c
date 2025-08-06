/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hmc5883l.c
  * @brief   HMC5883L Magnetometer Driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Your Name.
  * Licensed under your chosen license.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "hmc5883l.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart6;
static uint8_t hmc5883l_raw_data[6]; // DMA buffer for 6 bytes
static HMC5883L_Data_t *hmc5883l_data_ptr; // Pointer for callback
static volatile uint8_t dma_data_ready = 0; // DMA completion flag
static volatile uint8_t dma_error = 0; // DMA error flag
HMC5883L_Data_t hmc_data;
static HMC5883L_Data_t mag_offsets = {-0.09f, -0.08f, 0.22f}; // Default offsets
static Compass_Data_t compass_data = {0.0f, 0.0f}; // Heading and declination
static volatile uint8_t i2c_error = 0;
static uint32_t last_reset_time = 0;
static uint8_t reset_attempts = 0;
static const uint8_t max_reset_attempts = 5;

static void HMC5883L_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = {reg, value};
    HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_ADDR, tx_data, 2, 100);
}

uint8_t HMC5883L_ReadReg(uint8_t reg) {
    uint8_t rx_data;
    HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_ADDR, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, HMC5883L_ADDR, &rx_data, 1, 100);
    return rx_data;
}

void HMC5883L_Init(void) {
    HMC5883L_WriteReg(HMC5883L_CONFIG_A, 0x80); // Soft reset
    HAL_Delay(100);
    HMC5883L_WriteReg(HMC5883L_CONFIG_A, 0x78); // 8-avg, 75 Hz, normal mode
    HMC5883L_WriteReg(HMC5883L_CONFIG_B, 0x20); // ±1.3 Gauss
    HMC5883L_WriteReg(HMC5883L_MODE, 0x00);     // Continuous mode
    HAL_Delay(10);
}

void HMC5883L_ReadMag(HMC5883L_Data_t *data) {
    uint8_t raw_data[6];
    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, HMC5883L_DATA_X_H, 1, raw_data, 6, 100);
    data->mag_x = (int16_t)(raw_data[0] << 8 | raw_data[1]) / 1090.0f;
    data->mag_z = (int16_t)(raw_data[2] << 8 | raw_data[3]) / 1090.0f;
    data->mag_y = (int16_t)(raw_data[4] << 8 | raw_data[5]) / 1090.0f;
}

void HMC5883L_ReadMag_DMA(HMC5883L_Data_t *data) {
    hmc5883l_data_ptr = data;
    dma_data_ready = 0;
    dma_error = 0;
    char msg[50];
    HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(&hi2c1);
    if (i2c_state == HAL_I2C_STATE_READY) {
        if (HAL_I2C_Mem_Read_DMA(&hi2c1, HMC5883L_ADDR, HMC5883L_DATA_X_H, 1, hmc5883l_raw_data, 6) != HAL_OK) {
            dma_error = 1;
            printf("DMA Start Failed, Error: %lu\n", HAL_I2C_GetError(&hi2c1));
//            HAL_UART_Transmit(&huart6, (uint8_t *)msg, strlen(msg), 100);
        }
    } else {
        dma_error = 1;
        printf("I2C Not Ready, State: %d\n", i2c_state);
//        HAL_UART_Transmit(&huart6, (uint8_t *)msg, strlen(msg), 100);
        // Full I2C reset
        __HAL_I2C_DISABLE(&hi2c1);
        HAL_I2C_Master_Abort_IT(&hi2c1, HMC5883L_ADDR);
        HAL_Delay(10);
        HAL_I2C_MspDeInit(&hi2c1);
        HAL_I2C_MspInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        i2c_error = 1; // Trigger error handling in ProcessMagData
    }
}

void HMC5883L_DMA_Complete_Callback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1 && !dma_error) {
        hmc5883l_data_ptr->mag_x = (int16_t)(hmc5883l_raw_data[0] << 8 | hmc5883l_raw_data[1]) / 1090.0f;
        hmc5883l_data_ptr->mag_z = (int16_t)(hmc5883l_raw_data[2] << 8 | hmc5883l_raw_data[3]) / 1090.0f;
        hmc5883l_data_ptr->mag_y = (int16_t)(hmc5883l_raw_data[4] << 8 | hmc5883l_raw_data[5]) / 1090.0f;
        dma_data_ready = 1;
    }
}

void HMC5883L_SetOffsets(HMC5883L_Data_t *offsets) {
    mag_offsets.mag_x = offsets->mag_x;
    mag_offsets.mag_y = offsets->mag_y;
    mag_offsets.mag_z = offsets->mag_z;
}

Compass_Data_t* HMC5883L_GetCompassData(void) {
    return &compass_data;
}

static float ComputeHeading(HMC5883L_Data_t *mag_data, float declination) {
    float heading = atan2f(-mag_data->mag_y, mag_data->mag_x) * RAD_TO_DEG;
    heading += declination;
    if (heading < 0.0f) heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;
    return heading;
}


void HMC5883L_ProcessMagData(void) {
    char msg[150];
    if (!dma_data_ready && !i2c_error && HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
        HMC5883L_ReadMag_DMA(&hmc_data);
    }
    if (dma_data_ready) {
        dma_data_ready = 0;
        reset_attempts = 0; // Reset attempts on successful read
        hmc_data.mag_x -= mag_offsets.mag_x;
        hmc_data.mag_y -= mag_offsets.mag_y;
        hmc_data.mag_z -= mag_offsets.mag_z;
        compass_data.heading = ComputeHeading(&hmc_data, compass_data.declination);
//        snprintf(msg, sizeof(msg), "Raw MX: %.2f, MY: %.2f, MZ: %.2f, Heading: %.2f deg\n",
//                 hmc_data.mag_x, hmc_data.mag_y, hmc_data.mag_z, compass_data.heading);
    }
    if (i2c_error && reset_attempts < max_reset_attempts) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_reset_time >= 1000) { // Wait 1s before retry
            printf("I2C Error, resetting (%d/%d)...\n",
                     reset_attempts + 1, max_reset_attempts);
//            HAL_UART_Transmit(&huart6, (uint8_t *)msg, strlen(msg), 100);
            HAL_Delay(10);
            HAL_I2C_MspDeInit(&hi2c1);
            HAL_I2C_MspInit(&hi2c1);
            HAL_I2C_Init(&hi2c1);
            HMC5883L_Init(); // Reinitialize sensor
            i2c_error = 0;
            reset_attempts++;
            last_reset_time = current_time;
        }
    } else if (i2c_error && reset_attempts >= max_reset_attempts) {
        printf("Max I2C reset attempts reached, halting...\n");
        while (1) {} // Halt execution
    }
}

HMC5883L_Data_t* HMC5883L_GetRawMagData(void) {
    return &hmc_data;
}

