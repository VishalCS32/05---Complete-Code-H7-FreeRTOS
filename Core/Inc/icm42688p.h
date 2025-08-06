#ifndef ICM42688P_H
#define ICM42688P_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ICM42688P_SPI_READ   0x80
#define ICM42688P_SPI_WRITE  0x00

// Register Addresses
#define ICM42688P_REG_BANK_SEL     0x76
#define ICM42688P_WHO_AM_I         0x75
#define ICM42688P_DEVICE_CONFIG    0x11
#define ICM42688P_PWR_MGMT0        0x4E
#define ICM42688P_TEMP_DATA1       0x1D
#define ICM42688P_ACCEL_DATA_X1    0x1F
#define ICM42688P_GYRO_DATA_X1     0x25
#define ICM42688P_ACCEL_CONFIG0    0x50
#define ICM42688P_GYRO_CONFIG0     0x4F
#define ICM42688P_ACCEL_CONFIG1    0x53
#define ICM42688P_GYRO_CONFIG1     0x51

// Define your Chip Select GPIO here
#define ICM42688P_CS_PORT    GPIOA      // <-- Change to actual port
#define ICM42688P_CS_PIN     GPIO_PIN_15  // <-- Change to actual CS pin


#define ICM42688P_DEVICE_ID        0x47

typedef struct {
    SPI_HandleTypeDef *hspi;
    float accel_scale;
    float gyro_scale;
} ICM42688P_HandleTypeDef;

typedef struct {
    float accel[3];
    float gyro[3];
    float temperature;
} ICM42688P_Data_t;

void ICM42688P_Init(ICM42688P_HandleTypeDef *dev, SPI_HandleTypeDef *hspi);
void ICM42688P_ReadData(ICM42688P_HandleTypeDef *dev, ICM42688P_Data_t *data);
void ICM42688P_DeviceReset(ICM42688P_HandleTypeDef *dev);

#endif
