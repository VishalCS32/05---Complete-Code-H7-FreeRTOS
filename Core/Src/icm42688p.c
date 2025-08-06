#include "icm42688p.h"
#include "cmsis_os.h"
#include "semphr.h"
#include <stdio.h>

// ====== FreeRTOS Semaphore for DMA ======
extern SemaphoreHandle_t spiDmaSem;   // declared in freertos.c

// ====== LOW LEVEL SPI FUNCTIONS ======
static void CS_Low(void) {
    HAL_GPIO_WritePin(ICM42688P_CS_PORT, ICM42688P_CS_PIN, GPIO_PIN_RESET);
}
static void CS_High(void) {
    HAL_GPIO_WritePin(ICM42688P_CS_PORT, ICM42688P_CS_PIN, GPIO_PIN_SET);
}

// ====== DMA Callback ======
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) {    // check SPI3 DMA complete
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(spiDmaSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// ====== Single-Byte DMA Transfer with FreeRTOS Sync ======
static uint8_t SPI_RW(ICM42688P_HandleTypeDef *dev, uint8_t data) {
    uint8_t rx;
    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, &data, &rx, 1) != HAL_OK) {
        return 0xFF;
    }
    if (xSemaphoreTake(spiDmaSem, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0xFF; // Timeout
    }
    return rx;
}

// ====== Multi-Byte DMA Read with FreeRTOS Sync ======
static void ReadMulti(ICM42688P_HandleTypeDef *dev, uint8_t reg, uint8_t *buf, uint16_t len) {
    CS_Low();
    SPI_RW(dev, reg | ICM42688P_SPI_READ);

    // Dummy TX buffer for receiving data
    static uint8_t dummy_tx[256];
    if (len > sizeof(dummy_tx)) len = sizeof(dummy_tx);
    for (uint16_t i = 0; i < len; i++) dummy_tx[i] = 0xFF;

    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, dummy_tx, buf, len) != HAL_OK) {
        CS_High();
        return;
    }

    xSemaphoreTake(spiDmaSem, pdMS_TO_TICKS(10));
    CS_High();
}

// ====== Register Write ======
static void WriteReg(ICM42688P_HandleTypeDef *dev, uint8_t reg, uint8_t data) {
    CS_Low();
    SPI_RW(dev, reg | ICM42688P_SPI_WRITE);
    SPI_RW(dev, data);
    CS_High();
}

// ====== Register Read ======
static uint8_t ReadReg(ICM42688P_HandleTypeDef *dev, uint8_t reg) {
    CS_Low();
    SPI_RW(dev, reg | ICM42688P_SPI_READ);
    uint8_t val = SPI_RW(dev, 0xFF);
    CS_High();
    return val;
}

// ====== Bank Select ======
static void SelectBank(ICM42688P_HandleTypeDef *dev, uint8_t bank) {
    WriteReg(dev, ICM42688P_REG_BANK_SEL, (bank & 0x07) << 4);
}

// ====== Device Reset ======
void ICM42688P_DeviceReset(ICM42688P_HandleTypeDef *dev) {
    WriteReg(dev, ICM42688P_DEVICE_CONFIG, 0x01);
    HAL_Delay(50);
}

// ====== Initialization ======
void ICM42688P_Init(ICM42688P_HandleTypeDef *dev, SPI_HandleTypeDef *hspi) {
    dev->hspi = hspi;
    dev->accel_scale = 1.0f / 2048.0f;
    dev->gyro_scale  = 1.0f / 16.4f;

    HAL_Delay(50);
    ICM42688P_DeviceReset(dev);
    SelectBank(dev, 0);

    uint8_t whoami = ReadReg(dev, ICM42688P_WHO_AM_I);
    printf("WHO_AM_I=0x%02X\r\n", whoami);
    if (whoami != ICM42688P_DEVICE_ID) {
        printf("ICM42688P not found!\r\n");
        while (1);
    }

    WriteReg(dev, ICM42688P_PWR_MGMT0, 0x0F);
    HAL_Delay(10);
    uint8_t pwr = ReadReg(dev, ICM42688P_PWR_MGMT0);
    printf("PWR_MGMT0=0x%02X\r\n", pwr);

    WriteReg(dev, ICM42688P_GYRO_CONFIG0, 0x06);
    WriteReg(dev, ICM42688P_ACCEL_CONFIG0, 0x06);

    HAL_Delay(10);
    printf("ICM42688P Initialized Successfully\r\n");
}

// ====== Read Sensor Data ======
void ICM42688P_ReadData(ICM42688P_HandleTypeDef *dev, ICM42688P_Data_t *data) {
    uint8_t buf[14];
    ReadMulti(dev, ICM42688P_TEMP_DATA1, buf, 14);

    int16_t raw_temp  = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ax    = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_ay    = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_az    = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t raw_gx    = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy    = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz    = (int16_t)((buf[12] << 8) | buf[13]);

    data->temperature = ((float)raw_temp / 132.48f) + 25.0f;
    data->accel[0] = raw_ax * dev->accel_scale;
    data->accel[1] = raw_ay * dev->accel_scale;
    data->accel[2] = raw_az * dev->accel_scale;
    data->gyro[0]  = raw_gx * dev->gyro_scale;
    data->gyro[1]  = raw_gy * dev->gyro_scale;
    data->gyro[2]  = raw_gz * dev->gyro_scale;
}
