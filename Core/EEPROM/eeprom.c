#include "eeprom.h"
#include "octospi.h"
#include <string.h>
#include <stdio.h>

/* W25Qxx Command Definitions */
#define W25Qxx_CMD_WRITE_ENABLE     0x06
#define W25Qxx_CMD_READ_DATA        0x03
#define W25Qxx_CMD_PAGE_PROGRAM     0x02
#define W25Qxx_CMD_SECTOR_ERASE     0x20
#define W25Qxx_CMD_READ_STATUS1     0x05
#define W25Qxx_CMD_JEDEC_ID         0x9F

/* W25Qxx Configuration */
#define W25Qxx_SECTOR_SIZE          4096
#define W25Qxx_PAGE_SIZE            256
#define W25Qxx_CONFIG_ADDRESS       0x000000 // Start address for config storage

/* Static function prototypes */
static W25Qxx_Result_t W25Qxx_WaitForWriteEnd(void);
static W25Qxx_Result_t W25Qxx_Read(uint32_t address, uint8_t *buffer, uint32_t length);
static W25Qxx_Result_t W25Qxx_Write(uint32_t address, uint8_t *buffer, uint32_t length);

/**
  * @brief Initialize the W25Qxx flash memory
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_Init(void) {
    uint8_t jedec_id[3];
    OSPI_RegularCmdTypeDef cmd = {0};

    /* Configure command to read JEDEC ID */
    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction = W25Qxx_CMD_JEDEC_ID;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = 3;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;

    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("Failed to configure JEDEC ID command\r\n");
        return W25Qxx_ERROR;
    }

    if (HAL_OSPI_Receive(&hospi1, jedec_id, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("Failed to read JEDEC ID\r\n");
        return W25Qxx_ERROR;
    }

    /* Check for valid JEDEC ID (example: Winbond W25Q series) */
    if (jedec_id[0] != 0xEF) { // Manufacturer ID for Winbond
        printf("Invalid JEDEC ID\r\n");
        return W25Qxx_ERROR;
    }

    return W25Qxx_OK;
}

/**
  * @brief Read configuration from EEPROM
  * @param config Pointer to DroneConfig_t structure
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_ReadConfig(DroneConfig_t *config) {
    uint8_t buffer[sizeof(DroneConfig_t)];
    if (W25Qxx_Read(W25Qxx_CONFIG_ADDRESS, buffer, sizeof(DroneConfig_t)) != W25Qxx_OK) {
        printf("Failed to read data from flash\r\n");
        return W25Qxx_ERROR;
    }

    memcpy(config, buffer, sizeof(DroneConfig_t));
    uint32_t calculated_crc = CalculateCRC32((uint8_t*)config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    if (calculated_crc != config->crc) {
        printf("EEPROM config CRC mismatch\r\n");
        return W25Qxx_ERROR;
    }

    return W25Qxx_OK;
}

/**
  * @brief Write configuration to EEPROM
  * @param config Pointer to DroneConfig_t structure
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_WriteConfig(DroneConfig_t *config) {
    uint8_t buffer[sizeof(DroneConfig_t)];
    memcpy(buffer, config, sizeof(DroneConfig_t));

    if (W25Qxx_Write(W25Qxx_CONFIG_ADDRESS, buffer, sizeof(DroneConfig_t)) != W25Qxx_OK) {
        printf("Failed to write data to flash\r\n");
        return W25Qxx_ERROR;
    }

    return W25Qxx_OK;
}

/**
  * @brief Calculate CRC32 for data integrity
  * @param data Pointer to data
  * @param length Length of data
  * @retval uint32_t CRC value
  */
uint32_t CalculateCRC32(const uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t polynomial = 0x04C11DB7;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
  * @brief Get accelerometer calibration data
  * @param accel_cal Array to store calibration values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetAccelCalibration(float accel_cal[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetAccelCalibration\r\n");
        accel_cal[0] = accel_cal[1] = accel_cal[2] = 0.0f; // Initialize on error
        return W25Qxx_ERROR;
    }
    memcpy(accel_cal, config.accel_cal, 3 * sizeof(float));
    printf("Retrieved accel_cal: X=%f, Y=%f, Z=%f\r\n", accel_cal[0], accel_cal[1], accel_cal[2]);
    return W25Qxx_OK;
}

/**
  * @brief Set accelerometer calibration data
  * @param accel_cal Array of calibration values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetAccelCalibration(const float accel_cal[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetAccelCalibration\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(config.accel_cal, accel_cal, 3 * sizeof(float));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get gyroscope calibration data
  * @param gyro_cal Array to store calibration values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetGyroCalibration(float gyro_cal[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetGyroCalibration\r\n");
        gyro_cal[0] = gyro_cal[1] = gyro_cal[2] = 0.0f; // Initialize on error
        return W25Qxx_ERROR;
    }
    memcpy(gyro_cal, config.gyro_cal, 3 * sizeof(float));
    printf("Retrieved gyro_cal: X=%f, Y=%f, Z=%f\r\n", gyro_cal[0], gyro_cal[1], gyro_cal[2]);
    return W25Qxx_OK;
}

/**
  * @brief Set gyroscope calibration data
  * @param gyro_cal Array of calibration values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetGyroCalibration(const float gyro_cal[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetGyroCalibration\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(config.gyro_cal, gyro_cal, 3 * sizeof(float));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get magnetometer calibration data
  * @param mag_cal Array to store calibration values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetMagCalibration(float mag_cal[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetMagCalibration\r\n");
        mag_cal[0] = mag_cal[1] = mag_cal[2] = 0.0f; // Initialize on error
        return W25Qxx_ERROR;
    }
    memcpy(mag_cal, config.mag_cal, 3 * sizeof(float));
    return W25Qxx_OK;
}

/**
  * @brief Set magnetometer calibration data
  * @param mag_cal Array of calibration values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetMagCalibration(const float mag_cal[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetMagCalibration\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(config.mag_cal, mag_cal, 3 * sizeof(float));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get legacy PID values
  * @param pid Array to store PID values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetPID(float pid[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetPID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(pid, config.pid, 3 * sizeof(float));
    return W25Qxx_OK;
}

/**
  * @brief Set legacy PID values
  * @param pid Array of PID values
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetPID(const float pid[3]) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetPID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(config.pid, pid, 3 * sizeof(float));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get flight mode
  * @param flight_mode Pointer to store flight mode
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetFlightMode(uint8_t *flight_mode) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetFlightMode\r\n");
        return W25Qxx_ERROR;
    }
    *flight_mode = config.flight_mode;
    return W25Qxx_OK;
}

/**
  * @brief Set flight mode
  * @param flight_mode Flight mode value
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetFlightMode(uint8_t flight_mode) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetFlightMode\r\n");
        return W25Qxx_ERROR;
    }
    config.flight_mode = flight_mode;
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get GPS data
  * @param lat Pointer to store latitude
  * @param lon Pointer to store longitude
  * @param alt Pointer to store altitude
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetGPS(float *lat, float *lon, float *alt) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetGPS\r\n");
        return W25Qxx_ERROR;
    }
    *lat = config.gps_lat;
    *lon = config.gps_lon;
    *alt = config.gps_alt;
    return W25Qxx_OK;
}

/**
  * @brief Set GPS data
  * @param lat Latitude
  * @param lon Longitude
  * @param alt Altitude
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetGPS(float lat, float lon, float alt) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetGPS\r\n");
        return W25Qxx_ERROR;
    }
    config.gps_lat = lat;
    config.gps_lon = lon;
    config.gps_alt = alt;
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get roll PID values
  * @param roll_pid Pointer to store roll PID
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetRollPID(DualPID_t *roll_pid) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetRollPID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(roll_pid, &config.roll_pid, sizeof(DualPID_t));
    return W25Qxx_OK;
}

/**
  * @brief Set roll PID values
  * @param roll_pid Pointer to roll PID
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetRollPID(const DualPID_t *roll_pid) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetRollPID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(&config.roll_pid, roll_pid, sizeof(DualPID_t));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get pitch PID values
  * @param pitch_pid Pointer to store pitch PID
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetPitchPID(DualPID_t *pitch_pid) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetPitchPID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(pitch_pid, &config.pitch_pid, sizeof(DualPID_t));
    return W25Qxx_OK;
}

/**
  * @brief Set pitch PID values
  * @param pitch_pid Pointer to pitch PID
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetPitchPID(const DualPID_t *pitch_pid) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetPitchPID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(&config.pitch_pid, pitch_pid, sizeof(DualPID_t));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get yaw rate PID values
  * @param yaw_rate_pid Pointer to store yaw rate PID
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetYawRatePID(PID_t *yaw_rate_pid) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetYawRatePID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(yaw_rate_pid, &config.yaw_rate_pid, sizeof(PID_t));
    return W25Qxx_OK;
}

/**
  * @brief Set yaw rate PID values
  * @param yaw_rate_pid Pointer to yaw rate PID
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetYawRatePID(const PID_t *yaw_rate_pid) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetYawRatePID\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(&config.yaw_rate_pid, yaw_rate_pid, sizeof(PID_t));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Get aircraft lights configuration
  * @param lights Pointer to store lights configuration
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_GetAircraftLights(AircraftLights_t *lights) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during GetAircraftLights\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(lights, &config.lights, sizeof(AircraftLights_t));
    return W25Qxx_OK;
}

/**
  * @brief Set aircraft lights configuration
  * @param lights Pointer to lights configuration
  * @retval W25Qxx_Result_t
  */
W25Qxx_Result_t EEPROM_SetAircraftLights(const AircraftLights_t *lights) {
    DroneConfig_t config;
    if (EEPROM_ReadConfig(&config) != W25Qxx_OK) {
        printf("Failed to read config during SetAircraftLights\r\n");
        return W25Qxx_ERROR;
    }
    memcpy(&config.lights, lights, sizeof(AircraftLights_t));
    config.crc = CalculateCRC32((uint8_t*)&config, sizeof(DroneConfig_t) - sizeof(uint32_t));
    return EEPROM_WriteConfig(&config);
}

/**
  * @brief Wait for write operation to complete
  * @retval W25Qxx_Result_t
  */
static W25Qxx_Result_t W25Qxx_WaitForWriteEnd(void) {
    OSPI_RegularCmdTypeDef cmd = {0};
    uint8_t status;

    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction = W25Qxx_CMD_READ_STATUS1;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = 1;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;

    do {
        if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("Failed to configure read status command\r\n");
            return W25Qxx_ERROR;
        }
        if (HAL_OSPI_Receive(&hospi1, &status, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("Failed to read status\r\n");
            return W25Qxx_ERROR;
        }
    } while (status & 0x01); // Busy bit

    return W25Qxx_OK;
}

/**
  * @brief Read data from W25Qxx flash
  * @param address Start address
  * @param buffer Buffer to store data
  * @param length Length of data to read
  * @retval W25Qxx_Result_t
  */
static W25Qxx_Result_t W25Qxx_Read(uint32_t address, uint8_t *buffer, uint32_t length) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction = W25Qxx_CMD_READ_DATA;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.Address = address;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = length;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;

    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("Failed to configure read command\r\n");
        return W25Qxx_ERROR;
    }

    if (HAL_OSPI_Receive(&hospi1, buffer, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("Failed to read data\r\n");
        return W25Qxx_ERROR;
    }

    return W25Qxx_OK;
}

/**
  * @brief Write data to W25Qxx flash
  * @param address Start address
  * @param buffer Pointer to data
  * @param length Length of data to write
  * @retval W25Qxx_Result_t
  */
static W25Qxx_Result_t W25Qxx_Write(uint32_t address, uint8_t *buffer, uint32_t length) {
    OSPI_RegularCmdTypeDef cmd = {0};
    uint32_t remaining = length;
    uint32_t current_address = address;
    uint8_t *current_buffer = buffer;

    /* Erase sector if necessary */
    uint32_t sector = address / W25Qxx_SECTOR_SIZE;
    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction = W25Qxx_CMD_WRITE_ENABLE;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = HAL_OSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;

    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("Failed to enable write for sector erase\r\n");
        return W25Qxx_ERROR;
    }

    cmd.Instruction = W25Qxx_CMD_SECTOR_ERASE;
    cmd.Address = sector * W25Qxx_SECTOR_SIZE;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;

    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("Failed to erase sector\r\n");
        return W25Qxx_ERROR;
    }

    if (W25Qxx_WaitForWriteEnd() != W25Qxx_OK) {
        printf("Timeout waiting for sector erase\r\n");
        return W25Qxx_ERROR;
    }

    /* Write data in pages */
    while (remaining > 0) {
        uint32_t page_size = (remaining > W25Qxx_PAGE_SIZE) ? W25Qxx_PAGE_SIZE : remaining;

        cmd.Instruction = W25Qxx_CMD_WRITE_ENABLE;
        cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
        cmd.DataMode = HAL_OSPI_DATA_NONE;

        if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("Failed to enable write for page program\r\n");
            return W25Qxx_ERROR;
        }

        cmd.Instruction = W25Qxx_CMD_PAGE_PROGRAM;
        cmd.Address = current_address;
        cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
        cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
        cmd.DataMode = HAL_OSPI_DATA_1_LINE;
        cmd.NbData = page_size;

        if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("Failed to configure page program command\r\n");
            return W25Qxx_ERROR;
        }

        if (HAL_OSPI_Transmit(&hospi1, current_buffer, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("Failed to write page data\r\n");
            return W25Qxx_ERROR;
        }

        if (W25Qxx_WaitForWriteEnd() != W25Qxx_OK) {
            printf("Timeout waiting for page write\r\n");
            return W25Qxx_ERROR;
        }

        remaining -= page_size;
        current_address += page_size;
        current_buffer += page_size;
    }

    return W25Qxx_OK;
}
