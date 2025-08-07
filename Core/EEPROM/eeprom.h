#ifndef __EEPROM_H
#define __EEPROM_H

#include "main.h"
//#include "memorymap.h"

typedef enum {
    W25Qxx_OK = 0,
    W25Qxx_ERROR = -1
} W25Qxx_Result_t;

typedef struct {
    float kp;
    float ki;
    float kd;
} PID_t;

typedef struct {
    PID_t out;
    PID_t in;
} DualPID_t;

typedef struct {
    uint8_t rgb[4][3];
    uint8_t mode;
} AircraftLights_t;

typedef struct {
    float accel_cal[3];        // Accelerometer calibration biases (X, Y, Z)
    float gyro_cal[3];         // Gyroscope calibration biases (X, Y, Z)
    float mag_cal[3];          // Magnetometer calibration
    float pid[3];              // Legacy PID
    uint8_t flight_mode;       // Flight mode
    float gps_lat;             // GPS latitude
    float gps_lon;             // GPS longitude
    float gps_alt;             // GPS altitude
    DualPID_t roll_pid;        // Roll PID
    DualPID_t pitch_pid;       // Pitch PID
    PID_t yaw_rate_pid;        // Yaw rate PID
    AircraftLights_t lights;   // Aircraft lights
    uint32_t crc;              // CRC for data integrity
} DroneConfig_t;

W25Qxx_Result_t EEPROM_Init(void);
W25Qxx_Result_t EEPROM_ReadConfig(DroneConfig_t *config);
W25Qxx_Result_t EEPROM_WriteConfig(DroneConfig_t *config);
uint32_t CalculateCRC32(const uint8_t *data, uint32_t length);
W25Qxx_Result_t EEPROM_GetPID(float pid[3]);
W25Qxx_Result_t EEPROM_SetPID(const float pid[3]);
W25Qxx_Result_t EEPROM_GetFlightMode(uint8_t *flight_mode);
W25Qxx_Result_t EEPROM_SetFlightMode(uint8_t flight_mode);
W25Qxx_Result_t EEPROM_GetMagCalibration(float mag_cal[3]);
W25Qxx_Result_t EEPROM_SetMagCalibration(const float mag_cal[3]);
W25Qxx_Result_t EEPROM_GetGPS(float *lat, float *lon, float *alt);
W25Qxx_Result_t EEPROM_SetGPS(float lat, float lon, float alt);
W25Qxx_Result_t EEPROM_GetRollPID(DualPID_t *roll_pid);
W25Qxx_Result_t EEPROM_SetRollPID(const DualPID_t *roll_pid);
W25Qxx_Result_t EEPROM_GetPitchPID(DualPID_t *pitch_pid);
W25Qxx_Result_t EEPROM_SetPitchPID(const DualPID_t *pitch_pid);
W25Qxx_Result_t EEPROM_GetYawRatePID(PID_t *yaw_rate_pid);
W25Qxx_Result_t EEPROM_SetYawRatePID(const PID_t *yaw_rate_pid);
W25Qxx_Result_t EEPROM_GetAircraftLights(AircraftLights_t *lights);
W25Qxx_Result_t EEPROM_SetAircraftLights(const AircraftLights_t *lights);
W25Qxx_Result_t EEPROM_GetAccelCalibration(float accel_cal[3]);
W25Qxx_Result_t EEPROM_SetAccelCalibration(const float accel_cal[3]);
W25Qxx_Result_t EEPROM_GetGyroCalibration(float gyro_cal[3]);
W25Qxx_Result_t EEPROM_SetGyroCalibration(const float gyro_cal[3]);

#endif /* __EEPROM_H */
