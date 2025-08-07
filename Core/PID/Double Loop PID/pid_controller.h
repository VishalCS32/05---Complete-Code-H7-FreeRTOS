#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "../EEPROM/eeprom.h" // For PID_t and DualPID_t definitions

// Runtime PID structure for calculations
typedef struct {
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float kd;          // Derivative gain
    float integral;    // Accumulated integral
    float last_error;  // Previous error for derivative
    float output_limit; // Output saturation limit
} RuntimePID_t;

// Runtime Dual PID structure for roll/pitch
typedef struct {
    RuntimePID_t out; // Attitude PID
    RuntimePID_t in;  // Rate PID
} RuntimeDualPID_t;

// Initialize PID structures with EEPROM values
void pid_init(RuntimeDualPID_t *roll_pid, RuntimeDualPID_t *pitch_pid, RuntimePID_t *yaw_rate_pid,
              const DualPID_t *eeprom_roll_pid, const DualPID_t *eeprom_pitch_pid, const PID_t *eeprom_yaw_rate_pid);

// Compute cascade PID for roll/pitch
void pid_cascade_compute(RuntimeDualPID_t *pid, float des_angle, float act_angle, float gyro_rate, float dt, float *output);

// Compute single PID for yaw rate
void pid_rate_compute(RuntimePID_t *pid, float des_rate, float act_rate, float dt, float *output);

// Motor mixing for quadcopter (X configuration)
void motor_mixing(float roll_out, float pitch_out, float yaw_out, float throttle, uint16_t *ccr);

#define PWM_MIN 1050  // Match your PWM range
#define PWM_MAX 2000

#endif // PID_CONTROLLER_H
