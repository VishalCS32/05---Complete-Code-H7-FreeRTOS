#include "pid_controller.h"
#include <math.h>

// Initialize PID structures with EEPROM values
void pid_init(RuntimeDualPID_t *roll_pid, RuntimeDualPID_t *pitch_pid, RuntimePID_t *yaw_rate_pid,
              const DualPID_t *eeprom_roll_pid, const DualPID_t *eeprom_pitch_pid, const PID_t *eeprom_yaw_rate_pid) {
    // Roll PID
    roll_pid->out.kp = eeprom_roll_pid->out.kp;
    roll_pid->out.ki = eeprom_roll_pid->out.ki;
    roll_pid->out.kd = eeprom_roll_pid->out.kd;
    roll_pid->out.integral = 0.0f;
    roll_pid->out.last_error = 0.0f;
    roll_pid->out.output_limit = 200.0f; // Max angular rate (deg/s)

    roll_pid->in.kp = eeprom_roll_pid->in.kp;
    roll_pid->in.ki = eeprom_roll_pid->in.ki;
    roll_pid->in.kd = eeprom_roll_pid->in.kd;
    roll_pid->in.integral = 0.0f;
    roll_pid->in.last_error = 0.0f;
    roll_pid->in.output_limit = 950.0f; // Max PWM contribution

    // Pitch PID
    pitch_pid->out.kp = eeprom_pitch_pid->out.kp;
    pitch_pid->out.ki = eeprom_pitch_pid->out.ki;
    pitch_pid->out.kd = eeprom_pitch_pid->out.kd;
    pitch_pid->out.integral = 0.0f;
    pitch_pid->out.last_error = 0.0f;
    pitch_pid->out.output_limit = 200.0f;

    pitch_pid->in.kp = eeprom_pitch_pid->in.kp;
    pitch_pid->in.ki = eeprom_pitch_pid->in.ki;
    pitch_pid->in.kd = eeprom_pitch_pid->in.kd;
    pitch_pid->in.integral = 0.0f;
    pitch_pid->in.last_error = 0.0f;
    pitch_pid->in.output_limit = 950.0f;

    // Yaw rate PID
    yaw_rate_pid->kp = eeprom_yaw_rate_pid->kp;
    yaw_rate_pid->ki = eeprom_yaw_rate_pid->ki;
    yaw_rate_pid->kd = eeprom_yaw_rate_pid->kd;
    yaw_rate_pid->integral = 0.0f;
    yaw_rate_pid->last_error = 0.0f;
    yaw_rate_pid->output_limit = 950.0f;
}

// Compute PID for a single loop
static float pid_compute(RuntimePID_t *pid, float setpoint, float actual, float dt) {
    float error = setpoint - actual;
    pid->integral += error * dt;
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return fmaxf(-pid->output_limit, fminf(pid->output_limit, output));
}

// Compute cascade PID for roll/pitch
void pid_cascade_compute(RuntimeDualPID_t *pid, float des_angle, float act_angle, float gyro_rate, float dt, float *output) {
    // Outer loop: Attitude PID -> Desired rate
    float des_rate = pid_compute(&pid->out, des_angle, act_angle, dt);
    // Inner loop: Rate PID -> Motor command
    *output = pid_compute(&pid->in, des_rate, gyro_rate, dt);
}

// Compute single PID for yaw rate
void pid_rate_compute(RuntimePID_t *pid, float des_rate, float act_rate, float dt, float *output) {
    *output = pid_compute(pid, des_rate, act_rate, dt);
}

// Motor mixing for quadcopter (X configuration)
void motor_mixing(float roll_out, float pitch_out, float yaw_out, float throttle, uint16_t *ccr) {
    ccr[0] = throttle + pitch_out + roll_out - yaw_out; // Front-Left (M1)
    ccr[1] = throttle + pitch_out - roll_out + yaw_out; // Rear-Left (M2)
    ccr[2] = throttle - pitch_out + roll_out + yaw_out; // Rear-Right (M3)
    ccr[3] = throttle - pitch_out - roll_out - yaw_out; // Front-Right (M4)
    for (int i = 0; i < 4; i++) {
        ccr[i] = (ccr[i] > PWM_MAX) ? PWM_MAX : (ccr[i] < PWM_MIN) ? PWM_MIN : ccr[i];
    }
}
