#include <Wire.h>
#include "LowPass.h"

// Flight Mode Enumeration and Initialization
enum FlightMode{
  FM_disabled,
  FM_mounting,
  FM_stable,
  FM_alt_hold,
  FM_loiter
};

FlightMode fm = FM_disabled;

// BMP280: Adresses
#define BMP280_ADDRESS 0x76                   
#define BMP280_REGISTER_PRESSURE_MSB 0xF7
#define BMP280_REGISTER_PRESSURE_LSB 0xF8
#define BMP280_REGISTER_PRESSURE_XLSB 0xF9
#define BMP280_REGISTER_TEMPERATURE_MSB 0xFA
#define BMP280_REGISTER_TEMPERATURE_LSB 0xFB
#define BMP280_REGISTER_TEMPERATURE_XLSB 0xFC

// BMP280: Compensation Parameters
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

int64_t var1, var2, p;
int32_t p_32;
int32_t var1_32, var2_32;
int32_t t_fine;

// BMP280: Temperature and pressure variables
int32_t temperature_msb, temperature_lsb, temperature_xlsb, adc_T, t_temp;
float T, P, P_filt;
int32_t pressure_msb, pressure_lsb, pressure_xlsb, adc_P;

// BMP280: Pressure signal smoothened
float pressure_total_avarage, pressure_rotating_mem[12], actual_pressure_fast, actual_pressure_slow = 101325, actual_pressure_diff, actual_pressure;
uint8_t pressure_rotating_mem_location, barometer_counter;

// BMP280: Parachute throttle: 
int32_t parachute_buffer[20], parachute_throttle;
float pressure_parachute_previous;
uint8_t parachute_rotating_mem_location;



// HC-SR04
#define triggerPin PC3  // Trigger Pin Ultrasonico
#define echoPin PC2     // Echo Pin Ultrasonico
float distance;
float velocity;
LowPass<2> lp(3, 1e3, true);
float distanceFilt;
float velocityFilt;
float velocity_raw;
float sentLastPulse, duration, pulseStart, pulseEnd, computedDistance;
bool pulseSent;
long cAir = 331.3 + 20.0 * 0.606;
unsigned long timeLast;
float prevDistance, prevDistanceFilt;

// LED
volatile long led_timer;

// BUZZER
float batteryVoltage;
int buzzerTimer;
bool toneOn;

// SPI Flash
#define pin_BUZZER PA6
#define length_str 20000
uint16_t voltage_str[length_str];
uint16_t throttle_str[length_str];
uint16_t n_str = 0;

// MOTORS
long tiempo_motores_start, tiempo_1, tiempo_2, tiempo_ON;

// FlightSky i6
#define pin_PPM PA4
#define numero_canales 8
uint64_t pulso_instante[numero_canales * 2 + 2];
uint16_t Mando_canal[numero_canales];
uint8_t contador_flaco = 1;
int16_t throttle;


// Battery
#define pin_BAT PA5
float battery_voltage;

// PID: Variables
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
// Check:
float pid_i_mem_altitude, pid_i_mem_altitude_v2, pid_altitude_input, pid_altitude_v2_input, pid_output_altitude, pid_output_altitude_v2, pid_last_altitude_d_error, pid_last_altitude_v2_d_error;
float pid_altitude_setpoint, pid_error_gain_altitude;
uint16_t last_alt_hold_PID;
float pid_t_control_error, pid_t_output, pid_rate_control_error, pid_i_mem_rate, pid_rate_error_prev, pid_output_rate;


// PID: Roll
float pid_p_gain_roll = 0.9;
float pid_i_gain_roll = 0.009;
float pid_d_gain_roll = 4;
int pid_max_roll = 400;

// PID: Pitch
float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = 400;

// PID: Yaw
float pid_p_gain_yaw = 0.75;
float pid_i_gain_yaw = 0.01;
float pid_d_gain_yaw = 0;
int pid_max_yaw = 400;

// PID: Altitude
float pid_p_gain_altitude = 1.4;
float pid_i_gain_altitude = 0.2; //0.2;
float pid_d_gain_altitude = 0.75; //0.75;
int pid_max_altitude = 30;

// MPU6050: Address, setup and callibration

#define MPU6050_ADDRESS 0x68                  
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_CONFIG 0x1A

boolean auto_level = true;
int16_t manual_acc_pitch_cal_value = 0;
int16_t manual_acc_roll_cal_value = 0;
uint8_t use_manual_calibration = false;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;
int16_t manual_x_cal_value = 0;
int16_t manual_y_cal_value = 0;
int16_t manual_z_cal_value = 0;
int16_t cal_int;
int16_t temperature;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int32_t acc_x_cal, acc_y_cal, acc_z_cal;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

// ESC
#define pin_motor1 PC6  // Pin motor 1  GPIO 6
#define pin_motor2 PC7  // Pin motor 2  GPIO 5
#define pin_motor3 PB9  // Pin motor 3  GPIO 10
#define pin_motor4 PB8  // Pin motor 4  GPIO 9
int16_t esc_1, esc_2, esc_3, esc_4;

TIM_TypeDef *TIM_DEF_M1_M2 = TIM3;
TIM_TypeDef *TIM_DEF_M3_M4 = TIM4;

uint32_t channel_motor1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor1), PinMap_PWM));
uint32_t channel_motor2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor2), PinMap_PWM));
uint32_t channel_motor3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor3), PinMap_PWM));
uint32_t channel_motor4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor4), PinMap_PWM));

HardwareTimer *TIM_M1_M2 = new HardwareTimer(TIM_DEF_M1_M2);
HardwareTimer *TIM_M3_M4 = new HardwareTimer(TIM_DEF_M3_M4);

// Loop timer
uint32_t loop_timer;

// Error signal
uint8_t error;


// Setup routine
void setup() {
  Serial.begin(57600);
  delay(5000);
  
  init_components();
  led_off();

  while (Mando_canal[1] < 990 || Mando_canal[2] < 990 || Mando_canal[3] < 990 || Mando_canal[4] < 990) {
    read_rc();
    delay(4);
  }

  loop_timer = micros();
  led_on();
  Serial.println("Setup finished");
}

// Main routine
void loop() {

  reference_computation();  
  read_units();
  controllers();
  actuators();
  diagnostics();

  if (micros() - loop_timer > 4050) {
    Serial.println("LOOP SLOW");
  }
  
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}
