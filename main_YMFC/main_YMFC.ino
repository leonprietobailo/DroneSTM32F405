//#include <bsp_sd.h>
//#include <ffconf.h>
//#include <ffconf_default_32020.h>
//#include <ffconf_default_68300.h>
//#include <Sd2Card.h>
//#include <SdFatFs.h>
//#include <STM32SD.h>
#include <Wire.h>

// ULTRASONIC

float sentLastPulse, duration, pulseStart, pulseEnd, computedDistance;
bool pulseSent;

// Air temperature linear aproximation, temeprature to be replaced with vlaue obtainted from MPU6050
long cAir = 331.3 + 20.0 * 0.606;     

// LED
volatile long led_timer;

// BUZZER

float batteryVoltage;
int buzzerTimer;
bool toneOn;

///////////////////////////////////////////////////////////////////////////////////////
//FLASH STORAGE
///////////////////////////////////////////////////////////////////////////////////////

#define length_str 20000
uint16_t voltage_str[length_str];
uint16_t throttle_str[length_str];
uint16_t n_str = 0;

long tiempo_motores_start, tiempo_1, tiempo_2, tiempo_ON;

///////////////////////////////////////////////////////////////////////////////////////
//ALTITUDE CONTROL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////
int result;
float temperature_bar, pressure, h;

uint32_t st, stop;

// ULTRASONIC
unsigned long timeLast;
float prevDistance;

// BAROMETER FCN

uint8_t MS5611_address = 0x77;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
int32_t dT, dT_C5;
uint16_t C[7];
int64_t OFF_a, OFF_a_C2, SENS, SENS_C1, P;
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
uint8_t takeOFF_aa_detected, manual_altitude_change;
float pressure_parachute_previous;
int32_t parachute_buffer[35], parachute_throttle;
uint8_t parachute_rotating_mem_location;


///////////////////////////////////////////////////////////////////////////////////////
//GPS VARIABLES
///////////////////////////////////////////////////////////////////////////////////////

//GPS variables
uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set, latitude_north, longiude_east ;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;
int8_t flight_mode;
float angle_yaw;
///////////////////////////////////////////////////////////////////////////////////////
//RC
///////////////////////////////////////////////////////////////////////////////////////

#define numero_canales 8
uint64_t pulso_instante[numero_canales * 2 + 2];
uint16_t Mando_canal[numero_canales];
uint8_t contador_flaco = 1;

///////////////////////////////////////////////////////////////////////////////////////
//AUTO TAKE-OFF
///////////////////////////////////////////////////////////////////////////////////////
uint8_t takeoff_detected = 0;


///////////////////////////////////////////////////////////////////////////////////////
//BATTERY CONTROL
///////////////////////////////////////////////////////////////////////////////////////

#define pin_BAT PA5
#define pin_BUZZER PA6

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROLL PID

float pid_p_gain_roll = 0.9;
float pid_i_gain_roll = 0.009;
float pid_d_gain_roll = 4;
int pid_max_roll = 400;

float pid_i_gain_roll_in = 0;

// PITCH PID

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_i_gain_pitch_in = 0;

// YAW PID

float pid_p_gain_yaw = 0.75;
float pid_i_gain_yaw = 0.01;
float pid_d_gain_yaw = 0;
int pid_max_yaw = 400;

float pid_i_gain_yaw_in = 0;

// ALTITUDE PID

float pid_p_gain_altitude = 1;
float pid_i_gain_altitude = 0;
float pid_d_gain_altitude = 15;
int pid_max_altitude = 400;

// ALTITUDE PID V2

float pid_p_gain_altitude_v2 = 0.0005;
float pid_i_gain_altitude_v2 = 0;
float pid_d_gain_altitude_v2 = 0;
int pid_max_altitude_v2 = 400;  

// GPS PD

float gps_p_gain = 2.7;
float gps_d_gain = 6.5;

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

uint8_t gyro_address = 0x68;

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t highByte, lowByte, flip32, start, warning;
uint8_t error, error_counter, error_led;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, cal_int, hover_throttle, takeoff_throttle;
float hoverThrottle;
float throttle_ah = 800;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int16_t loop_counter;
int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total;

int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int32_t acc_x_cal, acc_y_cal, acc_z_cal;

uint32_t loop_timer, error_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_i_mem_altitude, pid_i_mem_altitude_v2, pid_altitude_input, pid_altitude_v2_input, pid_output_altitude, pid_output_altitude_v2, pid_last_altitude_d_error, pid_last_altitude_v2_d_error;

float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;

#define pin_PPM PA4

#define pin_motor1 PC6        // Pin motor 1  GPIO 6
#define pin_motor2 PC7        // Pin motor 2  GPIO 5  
#define pin_motor3 PB9        // Pin motor 3  GPIO 10
#define pin_motor4 PB8        // Pin motor 4  GPIO 9

#define triggerPin PC3           // Trigger Pin Ultrasonico
#define echoPin PC2           // Echo Pin Ultrasonico

float distance;
float velocity;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  delay(5000);

  init_components();
  led_off();
  
  while (Mando_canal[1] < 990 || Mando_canal[2] < 990 || Mando_canal[3] < 990 || Mando_canal[4] < 990)  {
    read_rc();
    delay(4);
  }
  
  loop_timer = micros();
  led_on();
  Serial.println("Setup finished");  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // Compute reference architecture must be added
  read_units();
  measurement_processing();
  controllers();
  actuators();
  diagnostics();

  //manage_throttle();
  //barometer_read();  //To be splitted within read_units and measurement_processing
  
  if (micros() - loop_timer > 4050){
    Serial.print("LOOP SLOW");
    Serial.print("\t");
    
  }
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}
