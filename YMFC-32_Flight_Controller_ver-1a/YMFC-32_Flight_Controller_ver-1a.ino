//#include <bsp_sd.h>
//#include <ffconf.h>
//#include <ffconf_default_32020.h>
//#include <ffconf_default_68300.h>
//#include <Sd2Card.h>
//#include <SdFatFs.h>
//#include <STM32SD.h>


///////////////////////////////////////////////////////////////////////////////////////
//FLASH STORAGE
///////////////////////////////////////////////////////////////////////////////////////

#define length_str 20000
uint16_t voltage_str[length_str];
uint16_t throttle_str[length_str];
uint16_t n_str = 0;

long tiempo_motores_start, tiempo_1, tiempo_2, tiempo_ON;
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

///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
//HardWire HWire(2, I2C_FAST_MODE);          //Initiate I2C port 2 at 400kHz.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROLL PID

float pid_p_gain_roll = 0.9; //.6;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.009; //0.0075; //0.001;        //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 4;               //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_i_gain_roll_in = 0;

// PITCH PID

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_i_gain_pitch_in = 0;

// YAW PID

float pid_p_gain_yaw = 0.75;               //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.01;          //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0;                  //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

float pid_i_gain_yaw_in = 0;

// ALTITUDE PID

float pid_p_gain_altitude = 1;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 15;// 9.5;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

// ALTITUDE PID V2

float pid_p_gain_altitude_v2 = 0.0005;      //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude_v2 = 0;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude_v2 = 0;           //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude_v2 = 400;  

// GPS PD

float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).

//uint16_t throttle_low  = 1140;             //Minimum Ch3 value
//uint16_t throttle_high = 1826;             //Maximum Ch3 value
//uint16_t roll_low      = 1053;             //Minimum Ch1 value
//uint16_t roll_high     = 1952;             //Maximum Ch1 value
//uint16_t pitch_low     = 1076;             //Minimum Ch2 value
//uint16_t pitch_high    = 1905;             //Maximum Ch2 value
//uint16_t yaw_low       = 1017;              //Minimum Ch4 value
//uint16_t yaw_high      = 1952;             //Maximum Ch4 value

boolean auto_level = true;                 //Auto level on (true) or off (false).

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 0;
int16_t manual_acc_roll_cal_value = 0;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false;    // Set to false or true;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;
int16_t manual_x_cal_value = 0;
int16_t manual_y_cal_value = 0;
int16_t manual_z_cal_value = 0;

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

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


//#define pin_INT_Throttle PA6  // Pin Throttle del mando RC
//#define pin_INT_Yaw PA7       // Pin Yaw del mando RC  
//#define pin_INT_Pitch PA5     // Pin Pitch del mando RC 
//#define pin_INT_Roll PA4      // Pin Roll del mando RC  

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

  flash_setup();
  led_setup();
  ultrasonic_setup();
  rc_setup();
  motors_setup();
  led_off();
  gyro_search();
  gyro_setup();
  calibrate_gyro();
  
  while (Mando_canal[1] < 990 || Mando_canal[2] < 990 || Mando_canal[3] < 990 || Mando_canal[4] < 990)  {
    read_RC();
    delay(4);
  }
  Serial.println("TRANSMITTER CONNECTED");
  
  //Wait until the throtle is set to the lower position.
//  while (Mando_canal[3] < 990 || Mando_canal[3] > 1050)  {
//    read_RC();
//  }

  loop_timer = micros();                                        //Set the timer for the first loop.

  led_on();                                         //Turn on the green led.
  Serial.println("SETUP FINISHED");

//  acc_z_average_short_total = acc_z * 25;
//  acc_z_average_long_total = acc_z * 50;
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  battery_control();
  esc_PWM();
  read_RC();
  process_RC();
  gyro_signalen();                                                                 //Read the gyro and accelerometer data.
  flash_write();
  gyro_process();
  pid_attitude_sp();
  calculate_pid();                                                                 //PID inputs are known. So we can calculate the pid output.
  altitude_pid();
  altitude_pid_v2();
  battery_control();
  
  //manage_throttle();

  if(Mando_canal[6] < 1500){
    throttle = Mando_canal[3];
    pid_i_mem_altitude = 0;
    pid_last_altitude_d_error = 0;
	  throttle_ah = 800;
  }
  else{
    //hoverThrottle = -63.4 * battery_voltage + 2183;
    //throttle = hoverThrottle - pid_output_altitude;   
    throttle = throttle_ah;
  }

                                                            //We need the throttle signal as a base signal.
//  if (takeoff_detected == 1 && start == 2) {                                         //If the quadcopter is started and flying.
//    throttle = Mando_canal[3] + takeoff_throttle;                                         //The base throttle is the receiver throttle channel + the detected take-off throttle.
//    if (flight_mode >= 2) {                                                          //If altitude mode is active.
//      throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;    //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
//    }
  //}
  measure_distance();
  //ultrasonicCorrection();
  //ultrasonicCorrectonV2();
  //ultrasonicCorrectonV3();
  //Serial.println(throttle);
  //Serial.println(velocity);
  esc_compute_outputs();
  serial_print();

  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/Nju9rvZOjVQ
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  //Serial.println(micros() - loop_timer);
  if (micros() - loop_timer > 4050){
    Serial.print("LOOP SLOW");                                      //Turn on the LED if the loop time exceeds 4050us.
    Serial.print("\t");
    
  }
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}
