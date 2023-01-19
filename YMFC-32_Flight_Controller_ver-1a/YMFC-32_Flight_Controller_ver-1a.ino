//#include <bsp_sd.h>
//#include <ffconf.h>
//#include <ffconf_default_32020.h>
//#include <ffconf_default_68300.h>
//#include <Sd2Card.h>
//#include <SdFatFs.h>
//#include <STM32SD.h>


///////////////////////////////////////////////////////////////////////////////////////
//SD CARD
///////////////////////////////////////////////////////////////////////////////////////

//#ifndef SD_DETECT_PIN
//#define SD_DETECT_PIN SD_DETECT_NONE
//#endif

//File dataFile;

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

float pid_p_gain_roll = 0.8; //.6;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0; //0.001;        //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 4;               //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

// PITCH PID

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

// YAW PID

float pid_p_gain_yaw = 0.75;               //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0; //0.01;          //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0;                  //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

// ALTITUDE PID

float pid_p_gain_altitude = 0.5;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 1.5;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

// GPS PD

float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).

uint16_t throttle_low  = 1140;             //Minimum Ch3 value
uint16_t throttle_high = 1826;             //Maximum Ch3 value
uint16_t roll_low      = 1053;             //Minimum Ch1 value
uint16_t roll_high     = 1952;             //Maximum Ch1 value
uint16_t pitch_low     = 1076;             //Minimum Ch2 value
uint16_t pitch_high    = 1905;             //Maximum Ch2 value
uint16_t yaw_low       = 1017;              //Minimum Ch4 value
uint16_t yaw_high      = 1952;             //Maximum Ch4 value

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
int16_t throttle, cal_int, takeoff_throttle;
float hoverThrottle;
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

uint32_t loop_timer, error_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_i_mem_altitude, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;

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
//String testString;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  delay(5000);

  // SD CARD
  //while (!SD.begin(SD_DETECT_PIN))
  //{
  //  delay(10);
  //}
  //dataFile = SD.open("log.txt", FILE_WRITE);
  //if (dataFile) {
  //  dataFile.seek(dataFile.size());
  //}
  //else {
  //  Serial.println("error opening datalog.txt");
  //}

// Green LED
  pinMode(PC1, OUTPUT);

// Distance Measurement PINS
	pinMode(triggerPin, OUTPUT); // Sets the trigPin as an OUTPUT
  digitalWrite(triggerPin, LOW);
	pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  attachInterrupt(digitalPinToInterrupt(echoPin), echoPin_trigger, CHANGE);

  // RC INTERRUPT
  pinMode(pin_PPM, INPUT);                   // YAW
  attachInterrupt(digitalPinToInterrupt(pin_PPM), read_PPM, CHANGE);


  //pinMode(4, INPUT_ANALOG);                                    //This is needed for reading the analog value of port A4.
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.esc
  //The following function connects PB3 and PB4 to the
  //alternate output function.
  //afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                    //Connects PB3 and PB4 to output function.


  // Adding interrupts

//  pinMode(pin_INT_Yaw, INPUT);                
//  attachInterrupt(digitalPinToInterrupt(pin_INT_Yaw), INT_Yaw, CHANGE);
//  pinMode(pin_INT_Throttle, INPUT);           
//  attachInterrupt(digitalPinToInterrupt(pin_INT_Throttle), INT_Throttle, CHANGE);
//  pinMode(pin_INT_Pitch, INPUT);              
//  attachInterrupt(digitalPinToInterrupt(pin_INT_Pitch), INT_Pitch, CHANGE);
//  pinMode(pin_INT_Roll, INPUT);                 
//  attachInterrupt(digitalPinToInterrupt(pin_INT_Roll), INT_Roll, CHANGE);

  // Declaración de los pines de los motores
  pinMode(pin_motor1, OUTPUT);  //Motor 1
  pinMode(pin_motor2, OUTPUT);  //Motor 2
  pinMode(pin_motor3, OUTPUT);  //Motor 3
  pinMode(pin_motor4, OUTPUT);  //Motor 4
  // Forzar los pines a estado LOW
  digitalWrite(pin_motor1, LOW);
  digitalWrite(pin_motor2, LOW);
  digitalWrite(pin_motor3, LOW);
  digitalWrite(pin_motor4, LOW);
  
  //On the Flip32 the LEDs are connected differently. A check is needed for controlling the LEDs.
//  pinMode(PB3, INPUT);                                         //Set PB3 as input.
//  pinMode(PB4, INPUT);                                         //Set PB4 as input.
//  if (digitalRead(PB3) || digitalRead(PB3))flip32 = 1;         //Input PB3 and PB4 are high on the Flip32
//  else flip32 = 0;
//

  led_off();

  Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  Serial.println("***SETUP: STARTED***");
  delay(250);                                                 //Give the serial port some time to start to prevent data loss.
  //timer_setup();                                                //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to start.
  Serial.println("***GYRO ADDRESS: SEARCHING***");
  Wire.begin();                                                //Start the I2C as master
  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not responde.
    error = 2;                                                  //Set the error status to 2.
    //error_signal();                                             //Show the error via the red LED.
    delay(4);
  }
  Serial.println("***GYRO ADDRESS: OBTAINED***");
  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.
  Serial.println("***GYRO SETUP: DONE***");
  if (!use_manual_calibration) {
    //Create a 5 second delay before calibration.
    for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
      if (count_var % 125 == 0) {                               //Every 125 loops (500ms).
        digitalWrite(PB4, !digitalRead(PB4));                   //Change the led status.
      }
      delay(4);                                                 //Delay 4 microseconds
    }
    count_var = 0;                                              //Set start back to 0.
  }

  calibrate_gyro();                                             //Calibrate the gyro offset.
  Serial.println("GYRO CALLIBRATION: DONE");
  Serial.println("WAITING FOR TRANSMITTER...");
  
  //Wait until the receiver is active.
  while (Mando_canal[1] < 990 || Mando_canal[2] < 990 || Mando_canal[3] < 990 || Mando_canal[4] < 990)  {
    read_RC();                                                //Set the error status to 3.
    //error_signal();                                             //Show the error via the red LED.
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.
  Serial.println("TRANSMITTER CONNECTED");
  
  //Wait until the throtle is set to the lower position.
//  while (Mando_canal[3] < 990 || Mando_canal[3] > 1050)  {
//    read_RC();
//  }
  error = 0;                                                    //Reset the error status to 0.
  //When everything is done, turn off the led.

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  //battery_voltage = (float)analogRead(4) / 112.81;

  loop_timer = micros();                                        //Set the timer for the first loop.

  led_on();                                         //Turn on the green led.
  Serial.println("SETUP FINISHED");

  acc_z_average_short_total = acc_z * 25;
  acc_z_average_long_total = acc_z * 50;
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
//  testString = String(millis());
//  testString += " ";
//  testString += String(throttle);
//  dataFile.println(testString);
//  dataFile.flush();
  battery_control();
  PWM();
  read_RC();
  process_RC();
  //error_signal();                                                                  //Show the errors via the red LED.
  gyro_signalen();                                                                 //Read the gyro and accelerometer data.

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

  if (!auto_level) {                                                               //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                        //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                         //Set the roll angle correcion to zero.
  }

  

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (Mando_canal[1] > 1508)pid_roll_setpoint = Mando_canal[1] - 1508;
  else if (Mando_canal[1] < 1492)pid_roll_setpoint = Mando_canal[1] - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (Mando_canal[2] > 1508)pid_pitch_setpoint = Mando_canal[2] - 1508;
  else if (Mando_canal[2] < 1492)pid_pitch_setpoint = Mando_canal[2] - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (Mando_canal[3] > 1050) { //Do not yaw when turning off the motors.
    if (Mando_canal[4] > 1508)pid_yaw_setpoint = (Mando_canal[4] - 1508) / 3.0;
    else if (Mando_canal[4] < 1492)pid_yaw_setpoint = (Mando_canal[4] - 1492) / 3.0;
  }

  calculate_pid();                                                                 //PID inputs are known. So we can calculate the pid output.
  altitude_pid();

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA7) / 352.27);

  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0)error = 1;
  

  if(Mando_canal[6] < 1500){
    throttle = Mando_canal[3];
    pid_i_mem_altitude = 0;
    pid_last_altitude_d_error= 0;
  }
  else{
    hoverThrottle = - 40.0 / 0.95 * battery_voltage + 1995.79;
    throttle = hoverThrottle - pid_output_altitude;   
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
  
  
  if (start == 2) {                                                                //The motors are started.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1000) esc_1 = 950;                                                //Keep the motors running.
    if (esc_2 < 1000) esc_2 = 950;                                                //Keep the motors running.
    if (esc_3 < 1000) esc_3 = 950;                                                //Keep the motors running.
    if (esc_4 < 1000) esc_4 = 950;                                                //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
  }

//  Serial.print(Mando_canal[1]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[2]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[3]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[4]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[5]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[6]);
//  Serial.print("\n");
//
//  Serial.print(esc_1);
//  Serial.print("\t");
//  Serial.print(esc_2);
//  Serial.print("\t");
//  Serial.print(esc_3);
//  Serial.print("\t");
//  Serial.print(esc_4);
//  Serial.print("\n");

//    Serial.print(acc_x);
//    Serial.print("\t");
//    Serial.print(acc_y);
//    Serial.print("\t");
//    Serial.print(acc_z);
//    Serial.print("\t");
//    Serial.print(temperature);
//    Serial.print("\t");
//    Serial.print(gyro_pitch);
//    Serial.print("\t");
//    Serial.print(gyro_roll);
//    Serial.print("\t");
//    Serial.print(gyro_yaw);
//    Serial.print("\n");
 
  Serial.println(hoverThrottle);

  //Serial.println(takeoff_throttle);
  //Serial.println(start);
  
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

  if (micros() - loop_timer > 4050)Serial.println("LOOP SLOW");                                      //Turn on the LED if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}

long tiempo_motores_start, tiempo_1, tiempo_2, tiempo_ON;

void PWM() {
  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  digitalWrite(pin_motor1, HIGH);
  digitalWrite(pin_motor2, HIGH);
  digitalWrite(pin_motor3, HIGH);
  digitalWrite(pin_motor4, HIGH);
  tiempo_motores_start = micros();

  // ------------------ ¡¡1ms max!! ------------------
  tiempo_1 = micros();

  //RC_procesar();             // Leer mando RC

  // Si la duracion entre tiempo_1 y tiempo_2 ha sido mayor de 900us, encender LED de aviso.
  // Nunca hay que sobrepasar 1ms de tiempo en estado HIGH.
  tiempo_2 = micros();
  tiempo_ON = tiempo_2 - tiempo_1;
  // ------------------ ¡¡1ms max!! ------------------

  // Pasamos las señales PWM a estado LOW cuando haya transcurrido el tiempo definido en las variables ESCx_us
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + esc_1 <= micros()) digitalWrite(pin_motor1, LOW);
    if (tiempo_motores_start + esc_2 <= micros()) digitalWrite(pin_motor2, LOW);
    if (tiempo_motores_start + esc_3 <= micros()) digitalWrite(pin_motor3, LOW);
    if (tiempo_motores_start + esc_4 <= micros()) digitalWrite(pin_motor4, LOW);
  }
}

//float RC_Throttle_filt, RC_Pitch_filt, RC_Yaw_filt, RC_Roll_filt;
//
//void RC_procesar() {
//  //  Filtrado de lecturas raw del mando RC
//  RC_Throttle_filt = RC_Throttle_filt * 0.9 + channel_3 * 0.1;
//  RC_Pitch_filt    = RC_Pitch_filt * 0.9 + channel_2 * 0.1;
//  RC_Roll_filt     = RC_Roll_filt  * 0.9 + channel_1  * 0.1;
//  RC_Yaw_filt      = RC_Yaw_filt   * 0.9 + channel_4   * 0.1;
//  
//  // Mapeo de señales del mando RC
//  RC_Throttle_consigna = map(RC_Throttle_filt, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
//  RC_Pitch_consigna    = map(RC_Pitch_filt, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
//  RC_Roll_consigna     = map(RC_Roll_filt, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
//  RC_Yaw_consigna      = map(RC_Yaw_filt, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);
//
////  // Si las lecturas son cercanas a 0, las forzamos a 0 para evitar inclinar el drone por error
////  if (RC_Pitch_consigna < 3 && RC_Pitch_consigna > -3)RC_Pitch_consigna = 0;
////  if (RC_Roll_consigna  < 3 && RC_Roll_consigna  > -3)RC_Roll_consigna  = 0;
////  if (RC_Yaw_consigna   < 3 && RC_Yaw_consigna   > -3)RC_Yaw_consigna   = 0;
//
//}
