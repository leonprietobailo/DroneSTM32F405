//
//    FILE: MS5611_test.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo application
//    DATE: 2014-okt-16
//     URL: https://github.com/RobTillaart/MS5611


#include "MS5611.h"

//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77

//MS5611 MS5611(0x77);
int result;
float temperature, pressure, h;
#ifndef LED_BUILTIN
#define LED_BUILTIN    13
#endif

uint32_t start, stop;


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
uint8_t takeOFF_a_detected, manual_altitude_change;
float pressure_parachute_previous;
int32_t parachute_buffer[35], parachute_throttle;
uint8_t parachute_rotating_mem_location;



void setup()
{
  Serial.begin(115200);
  while (!Serial);

   for (start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
    Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
    Wire.endTransmission();                                    //End the transmission.

    Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
    C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_a_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

  //The MS5611 needs a few readings to stabilize.
  for (start = 0; start < 100; start++) {                       //This loop runs 100 times.
    read_barometer_v2();                                           //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }
  actual_pressure = 0;                                          //Reset the pressure calculations.

  Serial.println("SETUP FINISHED");
}


/*
  There are 5 oversampling settings, each corresponding to a different amount of milliseconds
  The higher the oversampling, the more accurate the reading will be, however the longer it will take.
  OSR_ULTRA_HIGH -> 8.22 millis
  OSR_HIGH       -> 4.11 millis
  OSR_STANDARD   -> 2.1 millis
  OSR_LOW        -> 1.1 millis
  OSR_ULTRA_LOW  -> 0.5 millis   Default = backwards compatible
*/

int timer;
void loop()
{
  
  //MS5611.setOversampling(OSR_ULTRA_HIGH);
  read_barometer();
  

  while(micros() - timer <= 4000);
  timer = micros();
}

int64_t test1;
int64_t test2;


void read_barometer(void) {
  barometer_counter ++;
  //Serial.println(barometer_counter);

  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
    if (temperature_counter == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
      Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
      Serial.println(raw_temperature_rotating_memory[average_temperature_mem_location]);
      //Serial.println("Hola");
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
      
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
      Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.
      Serial.println(raw_pressure);
    }

    temperature_counter ++;                                                     //Increase the temperature_counter variable.
    if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
      temperature_counter = 0;                                                  //Reset the temperature_counter variable.
      //Request temperature data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write(0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
    else {                                                                      //If the temperature_counter variable does not equal 20.
      //Request pressure data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write(0x48);                                                        //Send a 0x48 to indicate that we want to request the pressure data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    } 
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF_a = OFF_a_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF_a) / pow(2, 15);
    
    //Serial.println(OFF_a);
    //Serial.println("\t");
    //Serial.println(dT);    
    //Serial.print("\t");
    //Serial.println(SENS);
    //Serial.print("\t");
    //Serial.println(P);
    //Serial.println();
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
    //Serial.println(actual_pressure);
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.
  }
}


void read_barometer_v2(void){

  Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
  Wire.write(0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
  Wire.endTransmission();                                                  //End the transmission with the MS5611.

  delay(500);

  Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
  Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
  Wire.endTransmission();                                                  //End the transmission with the MS5611.
  delay(500);
  Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
  delay(500);
  test1 = Wire.read() << 16 | Wire.read() << 8 | Wire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.

//
//  Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
//  Wire.write(0x48);                                                        //Send a 0x58 to indicate that we want to request the pressure data.
//  Wire.endTransmission();                                                  //End the transmission with the MS5611.
//      
//  Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
//  Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
//  Wire.endTransmission();                                                  //End the transmission with the MS5611.
//  Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
//
//  test2 = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
//  
//  Serial.print(test1);
//  Serial.print("\t");
  Serial.println(test1);
}
