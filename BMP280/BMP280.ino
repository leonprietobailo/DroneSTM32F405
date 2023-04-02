#include <Wire.h>
#include "LowPass.h"


#define BMP280_ADDRESS 0x76  // BMP280 I2C address (can also be 0x77)
#define BMP280_REGISTER_PRESSURE_MSB 0xF7  // pressure register address
#define BMP280_REGISTER_PRESSURE_LSB 0xF8  // pressure register address
#define BMP280_REGISTER_PRESSURE_XLSB 0xF9  // pressure register address
#define BMP280_REGISTER_TEMPERATURE_MSB 0xFA  // pressure register address
#define BMP280_REGISTER_TEMPERATURE_LSB 0xFB  // pressure register address
#define BMP280_REGISTER_TEMPERATURE_XLSB 0xFC  // pressure register address


int32_t t_fine;
LowPass<2> lp(0.50,20.8333333333,false);

// BMP COMPENSATION PARAMETERS
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;


int64_t var1, var2, p;
int32_t p_32;

int32_t var1_32, var2_32;
int32_t temperature_msb, temperature_lsb, temperature_xlsb, adc_T, t_temp;
float T, P, P_filt;
int32_t pressure_msb, pressure_lsb, pressure_xlsb, adc_P;



float pressure_total_avarage, pressure_rotating_mem[12], actual_pressure_fast, actual_pressure_slow, actual_pressure_diff, actual_pressure;
uint8_t pressure_rotating_mem_location;
    
// Parachute throttle: 
int32_t parachute_buffer[20], parachute_throttle;
float pressure_parachute_previous;
uint8_t parachute_rotating_mem_location;


void setup() {
  Serial.begin(115200);  // initialize serial communication
  Wire.begin();  // initialize I2C communication

  init_params();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF4); // “ctrl_meas” register
  Wire.write(0x57); // 57 (01010111) -> 010 -> x2 Temperature | 101: x16 Pressure | 11 -> Normal Mode
  //Wire.write(0x33); // 33 (00110011) -> 010 -> x2 Temperature | 101: x16 Pressure | 11 -> Normal Mode 
  Wire.endTransmission();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF5); // “config” register
  Wire.write(0x08); // AB (0001000) -> 000: t_sb: 0,5ms | 100 -> IIR Filter 16 | 0 -> No SPI 
  Wire.endTransmission();
}

//  Wire.beginTransmission(BMP280_ADDRESS);
//  Wire.write(0xF3);
//  Wire.endTransmission();
//  Wire.requestFrom(BMP280_ADDRESS, 1);
  
int64_t l_timer = 0;
float old_P;
void loop() {

  old_P = P;
  read_sensor();
  
  //read_temperature();
  //read_pressure();
  //Serial.print(micros() - l_timer);
  //Serial.print("\t");
  //Serial.println(P);

  //Serial.println(parachute_throttle);
  //Serial.print(actual_pressure);
  //Serial.print("\t");
  //Serial.println(P);
  if(old_P != P){
    //Serial.println(P);
  }

  while( micros() - l_timer < 4000); 
  l_timer = micros();
}

void init_params(){
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_ADDRESS, 24);
  dig_T1 = Wire.read() | Wire.read() << 8;
  dig_T2 = Wire.read() | Wire.read() << 8;
  dig_T3 = Wire.read() | Wire.read() << 8;
  dig_P1 = Wire.read() | Wire.read() << 8;
  dig_P2 = Wire.read() | Wire.read() << 8;
  dig_P3 = Wire.read() | Wire.read() << 8;
  dig_P4 = Wire.read() | Wire.read() << 8;
  dig_P5 = Wire.read() | Wire.read() << 8;
  dig_P6 = Wire.read() | Wire.read() << 8;
  dig_P7 = Wire.read() | Wire.read() << 8;
  dig_P8 = Wire.read() | Wire.read() << 8;
  dig_P9 = Wire.read() | Wire.read() << 8;
}

uint8_t sensorCnt = 0;
void read_sensor() {

  if(sensorCnt == 2) {
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_REGISTER_PRESSURE_MSB);
    Wire.endTransmission();
  }

  if(sensorCnt == 4){
    
    Wire.requestFrom(BMP280_ADDRESS, 6);
  
    pressure_msb = Wire.read();
    pressure_lsb = Wire.read();
    pressure_xlsb = Wire.read();
    temperature_msb = Wire.read();
    temperature_lsb = Wire.read();
    temperature_xlsb = Wire.read();
    
  
    adc_P = pressure_msb << 12 | pressure_lsb << 4 | pressure_xlsb >> 4;
    adc_T = temperature_msb << 12 | temperature_lsb << 4 | temperature_xlsb >> 4;
    
    bmp280_compensate_P_int64();
    bmp280_compensate_T_int32();
    sensorCnt = 0;
    
    P_filt = lp.filt(P);
//    
//    Serial.print(P);
//    Serial.print("\t");
//    Serial.println(P_filt);



    
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location]; 
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                              
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem_location++;                     
    if (pressure_rotating_mem_location == 12)pressure_rotating_mem_location = 0;       
    actual_pressure_fast = (float)pressure_total_avarage / 12.0;      
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;

    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                     
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                               
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                
    
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;


    // LONG TERM VARIATION
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 20)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.
    Serial.println(parachute_rotating_mem_location);
    
  }
  sensorCnt++;
}

void read_temperature(){
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(BMP280_REGISTER_TEMPERATURE_XLSB);
  Wire.endTransmission();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(BMP280_REGISTER_TEMPERATURE_LSB);
  Wire.endTransmission();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(BMP280_REGISTER_TEMPERATURE_MSB);
  Wire.endTransmission();

  Wire.requestFrom(BMP280_ADDRESS, 3);
  
  temperature_msb = Wire.read();
  temperature_lsb = Wire.read();
  temperature_xlsb = Wire.read();

  adc_T = temperature_msb << 12 | temperature_lsb << 4 | temperature_xlsb >> 4;

  bmp280_compensate_T_int32();
}


void bmp280_compensate_T_int32(){
  var1_32 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2_32 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1_32 + var2_32;
  t_temp = (t_fine * 5 + 128) >> 8;
  T = (float)t_temp / 100.0;
}




void read_pressure(){
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(BMP280_REGISTER_PRESSURE_XLSB);
  Wire.endTransmission();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(BMP280_REGISTER_PRESSURE_LSB);
  Wire.endTransmission();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(BMP280_REGISTER_PRESSURE_MSB);
  Wire.endTransmission();

  Wire.requestFrom(BMP280_ADDRESS, 3);
  
  pressure_msb = Wire.read();
  pressure_lsb = Wire.read();
  pressure_xlsb = Wire.read();

  adc_P = pressure_msb << 12 | pressure_lsb << 4 | pressure_xlsb >> 4;

  bmp280_compensate_P_int64();
}





void bmp280_compensate_P_int64(){
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 != 0)
  {
      p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  p_32 = (int32_t)p;
  P = float(p_32) / 256.0;
  }  
}
