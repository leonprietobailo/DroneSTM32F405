void read_units() {
  read_battery();
  read_rc();
  read_gyro();
  read_barometer_v2();
}



void read_battery(void) {
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA7) / 352.27 * 0.9838);

  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0) error = 1;
}

void read_rc() {
  if (contador_flaco == 18) {
    for (int i = 1; i <= numero_canales; i++) {
      Mando_canal[i] = map(pulso_instante[2 * i] - pulso_instante[2 * i - 1], 600, 1600, 1000, 2000);
    }
  }
}

void read_gyro(void) {
  Wire.beginTransmission(gyro_address);          //Start communication with the gyro.
  Wire.write(0x3B);                              //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                        //End the transmission.
  Wire.requestFrom(gyro_address, 14);            //Request 14 bytes from the MPU 6050.
  acc_x = Wire.read() << 8 | Wire.read();        //Add the low and high byte to the acc_x variable.
  acc_y = Wire.read() << 8 | Wire.read();        //Add the low and high byte to the acc_y variable.
  acc_z = Wire.read() << 8 | Wire.read();        //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();  //Add the low and high byte to the temperature variable.
  gyro_pitch = Wire.read() << 8 | Wire.read();   //Read high and low part of the angular data.
  gyro_roll = Wire.read() << 8 | Wire.read();    //Read high and low part of the angular data.
  gyro_yaw = Wire.read() << 8 | Wire.read();     //Read high and low part of the angular data.
  //gyro_pitch *= -1;                                          //Invert the direction of the axis.
  gyro_roll *= -1;
  gyro_yaw *= -1;               //Invert the direction of the axis.
  acc_x -= manual_x_cal_value;  //Subtact the manual accelerometer pitch calibration value.
  acc_y -= manual_y_cal_value;  //Subtact the manual accelerometer roll calibration value.
  acc_z -= manual_z_cal_value;
  gyro_roll -= manual_gyro_roll_cal_value;    //Subtact the manual gyro roll calibration value.
  gyro_pitch -= manual_gyro_pitch_cal_value;  //Subtact the manual gyro pitch calibration value.
  gyro_yaw -= manual_gyro_yaw_cal_value;      //Subtact the manual gyro yaw calibration value.
}

///////////////////////////////////////////////////////////////////////
/// FUNCTIONS TRIGGERED BY INTERRUPTIONS [CHECK initialization.ino] ///
///////////////////////////////////////////////////////////////////////


/// Must be splitted -> Measurement Processing
void read_ultrasonic() {
  if (pulseSent) {
    if (digitalRead(echoPin) == HIGH) {
      pulseStart = micros();
    } else {
      pulseEnd = micros();
      duration = pulseEnd - pulseStart;
      computedDistance = duration / 1e6 * cAir * 1e2 / 2.0;  // distance = duration [us] / 1e6 [us/s] * speedOfSound [m/s] * 1e2 [cm/m] / 2 (go and back] || [cm]

      if (computedDistance < 10) {
        computedDistance = 10;
      }
      if (computedDistance < 200) {
        prevDistance = distance;
        prevDistanceFilt = distanceFilt;
        distance = computedDistance;  //0.95 * distance + 0.05 * computedDistance;
        distanceFilt = lp.filt(distance);
        velocity_raw = (distance - prevDistance) / (micros() - timeLast) * 1e6;
        velocityFilt = velocityFilt * 0.8 + 0.2 * (distanceFilt - prevDistanceFilt) / (micros() - timeLast) * 1e6;
        velocity = 0.95 * velocity + 0.05 * (distance - prevDistance) / (micros() - timeLast) * 1e6;

        timeLast = micros();
      }
      pulseSent = false;
    }
  }
}

void read_PPM() {
  if (micros() - pulso_instante[contador_flaco - 1] > 2500) contador_flaco = 0;
  pulso_instante[contador_flaco] = micros();
  contador_flaco++;
}


void read_barometer_v2() {
  if (barometer_counter == 2) {
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_REGISTER_PRESSURE_MSB);
    Wire.endTransmission();
  }

  if (barometer_counter == 4) {

    // Burst request of pressure and temperature:
    Wire.requestFrom(BMP280_ADDRESS, 6);

    pressure_msb = Wire.read();
    pressure_lsb = Wire.read();
    pressure_xlsb = Wire.read();
    temperature_msb = Wire.read();
    temperature_lsb = Wire.read();
    temperature_xlsb = Wire.read();

    // Putting together obtained bits
    adc_P = pressure_msb << 12 | pressure_lsb << 4 | pressure_xlsb >> 4;
    adc_T = temperature_msb << 12 | temperature_lsb << 4 | temperature_xlsb >> 4;

    //BMP280 Compensation:
    bmp280_compensate_P_int64();
    bmp280_compensate_T_int32();

    // Pressure signal smoothened:
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem[pressure_rotating_mem_location] = P;
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem_location++;
    if (pressure_rotating_mem_location == 12) pressure_rotating_mem_location = 0;
    actual_pressure_fast = (float)pressure_total_avarage / 12.0;
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;

    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > 8) actual_pressure_diff = 8;
    if (actual_pressure_diff < -8) actual_pressure_diff = -8;

    if (actual_pressure_diff > 1 || actual_pressure_diff < -1) actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;

    // LONG TERM VARIATION
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 20)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.
    
    barometer_counter = 0;
  }
  barometer_counter++;
}

void bmp280_compensate_T_int32() {
  var1_32 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2_32 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1_32 + var2_32;
  t_temp = (t_fine * 5 + 128) >> 8;
  T = (float)t_temp / 100.0;
}

void bmp280_compensate_P_int64() {
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 != 0) {
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    p_32 = (int32_t)p;
    P = float(p_32) / 256.0;
  }
}