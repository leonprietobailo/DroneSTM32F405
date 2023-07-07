void read_process_units() {
  read_battery();
  read_rc();
  read_imu();
  process_imu();
  read_barometer();
}

void read_battery(void) {
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA7) / 352.27 * 0.9838);
  if (battery_voltage < 10.0 && error == 0) error = 1;
}

void read_rc() {
  if (flank_count == 18) {
    for (int i = 1; i <= number_channels; i++) {
      remote_channel[i] = map(pulse_instant[2 * i] - pulse_instant[2 * i - 1], 600, 1600, 1000, 2000);
    }
  }
}

void read_imu(void) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 14);
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_pitch = Wire.read() << 8 | Wire.read();
  gyro_roll = Wire.read() << 8 | Wire.read();
  gyro_yaw = Wire.read() << 8 | Wire.read();
  //gyro_pitch *= -1;
  gyro_roll *= -1;
  gyro_yaw *= -1;
  acc_x -= manual_x_cal_value;
  acc_y -= manual_y_cal_value;
  acc_z -= manual_z_cal_value;
  gyro_roll -= manual_gyro_roll_cal_value;
  gyro_pitch -= manual_gyro_pitch_cal_value;
  gyro_yaw -= manual_gyro_yaw_cal_value;
}

void process_imu() {

  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);     
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);  
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);     


  angle_pitch += (float)gyro_pitch * 0.0000611; 
  angle_roll += (float)gyro_roll * 0.0000611;   

  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066); 
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066); 

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); 

  if (abs(acc_y) < acc_total_vector) {                                
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; 
  }
  if (abs(acc_x) < acc_total_vector) {                             
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296; 
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;  

  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;

  if (!auto_level) {        
    pitch_level_adjust = 0;  
    roll_level_adjust = 0; 
  }
}

void read_barometer() {
  if (barometer_counter == 2) {
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_REGISTER_PRESSURE_MSB);
    Wire.endTransmission();
  }

  if (barometer_counter == 4) {
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

    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10;
    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location == 20)parachute_rotating_mem_location = 0;

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


///////////////////////////////////////////////////////////////////////
/// FUNCTIONS TRIGGERED BY INTERRUPTIONS                            ///
///////////////////////////////////////////////////////////////////////

void read_ultrasonic() {
  if (pulse_sent) {
    if (digitalRead(echo_pin) == HIGH) {
      pulse_start = micros();
    } else {
      pulse_end = micros();
      duration = pulse_end - pulse_start;
      computed_distance = duration / 1e6 * cAir * 1e2 / 2.0;

      if (computed_distance < 10) {
        computed_distance = 10;
      }
      if (computed_distance < 200) {
        prev_distance = distance;
        prev_distance_filt = distance_filt;
        distance = computed_distance;
        distance_filt = lp.filt(distance);
        velocity_raw = (distance - prev_distance) / (micros() - timeLast) * 1e6;
        velocityFilt = velocityFilt * 0.8 + 0.2 * (distance_filt - prev_distance_filt) / (micros() - timeLast) * 1e6;
        velocity = 0.95 * velocity + 0.05 * (distance - prev_distance) / (micros() - timeLast) * 1e6;

        timeLast = micros();
      }
      pulse_sent = false;
    }
  }
}

void read_PPM() {
  if (micros() - pulse_instant[flank_count - 1] > 2500) flank_count = 0;
  pulse_instant[flank_count] = micros();
  flank_count++;
}