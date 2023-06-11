void read_process_units() {
  read_battery();
  read_rc();
  read_gyro();
  process_gyro();
  read_barometer();
}

void read_battery(void) {
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA7) / 352.27 * 0.9838);
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

void process_gyro() {

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);     //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);  //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);        //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;    //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));  //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                 //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;  //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;  //Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;  //Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;    //Calculate the roll angle correction.

  if (!auto_level) {         //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;  //Set the pitch angle correction to zero.
    roll_level_adjust = 0;   //Set the roll angle correcion to zero.
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
  if (pulseSent) {
    if (digitalRead(echoPin) == HIGH) {
      pulseStart = micros();
    } else {
      pulseEnd = micros();
      duration = pulseEnd - pulseStart;
      computedDistance = duration / 1e6 * cAir * 1e2 / 2.0;

      if (computedDistance < 10) {
        computedDistance = 10;
      }
      if (computedDistance < 200) {
        prevDistance = distance;
        prevDistanceFilt = distanceFilt;
        distance = computedDistance;
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