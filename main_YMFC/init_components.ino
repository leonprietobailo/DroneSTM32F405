
void init_components() {
  init_flash();
  init_led();
  init_ultrasonic();
  init_rc();
  init_esc();
  init_gyro();
  init_barometer();
}


void init_flash(void) {
  // Init external flash
  flash.begin();

  // Open file system on the flash
  if (!fatfs.begin(&flash)) {
    Serial.println("Error: filesystem is not existed. Please try SdFat_format example to make one.");
    while (1) {
      yield();
      delay(1);
    }
  }
  myFile = fatfs.open("data.csv", FILE_WRITE);
  Serial.println("initialization done.");
}

void init_led() {
  pinMode(PC1, OUTPUT);
}

void init_ultrasonic() {
  pinMode(trigger_pin, OUTPUT);
  digitalWrite(trigger_pin, LOW);
  pinMode(echo_pin, INPUT);  
  attachInterrupt(digitalPinToInterrupt(echo_pin), read_ultrasonic, CHANGE);
}

void init_rc() {
  pinMode(pin_PPM, INPUT);  // YAW
  attachInterrupt(digitalPinToInterrupt(pin_PPM), read_PPM, CHANGE);
}

void init_esc() {
  TIM_M1_M2->setPWM(channel_motor1, pin_motor1, 250, 0);
  TIM_M1_M2->setPWM(channel_motor2, pin_motor2, 250, 0);
  TIM_M3_M4->setPWM(channel_motor3, pin_motor3, 250, 0);
  TIM_M3_M4->setPWM(channel_motor4, pin_motor4, 250, 0);  
}


void init_gyro(void) {
  Wire.begin();                    
  Wire.beginTransmission(MPU6050_ADDRESS);
  error = Wire.endTransmission();      
  while (error != 0) {                
    delay(4);
  }

  Wire.beginTransmission(MPU6050_ADDRESS);  
  Wire.write(MPU6050_PWR_MGMT_1);                    
  Wire.write(0x00);                   
  Wire.endTransmission();              

  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(MPU6050_GYRO_CONFIG);       
  Wire.write(0x08);                   
  Wire.endTransmission();            

  Wire.beginTransmission(MPU6050_ADDRESS);  
  Wire.write(MPU6050_ACCEL_CONFIG);         
  Wire.write(0x10);                   
  Wire.endTransmission();             

  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(MPU6050_CONFIG);              
  Wire.write(0x03);                     
  Wire.endTransmission();            

  if (use_manual_calibration) cal_int = 2000;  
  else {
    cal_int = 0;                     
    manual_gyro_pitch_cal_value = 0; 
    manual_gyro_roll_cal_value = 0;  
    manual_gyro_yaw_cal_value = 0; 
  }

  if (cal_int != 2000) {
    for (cal_int = 0; cal_int < 2000; cal_int++) {                  
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4)); 
      read_gyro();                                              
      gyro_roll_cal += gyro_roll;                                  
      gyro_pitch_cal += gyro_pitch;                             
      gyro_yaw_cal += gyro_yaw;                                 
      acc_x_cal += acc_x;
      acc_y_cal += acc_y;
      acc_z_cal += acc_z;
      delay(4);  
    }
    gyro_roll_cal /= 2000;  
    gyro_pitch_cal /= 2000;  
    gyro_yaw_cal /= 2000; 
    acc_x_cal /= 2000;
    acc_y_cal /= 2000;
    acc_z_cal /= 2000;
    manual_gyro_pitch_cal_value = gyro_pitch_cal;
    manual_gyro_roll_cal_value = gyro_roll_cal;    
    manual_gyro_yaw_cal_value = gyro_yaw_cal;   
    manual_x_cal_value = acc_x_cal;
    manual_y_cal_value = acc_y_cal;
    manual_z_cal_value = acc_z_cal - 4096;
  }
}


void init_barometer() {

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

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF4);  
  Wire.write(0x57); 
  Wire.endTransmission();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF5); 
  Wire.write(0x08); 
  Wire.endTransmission();
}
