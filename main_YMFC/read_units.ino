void read_units(){
	read_battery();
  read_rc();
  read_gyro();
}



void read_battery(void) {
	//The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA7) / 352.27*0.9838);

  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0)error = 1;
}

void read_rc() {
  if (contador_flaco == 18) {
    for (int i = 1; i <= numero_canales; i++) {
      Mando_canal[i] = map(pulso_instante[2 * i] - pulso_instante[2 * i - 1], 600, 1600, 1000, 2000);
    }
  }
}

void read_gyro(void) {
  Wire.beginTransmission(gyro_address);                      //Start communication with the gyro.
  Wire.write(0x3B);                                          //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                    //End the transmission.
  Wire.requestFrom(gyro_address, 14);                        //Request 14 bytes from the MPU 6050.
  acc_x = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_x variable.
  acc_y = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_y variable.
  acc_z = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();              //Add the low and high byte to the temperature variable.
  gyro_pitch = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data.
  gyro_roll = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
  gyro_yaw = Wire.read() << 8 | Wire.read();                 //Read high and low part of the angular data.
  //gyro_pitch *= -1;                                          //Invert the direction of the axis.
  gyro_roll *= -1;
  gyro_yaw *= -1;                                            //Invert the direction of the axis.
  acc_x -= manual_x_cal_value;                       //Subtact the manual accelerometer pitch calibration value.
  acc_y -= manual_y_cal_value;                        //Subtact the manual accelerometer roll calibration value.
  acc_z -= manual_z_cal_value;
  gyro_roll -= manual_gyro_roll_cal_value;                   //Subtact the manual gyro roll calibration value.
  gyro_pitch -= manual_gyro_pitch_cal_value;                 //Subtact the manual gyro pitch calibration value.
  gyro_yaw -= manual_gyro_yaw_cal_value;                     //Subtact the manual gyro yaw calibration value.
}

///////////////////////////////////////////////////////////////////////
/// FUNCTIONS TRIGGERED BY INTERRUPTIONS [CHECK initialization.ino] ///
///////////////////////////////////////////////////////////////////////


/// Must be splitted -> Measurement Processing
void read_ultrasonic(){
  if(pulseSent){
    if (digitalRead(echoPin) == HIGH){
      pulseStart = micros();
    }
    else{
      pulseEnd = micros();
      duration = pulseEnd - pulseStart;
      computedDistance = duration / 1e6 * cAir * 1e2 / 2.0;     // distance = duration [us] / 1e6 [us/s] * speedOfSound [m/s] * 1e2 [cm/m] / 2 (go and back] || [cm]

      if (computedDistance < 10){
        computedDistance = 10;
      }
      if (computedDistance < 200){
        prevDistance = distance;
        distance = computedDistance; //0.95 * distance + 0.05 * computedDistance;
        distanceFilt = lp.filt(distance);
        velocity_raw = (distance - prevDistance) / (micros() - timeLast) * 1e6;
        velocityFilt = (distanceFilt - prevDistance) / (micros() - timeLast) * 1e6;
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
