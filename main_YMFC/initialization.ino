
void init_components(){
	init_flash();
	init_led();
	init_ultrasonic();
	init_rc();
	init_esc();
	init_gyro();
	init_barometer();
}


void init_flash(void){
  // Init external flash
	flash.begin();

  // Open file system on the flash
	if ( !fatfs.begin(&flash) ) {
		Serial.println("Error: filesystem is not existed. Please try SdFat_format example to make one.");
		while(1)
		{
			yield();
			delay(1);
		}
	}
	myFile = fatfs.open("data.csv", FILE_WRITE);
	Serial.println("initialization done.");

}

void init_led(){
	pinMode(PC1, OUTPUT);
}

void init_ultrasonic(){
  pinMode(triggerPin, OUTPUT); // Sets the trigPin as an OUTPUT
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  attachInterrupt(digitalPinToInterrupt(echoPin), read_ultrasonic, CHANGE);
}

void init_rc(){
  pinMode(pin_PPM, INPUT);                   // YAW
  attachInterrupt(digitalPinToInterrupt(pin_PPM), read_PPM, CHANGE);
}

void init_esc(){
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
}

void init_gyro(void){
	Wire.begin();                                                //Start the I2C as master
  	Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  	error = Wire.endTransmission();                              //End the transmission and register the exit status.
  	while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not responde.
  		delay(4);
  	}

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  if (use_manual_calibration)cal_int = 2000;                                          //If manual calibration is used set cal_int to 2000 to skip the calibration.
  else {
    cal_int = 0;                                                                      //If manual calibration is not used.
    manual_gyro_pitch_cal_value = 0;                                                  //Set the manual pitch calibration variable to 0.
    manual_gyro_roll_cal_value = 0;                                                   //Set the manual roll calibration variable to 0.
    manual_gyro_yaw_cal_value = 0;                                                    //Set the manual yaw calibration variable to 0.
  }

  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                    //Change the led status every 125 readings to indicate calibration.
      read_gyro();                                                                //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
      acc_x_cal += acc_x;
      acc_y_cal += acc_y;
      acc_z_cal += acc_z;
      delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    //red_led(HIGH);                                                                     //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
    acc_x_cal /= 2000;
    acc_y_cal /= 2000;
    acc_z_cal /= 2000;    
    manual_gyro_pitch_cal_value = gyro_pitch_cal;                                     //Set the manual pitch calibration variable to the detected value.
    manual_gyro_roll_cal_value = gyro_roll_cal;                                       //Set the manual roll calibration variable to the detected value.
    manual_gyro_yaw_cal_value = gyro_yaw_cal;                                         //Set the manual yaw calibration variable to the detected value.
    manual_x_cal_value = acc_x_cal;
    manual_y_cal_value = acc_y_cal;
    manual_z_cal_value = acc_z_cal - 4096;
  }
}

void init_barometer()
{

  for (st = 1; st <= 6; st++) {
    Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
    Wire.write(0xA0 + st * 2);                              //Send the address that we want to read.
    Wire.endTransmission();                                    //End the transmission.

    Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
    C[st] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_a_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

  //The MS5611 needs a few readings to stabilize.
  for (st = 0; st < 100; st++) {                       //This loop runs 100 times.
    barometer_read();                                           //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }
  actual_pressure = 0;                                          //Reset the pressure calculations.

}
