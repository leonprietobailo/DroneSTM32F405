///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
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

void gyro_process(){
  
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
}
