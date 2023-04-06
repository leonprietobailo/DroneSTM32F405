void measurement_processing() {
  //process_rc();
  process_gyro();
}


//void process_rc(void) {
//
//  //For starting the motors: throttle low and yaw left (step 1).
//  if (Mando_canal[3] < 1100 && Mando_canal[4] < 1100) start = 1;
//  //When yaw stick is back in the center position start the motors (step 2).
//  if (start == 1 && Mando_canal[3] < 1100 && Mando_canal[4] > 1450) {
//    start = 2;
//    throttle = 950;
//    acc_total_vector_at_start = acc_total_vector;  //Register the acceleration when the quadcopter is started.
//    led_off();                                     //Turn off the green led.
//
//    angle_pitch = angle_pitch_acc;  //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
//    angle_roll = angle_roll_acc;    //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
//
//    //Reset the PID controllers for a bumpless start.
//    pid_i_mem_roll = 0;
//    pid_last_roll_d_error = 0;
//    pid_i_mem_pitch = 0;
//    pid_last_pitch_d_error = 0;
//    pid_i_mem_yaw = 0;
//    pid_last_yaw_d_error = 0;
//  }
//  //Stopping the motors: throttle low and yaw right.
//  if (start == 2 && Mando_canal[3] < 1050 && Mando_canal[4] > 1950) {
//    start = 0;
//    led_on();  //Turn on the green led.
//    takeoff_detected = 0;
//  }




//  if (takeoff_detected == 0 && start == 2) {                                 //When the quadcopter is started and no take-off is detected.
//                                                                             //    if (Mando_canal[3] > 1480 && throttle < 1750) throttle++;                           //When the throttle is half way or higher, increase the throttle.
//                                                                             //    if (throttle == 1750)error = 6;                                                //If take-off is not detected when the throttle has reached 1700: error = 6.
//                                                                             //    if (Mando_canal[3] <= 1480) {                                                       //When the throttle is below the center stick position.
//                                                                             //      if (throttle > 950)throttle--;                                  //Lower the throttle to the motor_idle_speed variable.
//                                                                             //      //Reset the PID controllers for a smooth take-off.
//                                                                             //      else {                                                                       //When the throttle is back at idle speed reset the PID controllers.
//                                                                             //        pid_i_mem_roll = 0;
//                                                                             //        pid_last_roll_d_error = 0;
//                                                                             //        pid_output_roll = 0;
//                                                                             //        pid_i_mem_pitch = 0;
//                                                                             //        pid_last_pitch_d_error = 0;
//                                                                             //        pid_output_pitch = 0;
//                                                                             //        pid_i_mem_yaw = 0;
//                                                                             //        pid_last_yaw_d_error = 0;
//                                                                             //        pid_output_yaw = 0;
//                                                                             //        }
//                                                                             //      }
//    if (acc_z_average_short_total / 25 - acc_total_vector_at_start > 800) {  //A take-off is detected when the quadcopter is accelerating.
//      takeoff_detected = 1;                                                  //Set the take-off detected variable to 1 to indicate a take-off.
//      //pid_altitude_setpoint = ground_pressure - 22;                                //Set the altitude setpoint at groundlevel + approximately 2.2 meters.
//      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - 1530;  //If the automated throttle is between 1400 and 1700us during take-off, calculate take-off throttle.
//      else {                                                                       //If the automated throttle is not between 1400 and 1600us during take-off.
//        takeoff_throttle = 0;                                                      //No take-off throttle is calculated.
//        //error = 7;                                                                 //Show error 7 on the red LED.
//      }
//    }
//  }
//}


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
