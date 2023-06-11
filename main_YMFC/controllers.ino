
void controllers() {
  cnt_attitude_sp();
  cnt_attitude_pid();
  cnt_altitude_pid();
}

void cnt_attitude_sp() {
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (Mando_canal[1] > 1501) pid_roll_setpoint = Mando_canal[1] - 1501;
  else if (Mando_canal[1] < 1499) pid_roll_setpoint = Mando_canal[1] - 1499;

  pid_roll_setpoint -= roll_level_adjust;  //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (Mando_canal[2] > 1501) pid_pitch_setpoint = Mando_canal[2] - 1501;
  else if (Mando_canal[2] < 1499) pid_pitch_setpoint = Mando_canal[2] - 1499;

  pid_pitch_setpoint -= pitch_level_adjust;  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (Mando_canal[3] > 1050) {  //Do not yaw when turning off the motors.
    if (Mando_canal[4] > 1550) pid_yaw_setpoint = (Mando_canal[4] - 1550) / 3.0;
    else if (Mando_canal[4] < 1450) pid_yaw_setpoint = (Mando_canal[4] - 1450) / 3.0;
  }
}

void cnt_attitude_pid() {
  //Roll calculations
  float pid_i_gain_roll_in;
  if (distance > 25) {
    pid_i_gain_roll_in = pid_i_gain_roll;
  } else {
    pid_i_gain_roll_in = 0;
    pid_i_mem_roll = 0;
  }
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll_in * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  float pid_i_gain_pitch_in;
  if (distance > 25) {
    pid_i_gain_pitch_in = pid_i_gain_pitch;
  } else {
    pid_i_gain_pitch_in = 0;
    pid_i_mem_pitch = 0;
  }
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch_in * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  float pid_i_gain_yaw_in;
  if (distance > 25) {
    pid_i_gain_yaw_in = pid_i_gain_yaw;
  } else {
    pid_i_gain_yaw_in = 0;
    pid_i_mem_yaw = 0;
  }
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw_in * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void cnt_altitude_pid() {

  if (barometer_counter == 1) {
  pid_altitude_input = actual_pressure;
  pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

  pid_error_gain_altitude = 0;
  if (pid_error_temp > 10 || pid_error_temp < -10) {
    pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;
    if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;
  }

  pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
  if (pid_i_mem_altitude > pid_max_altitude) pid_i_mem_altitude = pid_max_altitude;
  else if (pid_i_mem_altitude < pid_max_altitude * -1) pid_i_mem_altitude = pid_max_altitude * -1;
  pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
  if (pid_output_altitude > pid_max_altitude) pid_output_altitude = pid_max_altitude;
  else if (pid_output_altitude < pid_max_altitude * -1) pid_output_altitude = pid_max_altitude * -1;
  }
}
