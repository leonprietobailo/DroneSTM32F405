
void actuators() {
  act_esc_outputs();
  act_esc_PWM();
  act_us_pulse();
}

void act_esc_outputs() {
  if (fm == FM_stable) {                                                      
    if (throttle > 1800) throttle = 1800;                                  
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
  }

  else if (fm == FM_alt_hold){

    if (throttle > 1800) throttle = 1800;
    esc_1 = throttle + pid_output_altitude - pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_altitude + pid_output_pitch - pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_altitude + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_4 = throttle + pid_output_altitude - pid_output_pitch + pid_output_roll + pid_output_yaw;
  }

  else {
    esc_1 = 1000;  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;  //If start is not 2 keep a 1000us pulse for ess-4.
  }

  if (esc_1 < 1000) esc_1 = 950;  //Keep the motors running.
  if (esc_2 < 1000) esc_2 = 950;  //Keep the motors running.
  if (esc_3 < 1000) esc_3 = 950;  //Keep the motors running.
  if (esc_4 < 1000) esc_4 = 950;  //Keep the motors running.

  if (esc_1 > 2000) esc_1 = 2000;  //Limit the esc-1 pulse to 2000us.
  if (esc_2 > 2000) esc_2 = 2000;  //Limit the esc-2 pulse to 2000us.
  if (esc_3 > 2000) esc_3 = 2000;  //Limit the esc-3 pulse to 2000us.
  if (esc_4 > 2000) esc_4 = 2000;  //Limit the esc-4 pulse to 2000us.
}

void act_esc_PWM(){
  TIM_M1_M2->setCaptureCompare(channel_motor1, esc_1, MICROSEC_COMPARE_FORMAT);
  TIM_M1_M2->setCaptureCompare(channel_motor2, esc_2, MICROSEC_COMPARE_FORMAT);
  TIM_M3_M4->setCaptureCompare(channel_motor3, esc_3, MICROSEC_COMPARE_FORMAT);
  TIM_M3_M4->setCaptureCompare(channel_motor4, esc_4, MICROSEC_COMPARE_FORMAT);
}

void act_us_pulse() {
  if (micros() - sentLastPulse > 7500) {
    sentLastPulse = micros();
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    pulseSent = true;
  }
}
