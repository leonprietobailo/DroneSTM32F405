
void actuators() {

  act_esc_PWM_v2();

  if (Mando_canal[6] < 1500) {
    throttle = Mando_canal[3];
    pid_i_mem_altitude = 0;
    pid_last_altitude_d_error = 0;
    throttle_ah = throttle;
    pid_altitude_setpoint = actual_pressure;
  } else {
    hoverThrottle = -63.4 * battery_voltage + 2183 + 20;
    throttle = hoverThrottle - pid_output_altitude;
    //throttle = throttle_ah;
  }

  act_esc_outputs();
  act_us_pulse();
}



void act_esc_PWM() {

  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  digitalWrite(pin_motor1, HIGH);
  digitalWrite(pin_motor2, HIGH);
  digitalWrite(pin_motor3, HIGH);
  digitalWrite(pin_motor4, HIGH);
  tiempo_motores_start = micros();

  // ------------------ ¡¡1ms max!! ------------------
  tiempo_1 = micros();

  //RC_procesar();             // Leer mando RC

  // Si la duracion entre tiempo_1 y tiempo_2 ha sido mayor de 900us, encender LED de aviso.
  // Nunca hay que sobrepasar 1ms de tiempo en estado HIGH.
  tiempo_2 = micros();
  tiempo_ON = tiempo_2 - tiempo_1;
  // ------------------ ¡¡1ms max!! ------------------

  // Pasamos las señales PWM a estado LOW cuando haya transcurrido el tiempo definido en las variables ESCx_us
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + esc_1 <= micros()) digitalWrite(pin_motor1, LOW);
    if (tiempo_motores_start + esc_2 <= micros()) digitalWrite(pin_motor2, LOW);
    if (tiempo_motores_start + esc_3 <= micros()) digitalWrite(pin_motor3, LOW);
    if (tiempo_motores_start + esc_4 <= micros()) digitalWrite(pin_motor4, LOW);
  }
}

void act_esc_PWM_v2(){

//  MyTim_motor1->pause();
//  MyTim_motor2->pause();
//  MyTim_motor3->pause();
//  MyTim_motor4->pause();

 // MyTim->setCaptureCompare(channel, 900000, MICROSEC_COMPARE_FORMAT);

  
  TIM_M1_M2->setCaptureCompare(channel_motor1, esc_1, MICROSEC_COMPARE_FORMAT);
  TIM_M1_M2->setCaptureCompare(channel_motor2, esc_2, MICROSEC_COMPARE_FORMAT);
  TIM_M3_M4->setCaptureCompare(channel_motor3, esc_3, MICROSEC_COMPARE_FORMAT);
  TIM_M3_M4->setCaptureCompare(channel_motor4, esc_4, MICROSEC_COMPARE_FORMAT);

//  Serial.println("Before resume");
//  MyTim_motor1->resume();
//  MyTim_motor2->resume();
//  MyTim_motor3->resume();
//  MyTim_motor4->resume();

}


void act_esc_outputs() {
  if (start == 2) {                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                    //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;  //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;  //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1000) esc_1 = 950;  //Keep the motors running.
    if (esc_2 < 1000) esc_2 = 950;  //Keep the motors running.
    if (esc_3 < 1000) esc_3 = 950;  //Keep the motors running.
    if (esc_4 < 1000) esc_4 = 950;  //Keep the motors running.

    if (esc_1 > 2000) esc_1 = 2000;  //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000) esc_2 = 2000;  //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000) esc_3 = 2000;  //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000) esc_4 = 2000;  //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;  //If start is not 2 keep a 1000us pulse for ess-4.
  }
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
