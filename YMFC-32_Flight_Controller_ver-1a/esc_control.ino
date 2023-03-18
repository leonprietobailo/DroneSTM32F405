

void motors_setup(){
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

void esc_compute_outputs(){
  if (start == 2) {                                                                //The motors are started.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1000) esc_1 = 950;                                                //Keep the motors running.
    if (esc_2 < 1000) esc_2 = 950;                                                //Keep the motors running.
    if (esc_3 < 1000) esc_3 = 950;                                                //Keep the motors running.
    if (esc_4 < 1000) esc_4 = 950;                                                //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
  }
}

void esc_PWM(){

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
