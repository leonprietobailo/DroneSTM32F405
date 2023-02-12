float sentLastPulse, duration, pulseStart, pulseEnd, computedDistance;
bool pulseSent;

// Air temperature linear aproximation, temeprature to be replaced with vlaue obtainted from MPU6050
long cAir = 331.3 + 20.0 * 0.606;     

void measure_distance(){
  if (micros() - sentLastPulse > 7500){
    sentLastPulse = micros();
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    pulseSent = true;
  } 
}


bool startUltrasonic = false;
bool performAutolanding = false;
int16_t mappedThrottle, throttleOutput;
unsigned long throttleTimer;

void ultrasonicCorrection(){
  if (distance >= 100){ // ADD START COMMAND
    startUltrasonic = true;
  }
  if (startUltrasonic){
    led_slow();
    if (distance < 40) {
      mappedThrottle = map(distance, 60, 0, 0, 100);
    }
    else{
      mappedThrottle = 0;
    }
    
    if (throttle > 1000){
      throttleTimer = millis();
    }
    else{
      if (millis() - throttleTimer > 5000){
        startUltrasonic = false;
        performAutolanding = true;
      }
    }
    throttle = throttle + mappedThrottle;
  }
  if (performAutolanding) {
    throttle = throttle + mappedThrottle - 75;
    led_fast();
  }
}


float pid_p_gain_AL = 10;
float pid_i_gain_AL = 0;
float pid_d_gain_AL = 0;


float pid_i_mem_AL, pid_output_AL, pid_last_AL_d_error;
float pid_max_AL = 200;

void ultrasonicCorrectonV2() {

  if (distance > 20){
    startUltrasonic = true;
  }
  if(startUltrasonic){
    if (throttle < 1400) {
      performAutolanding = true;
    }
    else{
      performAutolanding = false;
    }
  }
  
  if (performAutolanding && distance < 50){
    // COMPUTE PIDs
    pid_error_temp = -3 - velocity;
    pid_i_mem_AL += pid_i_gain_AL * pid_error_temp;
    if(pid_i_mem_AL > pid_max_AL)pid_i_mem_AL = pid_max_AL;
    else if(pid_i_mem_AL < pid_max_AL * -1)pid_i_mem_AL = pid_max_AL * -1;
  
    pid_output_AL = pid_p_gain_AL * pid_error_temp + pid_i_mem_AL + pid_d_gain_AL * (pid_error_temp - pid_last_AL_d_error);
    if(pid_output_AL > pid_max_AL)pid_output_AL = pid_max_AL;
    else if(pid_output_AL < pid_max_AL * -1)pid_output_AL = pid_max_AL * -1;
    pid_last_AL_d_error = pid_error_temp;
    throttle = 1400 + pid_output_AL;
  }  
  else{
    pid_last_AL_d_error = 0;
    pid_output_AL = 0;
    pid_i_mem_AL = 0;
  }
}

void ultrasonicCorrectonV3() {
  
  if (distance >= 50){ // ADD START COMMAND
    startUltrasonic = true;
  }
  if (startUltrasonic){
    led_slow();
    
    if (distance < 50 && channel_3 < 1500) {
      mappedThrottle = 1500 + map(distance, 50, 0, 0, 100);
      throttle = mappedThrottle; //0.95 * throttle + 0.05 * mappedThrottle;
      
      if (channel_3 > 1000){
        throttleTimer = millis();
      }
      
      if (millis() - throttleTimer > 5000){
        startUltrasonic = false;
        performAutolanding = true;
      }
    }
  }
  if (performAutolanding){
    throttle = mappedThrottle - 50;
    led_fast();
  }
}


unsigned long timeLast;
float prevDistance;

void echoPin_trigger(){
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
        velocity = 0.95 * velocity + 0.05 * (distance - prevDistance) / (micros() - timeLast) * 1e6; 
        timeLast = micros();
      }
      pulseSent = false;
    }
  }
}
