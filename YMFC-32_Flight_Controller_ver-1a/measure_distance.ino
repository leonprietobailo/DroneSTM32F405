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

void echoPin_trigger(){
  if(pulseSent){
    if (digitalRead(echoPin) == HIGH){
      pulseStart = micros();
    }
    else{
      pulseEnd = micros();
      duration = pulseEnd - pulseStart;
      computedDistance = duration / 1e6 * cAir * 1e2 / 2.0;     // distance = duration [us] / 1e6 [us/s] * speedOfSound [m/s] * 1e2 [cm/m] / 2 (go and back] || [cm]
//      if (computedDistance > 120){
//        computedDistance = 120;
//      }
      if (computedDistance < 10){
        computedDistance = 10;
      }
      distance = 0.95 * distance + 0.05 * computedDistance;
      pulseSent = false;
    }
  }
}
