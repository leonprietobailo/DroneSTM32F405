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

void echoPin_trigger(){
  if(pulseSent){
    if (digitalRead(echoPin) == HIGH){
      pulseStart = micros();
    }
    else{
      pulseEnd = micros();
      duration = pulseEnd - pulseStart;
      computedDistance = duration / 1e6 * cAir * 1e2 / 2.0;     // distance = duration [us] / 1e6 [us/s] * speedOfSound [m/s] * 1e2 [cm/m] / 2 (go and back] || [cm]
      if (computedDistance < distance + 30 && computedDistance > distance - 30){
        distance = 0.9 * distance + 0.1 * computedDistance;
      }
      pulseSent = false;
      Serial.println(distance);
    }
  }
}
