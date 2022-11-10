#define pin_INT_Throttle PA6 // Pin Throttle del mando RC
#define pin_INT_Yaw PA7      // Pin Yaw del mando RC  
#define pin_INT_Pitch PA5    // Pin Pitch del mando RC 
#define pin_INT_Roll PA4     // Pin Roll del mando RC  

int16_t showResults = 0; 

uint16_t highThrottle = 0;
uint16_t highYaw = 0;
uint16_t highPitch = 0;
uint16_t highRoll = 0;

uint16_t lowThrottle = 10000;
uint16_t lowYaw = 10000;
uint16_t lowPitch = 10000;
uint16_t lowRoll = 10000;

uint8_t data;
int32_t channel_1, channel_2, channel_3, channel_4;

void setup() {
  pinMode(pin_INT_Pitch, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Pitch), handler_channel_2, CHANGE);
  pinMode(pin_INT_Yaw, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Yaw), handler_channel_4, CHANGE);
  pinMode(pin_INT_Throttle, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Throttle), handler_channel_3, CHANGE);
  pinMode(pin_INT_Roll, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Roll), handler_channel_1, CHANGE);
  Serial.begin(57600);
}

void loop() {
//  data = Serial.read();
//  if (data == 'q'){
    Serial.println("Roll     - From: " + String(lowRoll) + " To: " + String(highRoll));
    Serial.println("Pitch    - From: " + String(lowPitch) + " To: " + String(highPitch));
    Serial.println("Throttle - From: " + String(lowThrottle) + " To: " + String(highThrottle));
    Serial.println("Yaw      - From: " + String(lowYaw) + " To: " + String(highYaw));
  //}

//  Serial.print(channel_1);
//  Serial.print("\t");
//  Serial.print(channel_2);
//  Serial.print("\t");
//  Serial.print(channel_3);
//  Serial.print("\t");
//  Serial.print(channel_4);
//  Serial.print("\n");


  
  if (channel_1 > 500 && channel_1 < 2500){
    if(channel_1 > highRoll){
      highRoll = channel_1;
    }
    else if (channel_1 < lowRoll){
      lowRoll = channel_1;
    }
  }

  if (channel_2 > 500 && channel_2 < 2500){
    if(channel_2 > highPitch){
      highPitch = channel_2;
    }
    else if (channel_2 < lowPitch){
      lowPitch = channel_2;
    }
  }
  
  if (channel_3 > 500 && channel_3 < 2500){
    if(channel_3 > highThrottle){
      highThrottle = channel_3;
    }
    else if (channel_3 < lowThrottle){
      lowThrottle = channel_3;
    }
  }

  if (channel_4 > 500 && channel_4 < 2500){
    if(channel_4 > highYaw){
      highYaw = channel_4;
    }
    else if (channel_4 < lowYaw){
      lowYaw = channel_4;
    }
  }

}


// INTERRUPCIÓN MANDO RC – > ROLL
volatile long Roll_HIGH_us;
volatile int RC_Roll_raw;

void handler_channel_1() {
  if (digitalRead(pin_INT_Roll) == HIGH) Roll_HIGH_us = micros();
  if (digitalRead(pin_INT_Roll) == LOW)  RC_Roll_raw  = micros() - Roll_HIGH_us;
  channel_1 = RC_Roll_raw;
}

// INTERRUPCIÓN MANDO RC – > PITCH
volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;
void handler_channel_2() {
  if (digitalRead(pin_INT_Pitch) == HIGH) Pitch_HIGH_us = micros();
  if (digitalRead(pin_INT_Pitch) == LOW)  RC_Pitch_raw  = micros() - Pitch_HIGH_us;
  channel_2 = RC_Pitch_raw;
}

// INTERRUPCIÓN MANDO RC – > THROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void handler_channel_3() {
  if (digitalRead(pin_INT_Throttle) == HIGH) Throttle_HIGH_us = micros();
  if (digitalRead(pin_INT_Throttle) == LOW)  RC_Throttle_raw  = micros() - Throttle_HIGH_us;
  channel_3 = RC_Throttle_raw;
}

// INTERRUPCIÓN MANDO RC – > YAW
volatile long Yaw_HIGH_us;
volatile int RC_Yaw_raw;
void handler_channel_4() {
  if (digitalRead(pin_INT_Yaw) == HIGH) Yaw_HIGH_us = micros();
  if (digitalRead(pin_INT_Yaw) == LOW)  RC_Yaw_raw  = micros() - Yaw_HIGH_us;
  channel_4 = RC_Yaw_raw;
}
