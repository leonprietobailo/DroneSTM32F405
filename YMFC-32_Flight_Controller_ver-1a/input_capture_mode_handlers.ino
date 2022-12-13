//void handler_channel_1(void) {                           //This function is called when channel 1 is captured.
//  if (0b1 & GPIOA->IDR  >> 0) {                     //If the receiver channel 1 input pulse on A0 is high.
//    channel_1_start = TIM2->CCR1;                 //Record the start time of the pulse.
//    TIM2->CCER |= TIM_CCER_CC1P;                //Change the input capture mode to the falling edge of the pulse.
//  }
//  else {                                                 //If the receiver channel 1 input pulse on A0 is low.
//    channel_1 = TIM2->CCR1 - channel_1_start;     //Calculate the total pulse time.
//    if (channel_1 < 0)channel_1 += 0xFFFF;               //If the timer has rolled over a correction is needed.
//    TIM2->CCER &= ~TIM_CCER_CC1P;               //Change the input capture mode to the rising edge of the pulse.
//  }
//  Serial.println("Hola");
//}
//
//void handler_channel_2(void) {                           //This function is called when channel 2 is captured.
//  if (0b1 & GPIOA->IDR >> 1) {                      //If the receiver channel 2 input pulse on A1 is high.
//    channel_2_start = TIM2->CCR2;                 //Record the start time of the pulse.
//    TIM2->CCER |= TIM_CCER_CC2P;                //Change the input capture mode to the falling edge of the pulse.
//  }
//  else {                                                 //If the receiver channel 2 input pulse on A1 is low.
//    channel_2 = TIM2->CCR2 - channel_2_start;     //Calculate the total pulse time.
//    if (channel_2 < 0)channel_2 += 0xFFFF;               //If the timer has rolled over a correction is needed.
//    TIM2->CCER &= ~TIM_CCER_CC2P;               //Change the input capture mode to the rising edge of the pulse.
//  }
//  Serial.println("Hola");
//}
//
//void handler_channel_3(void) {                           //This function is called when channel 3 is captured.
//  if (0b1 & GPIOA->IDR >> 2) {                      //If the receiver channel 3 input pulse on A2 is high.
//    channel_3_start = TIM2->CCR3;                 //Record the start time of the pulse.
//    TIM2->CCER |= TIM_CCER_CC3P;                //Change the input capture mode to the falling edge of the pulse.
//  }
//  else {                                                 //If the receiver channel 3 input pulse on A2 is low.
//    channel_3 = TIM2->CCR3 - channel_3_start;     //Calculate the total pulse time.
//    if (channel_3 < 0)channel_3 += 0xFFFF;               //If the timer has rolled over a correction is needed.
//    TIM2->CCER &= ~TIM_CCER_CC3P;               //Change the input capture mode to the rising edge of the pulse.
//  }
//  Serial.println("Hola");
//}
//
//void handler_channel_4(void) {                           //This function is called when channel 4 is captured.
//  if (0b1 & GPIOA->IDR >> 3) {                      //If the receiver channel 4 input pulse on A3 is high.
//    channel_4_start = TIM2->CCR4;                 //Record the start time of the pulse.
//    TIM2->CCER |= TIM_CCER_CC4P;                //Change the input capture mode to the falling edge of the pulse.
//  }
//  else {                                                 //If the receiver channel 4 input pulse on A3 is low.
//    channel_4 = TIM2->CCR4 - channel_4_start;     //Calculate the total pulse time.
//    if (channel_4 < 0)channel_4 += 0xFFFF;               //If the timer has rolled over a correction is needed.
//    TIM2->CCER &= ~TIM_CCER_CC4P;               //Change the input capture mode to the rising edge of the pulse.
//  }
//  Serial.println("Hola");
//}
//
//void handler_channel_5(void) {                           //This function is called when channel 5 is captured.
//  if (0b1 & GPIOA->IDR >> 6) {                      //If the receiver channel 5 input pulse on A6 is high.
//    channel_5_start = TIM3->CCR1;                 //Record the start time of the pulse.
//    TIM3->CCER |= TIM_CCER_CC1P;                //Change the input capture mode to the falling edge of the pulse.
//  }
//  else {                                                 //If the receiver channel 5 input pulse on A6 is low.
//    channel_5 = TIM3->CCR1 - channel_5_start;     //Calculate the total pulse time.
//    if (channel_5 < 0)channel_5 += 0xFFFF;               //If the timer has rolled over a correction is needed.
//    TIM3->CCER &= ~TIM_CCER_CC1P;               //Change the input capture mode to the rising edge of the pulse.
//  }
//}
//
//void handler_channel_6(void) {                           //This function is called when channel 6 is captured.
//  if (0b1 & GPIOA->IDR >> 7) {                      //If the receiver channel 6 input pulse on A7 is high.
//    channel_6_start = TIM3->CCR2;                 //Record the start time of the pulse.
//    TIM3->CCER |= TIM_CCER_CC2P;                //Change the input capture mode to the falling edge of the pulse.
//  }
//  else {                                                 //If the receiver channel 6 input pulse on A7 is low.
//    channel_6 = TIM3->CCR2 - channel_6_start;     //Calculate the total pulse time.
//    if (channel_6 < 0)channel_6 += 0xFFFF;               //If the timer has rolled over a correction is needed.
//    TIM3->CCER &= ~TIM_CCER_CC2P;               //Change the input capture mode to the rising edge of the pulse.
//  }
//}

// long loop_timer, tiempo_ejecucion;
//float RC_Throttle_consigna, RC_Pitch_consigna, RC_Roll_consigna, RC_Yaw_consigna;

// AJUSTE MANDO RC - THROTLLE
const int us_max_Throttle_adj = 50;
const int us_min_Throttle_adj = -50;
const float us_max_Throttle_raw = 2004; // < – Si teneis la entrada Throttle invertida sustituid este valor
const float us_min_Throttle_raw = 1116; // < – por este y viceversa

// AJUSTE MANDO RC - PITCH
const float us_max_Pitch_raw = 1952;
const float us_min_Pitch_raw = 992;
const int us_max_Pitch_adj = -30;   // < – Si teneis la entrada Pitch invertido sustituid este valor
const int us_min_Pitch_adj = 30;    // < – por este y viceversa

// AJUSTE MANDO RC - ROLL
const float us_max_Roll_raw = 1960;
const float us_min_Roll_raw = 992;
const int us_max_Roll_adj = 30;     // < – Si teneis la entrada Roll invertido sustituid este valor
const int us_min_Roll_adj = -30;    // < – por este y viceversa

// AJUSTE MANDO RC - YAW
const float us_max_Yaw_raw = 1928;
const float us_min_Yaw_raw = 972;
const int us_max_Yaw_adj = 30;      // < – Si teneis la entrada Yaw invertido sustituid este valor
const int us_min_Yaw_adj = -30;     // < – por este y viceversa



// INTERRUPCIÓN MANDO RC – > PITCH
volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;
void handler_channel_2() {
  if (digitalRead(pin_INT_Pitch) == HIGH) Pitch_HIGH_us = micros();
  if (digitalRead(pin_INT_Pitch) == LOW)  RC_Pitch_raw  = micros() - Pitch_HIGH_us;
  channel_2 = map(RC_Pitch_raw, pitch_low, pitch_high, 2000, 1000);

}

// INTERRUPCIÓN MANDO RC – > YAW
volatile long Yaw_HIGH_us;
volatile int RC_Yaw_raw;
void handler_channel_4() {
  if (digitalRead(pin_INT_Yaw) == HIGH) Yaw_HIGH_us = micros();
  if (digitalRead(pin_INT_Yaw) == LOW)  RC_Yaw_raw  = micros() - Yaw_HIGH_us;
  channel_4 = map(RC_Yaw_raw, yaw_low, yaw_high, 1000, 2000);
}

// INTERRUPCIÓN MANDO RC – > THROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void handler_channel_3() {
  if (digitalRead(pin_INT_Throttle) == HIGH) Throttle_HIGH_us = micros();
  if (digitalRead(pin_INT_Throttle) == LOW)  RC_Throttle_raw  = micros() - Throttle_HIGH_us;
  channel_3 = map(RC_Throttle_raw, throttle_low, throttle_high, 2000, 1000);

}

// INTERRUPCIÓN MANDO RC – > ROLL
volatile long Roll_HIGH_us;
volatile int RC_Roll_raw;
void handler_channel_1() {
  if (digitalRead(pin_INT_Roll) == HIGH) Roll_HIGH_us = micros();
  if (digitalRead(pin_INT_Roll) == LOW)  RC_Roll_raw  = micros() - Roll_HIGH_us;
  channel_1 = map(RC_Roll_raw, roll_low, roll_high, 1000, 2000);

}
