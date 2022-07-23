//
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
//More information can be found in these two videos:
//STM32 for Arduino - Connecting an RC receiver via input capture mode: https://youtu.be/JFSFbSg0l2M
//STM32 for Arduino - Electronic Speed Controller (ESC) - STM32F103C8T6: https://youtu.be/Nju9rvZOjVQ
//
void timer_setup(void) {
  Timer2.attachCompare1Interrupt(handler_channel_1);
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->CR2 = 0;
  TIM2->SMCR = 0;
  TIM2->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
  TIM2->EGR = 0;
  TIM2->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIM2->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM2->PSC = 71;
  TIM2->ARR = 0xFFFF;
  TIM2->DCR = 0;

  Timer3.attachCompare1Interrupt(handler_channel_5);
  Timer3.attachCompare2Interrupt(handler_channel_6);
  TIM3->CR1 = TIM_CR1_CEN;
  TIM3->CR2 = 0;
  TIM3->SMCR = 0;
  TIM3->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE;
  TIM3->EGR = 0;
  TIM3->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIM3->CCMR2 = 0;
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM3->PSC = 71;
  TIM3->ARR = 0xFFFF;
  TIM3->DCR = 0;

//A test is needed to check if the throttle input is active and valid. Otherwise the ESC's might start without any warning.
  loop_counter = 0;
  while ((channel_3 > 2100 || channel_3 < 900) && warning == 0) {
    delay(100);
    loop_counter++;
    if (loop_counter == 40) {
      Serial.println(F("Waiting for a valid receiver channel-3 input signal"));
      Serial.println(F(""));
      Serial.println(F("The input pulse should be between 1000 till 2000us"));
      Serial.print(F("Current channel-3 receiver input value = "));
      Serial.println(channel_3);
      Serial.println(F(""));
      Serial.println(F("Is the receiver connected and the transmitter on?"));
      Serial.println(F("For more support and questions: www.brokking.net"));
      Serial.println(F(""));
      Serial.print(F("Waiting for another 5 seconds."));
    }
    if (loop_counter > 40 && loop_counter % 10 == 0)Serial.print(F("."));

      if (loop_counter == 90) {
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("The ESC outputs are disabled for safety!!!"));
      warning = 1;
    }
  }
  if (warning == 0) {
    TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    TIM4->CR2 = 0;
    TIM4->SMCR = 0;
    TIM4->DIER = 0;
    TIM4->EGR = 0;
    TIM4->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE |(0b110 << 12) | TIM_CCMR1_OC2PE;
    TIM4->CCMR2 = (0b110 << 4) | TIM_CCMR2_OC3PE |(0b110 << 12) | TIM_CCMR2_OC4PE;
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM4->PSC = 71;
    TIM4->ARR = 4000;
    TIM4->DCR = 0;
    TIM4->CCR1 = 1000;

    TIM4->CCR1 = channel_3;
    TIM4->CCR2 = channel_3;
    TIM4->CCR3 = channel_3;
    TIM4->CCR4 = channel_3;
    pinMode(PB6, PWM);
    pinMode(PB7, PWM);
    pinMode(PB8, PWM);
    pinMode(PB9, PWM);
  }
}
