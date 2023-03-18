
float batteryVoltage;
float timer;
int buzzerTimer;
bool toneOn;

void battery_control(void) {
	//The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA7) / 352.27*0.9838);

  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0)error = 1;
}

void buzzer(void){
	
	if (battery_voltage <= 10.5){					// If battery below 10.5 V -> Permanent Buzzer Sound
		tone(pin_BUZZER, 2500);
	}
	else if (battery_voltage <= 11.2){					// If battery below 11 V -> Buzzer sounds twice per second
		if (micros() - buzzerTimer > 0.25e6){		// If quarter of a second passed (25000 us), change status of buzzer.
			if (toneOn){
				noTone(pin_BUZZER);
				toneOn = false;
			}
			else{
				tone(pin_BUZZER, 2500);
				toneOn = true;
			}
			buzzerTimer = micros();
		}
	}
	else{											// If battery is high, stop all tones
		noTone(pin_BUZZER);
	}
}
