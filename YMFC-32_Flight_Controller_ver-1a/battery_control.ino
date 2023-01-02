
float batteryVoltage;
float timer;
int buzzerTimer;
bool toneOn;

void battery_control(void) {
	batteryVoltage = (float)analogRead(pin_BAT) / 112.81;
	buzzer();
}

void buzzer(void){
	
	if (batteryVoltage <= 10.5){					// If battery below 10.5 V -> Permanent Buzzer Sound
		tone(pin_BUZZER, 2500);
	}
	else if (batteryVoltage <= 11){					// If battery below 11 V -> Buzzer sounds twice per second
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