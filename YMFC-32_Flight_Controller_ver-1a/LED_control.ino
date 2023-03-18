///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These functions handle the red and green LEDs. The LEDs on the flip 32 are inverted. That is why a Flip32 test is needed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// void red_led(int8_t level) {
  // digitalWrite(PC1, level);
// }


volatile long led_timer;

void led_setup(){
  pinMode(PC1, OUTPUT);
}

// LED ON
void led_on() {
	digitalWrite(PC1, HIGH);
}

// 4 LED BLINKS / SEC
void led_fast() {
	if ((digitalRead(PC1) == LOW) && (millis() - led_timer > 250)) {
		digitalWrite(PC1, HIGH);
		led_timer = millis();
	}
	else if ((digitalRead(PC1) == HIGH) && (millis() - led_timer > 250)) {
		digitalWrite(PC1, LOW);
		led_timer = millis();
	}
	
}

// 1 LED BLINK / SEC
void led_slow() {
	if ((digitalRead(PC1) == LOW) && (millis() - led_timer > 1000)) {
		digitalWrite(PC1, HIGH);
		led_timer = millis();
	}
	else if ((digitalRead(PC1) == HIGH) && (millis() - led_timer > 1000)) {
		digitalWrite(PC1, LOW);
		led_timer = millis();
	}
	
}

// LED OFF
void led_off() {
	digitalWrite(PC1, LOW);
}
