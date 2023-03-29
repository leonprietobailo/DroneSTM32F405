void diagnostics(){
	diag_print();
	diag_flash();
}


void diag_print(){


//Serial.println(throttle);

  
	//  Serial.print(Mando_canal[1]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[2]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[3]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[4]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[5]);
//  Serial.print("\t");
//  Serial.print(Mando_canal[6]);
//  Serial.print("\n");

//  Serial.print(distance);
//  Serial.print("\t");
//  Serial.print(throttle);
//  Serial.print("\n");
//
//  Serial.print(esc_1);
//  Serial.print("\t");
//  Serial.print(esc_2);
//  Serial.print("\t");
//  Serial.print(esc_3);
//  Serial.print("\t");
//  Serial.print(esc_4);
//  Serial.print("\n");

//    Serial.print(acc_x_cal);
//    Serial.print("\t");
//    Serial.print(acc_y_cal);
//    Serial.print("\t");
//    Serial.print(acc_z_cal);
//    Serial.print("\t");
//    Serial.print(acc_x);
//    Serial.print("\t");
//    Serial.print(acc_y);
//    Serial.print("\t");
//    Serial.print(acc_z);
//    Serial.print("\t");
//    Serial.print(temperature);
//    Serial.print("\t");
//    Serial.print(gyro_pitch);
//    Serial.print("\t");
//    Serial.print(gyro_roll);
//    Serial.print("\t");
//    Serial.print(gyro_yaw);
//    Serial.print("\n");

 
  //Serial.println(hoverThrottle);

  //Serial.println(takeoff_throttle);
  //Serial.println(start);
  //Serial.println(battery_voltage);

  Serial.print(distance);
  Serial.print("\t");
  Serial.println(distanceFilt);
	
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

SPIClass SPI_FLASH(PIN_SPI3_MOSI, PIN_SPI3_MISO, PIN_SPI3_SCK, PIN_SPI3_SS);
Adafruit_FlashTransport_SPI flashTransport(PIN_SPI3_SS, &SPI_FLASH);

Adafruit_SPIFlash flash(&flashTransport);

FatVolume fatfs;
File32 myFile;





bool written = false;

void diag_flash(void){

  if(Mando_canal[5] > 1500 && !written){
    if (myFile) {
      Serial.println("Writing...");
      //myFile.println("---");
  
      for(int16_t i = 0; i < n_str; i++){
        myFile.print(i);
        myFile.print(',');
        myFile.print(voltage_str[i]);
        myFile.print(',');
        myFile.print(throttle_str[i]);
        myFile.print('\n');
      }
      Serial.println("Done!");
      //myFile.println("***");    
      myFile.close();
      written = true;
      n_str = 0;
      memset(voltage_str, 0, sizeof(length_str));
      memset(throttle_str, 0, sizeof(length_str));
    }
    else {
      Serial.println("error opening data.csv");
    }
  }

  if (Mando_canal[5] < 1500 && written) {
    written = false;
    myFile = fatfs.open("data.csv", FILE_WRITE);
  }

}

void diag_buzzer(void){
	
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
