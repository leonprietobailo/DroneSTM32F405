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


void flash_setup(void){
  // Init external flash
  flash.begin();

  // Open file system on the flash
  if ( !fatfs.begin(&flash) ) {
    Serial.println("Error: filesystem is not existed. Please try SdFat_format example to make one.");
    while(1)
    {
      yield();
      delay(1);
    }
  }
 myFile = fatfs.open("data.csv", FILE_WRITE);
 Serial.println("initialization done.");

}


bool written = false;

void flash_write(void){

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
