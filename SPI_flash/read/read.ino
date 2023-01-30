/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

// for flashTransport definition
SPIClass SPI_FLASH(PIN_SPI3_MOSI, PIN_SPI3_MISO, PIN_SPI3_SCK, PIN_SPI3_SS);
Adafruit_FlashTransport_SPI flashTransport(PIN_SPI3_SS, &SPI_FLASH);

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;
File32 myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Initializing Filesystem on external flash...");
  
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

  Serial.println("initialization done.");
  
  // open the file for reading:
  myFile = fatfs.open("datalog.txt");
  if (myFile) {
    Serial.println("datalog.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening datalog.txt");
  }
}

void loop() {
}
