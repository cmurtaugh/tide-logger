#include "SD.h"
#include "RTClib.h"

RTC_PCF8523 rtc;

// Example code for DYP-ME007YS sensor

#if defined(__AVR__) || defined(ESP8266)
// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (WHITE wire)
// Set up the serial port to use softwareserial..
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, -1);

#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is white wire, data input
#define mySerial Serial1
#endif

#define PRINT_TO_SERIAL 0

#define CONTROL_PIN 5   // This is the YELLOW wire, can be any data line
#define CHIP_SELECT 10 
#define SYNC_INTERVAL 10000  // mills between calls to flush() - to write data to the card
#define LOG_INTERVAL 5000

int16_t distance;  // The last measured distance
bool newData = false; // Whether new data is available from the sensor
uint8_t buffer[4];  // our buffer for storing data
uint8_t idx = 0;  // our idx into the storage buffer
uint32_t syncTime = 0; // time of last sync()
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  while(1);
}

void setup() {
  
#if PRINT_TO_SERIAL
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }
#endif 

  if (! rtc.begin()) {
#if PRINT_TO_SERIAL    
    Serial.println("Couldn't find RTC");
    Serial.flush();
#endif
    abort();
  }
  rtc.start();

  // set the data rate for the Serial port, 9600 for the sensor
  mySerial.begin(9600);
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, HIGH);
  
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(CHIP_SELECT, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(CHIP_SELECT)) {
#if PRINT_TO_SERIAL        
    Serial.println("Card failed, or not present");
#endif
    // don't do anything more:
    return;
  }
#if PRINT_TO_SERIAL      
  Serial.println("card initialized.");
#endif

  // create a new data file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    error("couldn't create file");
  }

#if PRINT_TO_SERIAL    
  Serial.print("Logging to: ");
  Serial.println(filename);
#endif
  
}


void loop() { // run over and over

  if (mySerial.available()) {
    uint8_t c = mySerial.read();

    // See if this is a header byte
    if (idx == 0 && c == 0xFF) {
      buffer[idx++] = c;
    }
    // Two middle bytes can be anything
    else if ((idx == 1) || (idx == 2)) {
      buffer[idx++] = c;
    }
    else if (idx == 3) {
      uint8_t sum = 0;
      sum = buffer[0] + buffer[1] + buffer[2];
      if (sum == c) {
        distance = ((uint16_t)buffer[1] << 8) | buffer[2];
        newData = true;
      }
      idx = 0;
    }
  }

  // when we have a measurement:
  if (newData) {
    DateTime now = rtc.now();
  
 #if PRINT_TO_SERIAL    
   Serial.print(now.unixtime());
    Serial.print(",");
    Serial.print(now.timestamp());
    Serial.print(",");
    Serial.print(distance);
    Serial.println(" mm");
#endif
    
    newData = false;

    // if it's not time to write to the log yet, just keep looping
    if ((millis() - syncTime) < SYNC_INTERVAL) return;

    // otherwise, write to the log! 
    syncTime = millis();
  
#if PRINT_TO_SERIAL    
    Serial.println("Writing to log!");
#endif
    logfile.print(now.unixtime());
    logfile.print(",");
    logfile.print(now.timestamp());
    logfile.print(",");
    logfile.println(distance);
    
#if PRINT_TO_SERIAL    
    Serial.println("Flushing to disk!");
#endif
    logfile.flush();
    
  }

}
