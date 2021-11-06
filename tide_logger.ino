
#include "SD.h"
#include "RTClib.h"

RTC_PCF8523 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

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

// the digital pins that connect to the LEDs
#define redLEDpin 3
#define greenLEDpin 4

#define IO_USERNAME  "cmurtaugh"
#define IO_KEY       "aio_XvXc79nKkznCs8bkC423b9AAAnZm"

#define CONTROL_PIN 5   // This is the YELLOW wire, can be any data line

int16_t distance;  // The last measured distance
bool newData = false; // Whether new data is available from the sensor
uint8_t buffer[4];  // our buffer for storing data
uint8_t idx = 0;  // our idx into the storage buffer
File logfile;
const int chipSelect = 10;

#define SYNC_INTERVAL 10000  // mills between calls to flush() - to write data to the card
#define LOG_INTERVAL 5000
//const int SYNC_INTERVAL = 1000 * 60 * 5; // mills between calls to flush() - to write data to the card
//const int LOG_INTERVAL = 1000 * 60;

uint32_t syncTime = 0; // time of last sync()

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  
  while(1);
}


int16_t getDistance() {
//  int16_t distance;  // The last measured distance
//  bool newData = false; // Whether new data is available from the sensor
//  uint8_t buffer[4];  // our buffer for storing data
//  uint8_t idx = 0;  // our idx into the storage buffer
  uint8_t samples = 0;
  
  while(samples < 5) {
    if (mySerial.available()) {
      uint8_t c = mySerial.read();
      Serial.println(c, HEX);
  
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
          samples++;
          Serial.print("sample ");
          Serial.print(samples);
          Serial.print(" -> ");
          Serial.println(distance);
        }
        idx = 0;
      }
    }    
  }
  return distance;
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Adafruit DYP-ME007YS Test");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
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
  pinMode(chipSelect, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // create a new file
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
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);  
}


void loop() { // run over and over
  int delayMsec = ((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  Serial.print("Will delay for: ");
  Serial.println(delayMsec);
  delay(delayMsec);
/*
  if (mySerial.available()) {
    uint8_t c = mySerial.read();
    //Serial.println(c, HEX);

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
  
  if (newData) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    newData = false;
  }
*/
  int16_t lastDist = getDistance();
  DateTime now = rtc.now();
  
//  Serial.print(now.year(), DEC);
//  Serial.print('/');
//  Serial.print(now.month(), DEC);
//  Serial.print('/');
//  Serial.print(now.day(), DEC);
//  Serial.print(" (");
//  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
//  Serial.print(") ");
//  Serial.print(now.hour(), DEC);
//  Serial.print(':');
//  Serial.print(now.minute(), DEC);
//  Serial.print(':');
//  Serial.print(now.second(), DEC);
  Serial.print(now.unixtime());  
  Serial.print(",");
  Serial.print(now.timestamp());
  Serial.print(",");
  Serial.print(lastDist);
  Serial.println(" mm");  

//  logfile.print(now.year(), DEC);
//  logfile.print('/');
//  logfile.print(now.month(), DEC);
//  logfile.print('/');
//  logfile.print(now.day(), DEC);
//  logfile.print(" (");
//  logfile.print(daysOfTheWeek[now.dayOfTheWeek()]);
//  logfile.print(") ");
//  logfile.print(now.hour(), DEC);
//  logfile.print(':');
//  logfile.print(now.minute(), DEC);
//  logfile.print(':');
//  logfile.print(now.second(), DEC);

  logfile.print(now.unixtime());  
  logfile.print(",");
  logfile.print(now.timestamp());  
  logfile.print(",");
  logfile.println(lastDist);


  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  Serial.println("Flushing to disk!");
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
}
