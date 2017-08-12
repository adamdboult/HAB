/* Attempt to read GPS info and transmit over radio
  Based on: DeviceExample, modified version using NeoGPS: https://stackoverflow.com/questions/43006844/want-to-get-gps-data-at-every-5-sec-using-arduino/43013240, TT7_1_60 AND NTX2 Radio Test Part 2, from https://ukhas.org.uk/guides:linkingarduinotontx2
  Consider also: "The best software serial port library is AltSoftSerial. If you can switch to pins 8 & 9, I would strongly recommend doing so."
*/

// GPS
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
NeoSWSerial gpsPort(RXPin, TXPin); // The serial connection to the GPS device
NMEAGPS gps;
gps_fix fix;
uint8_t fixCount = 0;
// GPS END

#define RADIOPIN 13

#include <string.h> //needed?
#include <util/crc16.h>

//char datastring[80];
char ntx2buffer[80];
int count = 0;
uint8_t errorCount = 0; //needed later?

void setup() {
  pinMode(RADIOPIN, OUTPUT);
  gpsPort.begin(GPSBaud);
  Serial.begin(9600);
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
  //while (ss.available() > 0)
  //if (gps.encode(ss.read()))
  // {

  while (gps.available( gpsPort )) {
    fix = gps.read();

    count++;// or += ?

    // Shift the date/time to local time
    NeoGPS::clock_t localSeconds;
    NeoGPS::time_t  localTime;
    if (fix.valid.date && fix.valid.time) {
      using namespace NeoGPS; // save a little typing below...

      localSeconds = (clock_t) fix.dateTime; // convert structure to a second count
      //localSeconds += 1 * SECONDS_PER_HOUR + 0 * SECONDS_PER_MINUTE; // UK Summer Time!
      localTime = localSeconds;              // convert back to a structure
    }

    //CONSTRUCT STRING
    //Serial.println(gps.altitude.kilometers()); // debug***
    //Serial.println(gps.altitude.meters()); //debug***
    sprintf(ntx2buffer, "$$XL5,%d,%02d:%02d:%02d,", count, localTime.hours, localTime.minutes, localTime.seconds);
    //Serial.println(ntx2buffer); // debug***
    dtostrf(fix.latitude(), 8, 5, &ntx2buffer[strlen(ntx2buffer)]);
    strcat(ntx2buffer, ",");
    dtostrf(fix.longitude(), 8, 5, &ntx2buffer[strlen(ntx2buffer)]);
    strcat(ntx2buffer, ",");
    dtostrf(fix.altitude(), 5, 2, &ntx2buffer[strlen(ntx2buffer)]);
    sprintf(ntx2buffer, "%s,%d", ntx2buffer, fix.satellites);
    sprintf(ntx2buffer, "%s*%04X\n", ntx2buffer, gps_CRC16_checksum(ntx2buffer));

    Serial.println(ntx2buffer); // debug***
    // also, need debug failed checksums and soft serial device overflow, as with tinygpsplus

    //TRANSMIT
    rtty_txstring(ntx2buffer);
   
  }

  if ((gps.statistics.chars < 10) && (millis() > 5000)) {
    Serial.println( F("No GPS detected: check wiring.") );
    while (true);
  }

}

void rtty_txstring (char * string)
{

  /* Simple function to sent a char at a time to
  ** rtty_txbyte function.
  ** NB Each char is one byte (8 Bits)
  */

  char c;

  c = *string++;

  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}


void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to
  ** rtty_txbit function.
  ** NB The bits are sent Least Significant Bit first
  **
  ** All chars should be preceded with a 0 and
  ** proceded with a 1. 0 = Start bit; 1 = Stop bit
  **
  */

  int i;

  rtty_txbit (0); // Start bit

  // Send bits for for char LSB first

  for (i = 0; i < 7; i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1);

    else rtty_txbit(0);

    c = c >> 1;

  }

  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    digitalWrite(RADIOPIN, HIGH);
  }
  else
  {
    // low
    digitalWrite(RADIOPIN, LOW);

  }

  //                  delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
  delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
  // largest value that will produce an accurate delay is 16383
  // See : http://arduino.cc/en/Reference/DelayMicroseconds

}

uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

