/* Attempt to read GPS AND temperature info and transmit over radio
  Based on: DeviceExample, modified version using NeoGPS: https://stackoverflow.com/questions/43006844/want-to-get-gps-data-at-every-5-sec-using-arduino/43013240,
  TT7_1_60, OneWire DS18S20, DS18B20, DS1822 Temperature Example AND NTX2 Radio Test Part 2, from https://ukhas.org.uk/guides:linkingarduinotontx2

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
#define tempPin 10  // on pin 10 (a 4.7K resistor is necessary)

#include <string.h>
#include <util/crc16.h>
#include <OneWire.h>

//char datastring[80];

OneWire  ds(tempPin);

byte j;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];
float celsius;

char ntx2buffer[80];
int count = 0;
uint8_t errorCount = 0; //needed later?

void setup() {
  pinMode(RADIOPIN, OUTPUT);
  gpsPort.begin(GPSBaud);
  Serial.begin(9600);
}

void loop() {

  // TEMP
  if ( !ds.search(addr)) {
    // No more addresses
    ds.reset_search();
    delay(250);
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    return;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //   Chip = DS18S20 or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //   Chip = DS18B20
      type_s = 0;
      break;
    case 0x22:
      //   Chip = DS1822
      type_s = 0;
      break;
    default:
      //   Device is not a DS18x20 family device
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, regular power

  // delay(1000);     // maybe 750ms is enough, maybe not  //***GET RID OF THIS?***
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( j = 0; j < 9; j++) {           // we need 9 bytes
    data[j] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  //Serial.print("  Temperature = "); // debug***
  //Serial.print(celsius); // debug***
  //Serial.println(" Celsius, "); // debug***
  // END TEMP


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
    sprintf(ntx2buffer, "%s,%d,", ntx2buffer, fix.satellites);
    dtostrf(celsius, 5, 2, &ntx2buffer[strlen(ntx2buffer)]); // for floating point.  up to 3 digits before dec point, 2 after.
    sprintf(ntx2buffer, "%s*%04X\n", ntx2buffer, gps_CRC16_checksum(ntx2buffer));

    Serial.println(ntx2buffer); // debug***
    // also, need debug failed checksums and soft serial device overflow, as with tinygpsplus

    //TRANSMIT
    //rtty_txstring(ntx2buffer);

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

