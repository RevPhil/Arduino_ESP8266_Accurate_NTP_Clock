/*
   A.N.T. Accurate Ntp Time

   A Proof Of Concept attempt to get accurate(ish) time from an NTP server. The result gives the time within
   +-10 milliseconds of the time as referenced to the DCF77 time signal from Frankfurt, Germany.
   NTP request round-trip should be in the range 12 - 20 mS to achieve the accuracy stated above!

   In use the actual discrepancy can be as low as +- 1mS. +-5mS to +-10 mS is typical.

   Change the "const char *ntpServer = "time.windows.com";"  NTP server entries in the NTP Client function
   to time servers close to your location.

   The DS3231 (or similar) RTC is used to generate a 1 Pulse Per Second output to increment the sysClock variable.
   The 1PPS signal is really a 1Hx square wave but we only use the falling edge so it's 1PPS to us.
   The RTC 1PPS output is set when the seconds are written to the RTC so we can control when the interrupt occurs.
   The FALLING edge of the 1PPS output calls the rtcIntISR() sub-routine which increments sysClock and sets
   outputTimestampEnable which in turn triggers the display of the current time.

   The NTP server provides the current number of seconds since 1900 and the number of picoseconds since the last
   time increment so we convert the picoseconds fraction to milliseconds and use the result to calculate the RTC
   setting time delay. RTC setting is delayed for 1 second - the fraction + the NTP round-trip / 2. The NTP seconds are
   altered to UNIX seconds (seconds since 1970) and TimeLib is used to create the time values. sysClock is always UTC.

   Synchronisation using microseconds has been tried but it made little difference to the end result and added
   complications, especially when used on an ESP8266 which seems to have problems with micros() in Interrupt routines.

   The Timezone library is used to create the current local time as defined by the timezone configuration below.

   All testing was done using a Saleae Logic Analyser and a DCF77 receiver. As you don't all have these tools to hand
   you could try or a pretty accurate time comparison such as www.time.is

   The output to the Serial Monitor is shown below. When the NTP time is received, basic time data is also
   displayed along with NTP error messages. The primary display is designed to be a MAX7219 8 x 7 Segment module. The
   code can be modified, of course, to use any display you choose. It's up to you...

    10:46:18 Fri 09-Feb-2018
    10:46:19 Fri 09-Feb-2018
    10:46:20 Fri 09-Feb-2018
    10:46:21 Fri 09-Feb-2018
    10:46:22 Fri 09-Feb-2018 * NTP Fraction: 198 mS - NTP Round-trip: 14 mS
    10:46:23 Fri 09-Feb-2018
    10:46:24 Fri 09-Feb-2018
    10:46:25 Fri 09-Feb-2018
    10:46:26 Fri 09-Feb-2018

   A simple class is provided to communicate with the MAX7219 8 x 7 Segment LED display using Bit-Bang Serial

   ALL ESP8266 PIN NUMBERS ARE THE GPIO NUMBERS!

   (C) Copyright Reverend Phil Morris

   No warranty or guarantee either actual or implied as to the accuracy or efficacy of this software is provided.
   You may copy, modify and distribute this software for personal use as long as this text header remains intact!

   ANY AND ALL COMMERCIAL USE IS PROHIBITED!
*/

#define DEBUG true      // change to false to stop Serial output if you use a display such as LED or LCD

#include <TimeLib.h>    // https://github.com/PaulStoffregen/Time
#include <Timezone.h>   // https://github.com/JChristensen/Timezone
#include <DS3232RTC.h>  // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>  // https://github.com/geneReeves/ArduinoStreaming

/* ALTERNATIVE DISPLAY */
// choose LCD, LED or neither alternative display
//#define LCD_DISPLAY // 16 x 2 HD44780 LCD Display with I2C adapter
//#define LED_DISPLAY // 8 x 7 Segment LED Display with MAX7219 Controller

/* 16 x 2 LCD Display on I2C */
#ifdef LCD_DISPLAY
#include <LiquidCrystal_I2C.h>

// lcd(I2C,EN,RW,RS,D4,D5,D6,D7,BACKLIGHT_PIN,BACKLIGHT_POL)
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

/* NETWORK */
#ifdef ESP8266
// WiFi security details
const char* ssid = "<SSID>";
const char* password = "<password>";
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
WiFiUDP ntpClient;
// RTC interrupt config
#define RTC_INTERRUPT_PIN 13 // interrupt on GPIO 13
#else
#include <avr/wdt.h>  // watchdog timer
#include <Ethernet.h>
EthernetUDP ntpClient;
// for Ethernet shield without a built-in MAC address
uint8_t MAC[] = { 0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define RTC_INTERRUPT_PIN 2 // must be a valid EXTERNAL Interrupt pin e.g Arduino UNO 2 or 3
#endif

#define UDP_LISTEN_PORT 8888

/* TIMEZONE */
/*
   Pre-defined timezone definition examples. Use only one of these at a time or risk running
   out of memory on a small AVR e.g. ATmega328. Replace the example UK timezone definition
   with one of these. See the Timezone library documentation for further details.

  // US Eastern Time Zone (New York, Detroit)
  TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  // Eastern Daylight Time = UTC - 4 hours
  TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   // Eastern Standard Time = UTC - 5 hours
  Timezone usET(usEDT, usEST);

  // US Central Time Zone (Chicago, Houston)
  TimeChangeRule usCDT = {"CDT", Second, dowSunday, Mar, 2, -300};
  TimeChangeRule usCST = {"CST", First, dowSunday, Nov, 2, -360};
  Timezone usCT(usCDT, usCST);

  // US Mountain Time Zone (Denver, Salt Lake City)
  TimeChangeRule usMDT = {"MDT", Second, dowSunday, Mar, 2, -360};
  TimeChangeRule usMST = {"MST", First, dowSunday, Nov, 2, -420};
  Timezone usMT(usMDT, usMST);

  // Arizona is US Mountain Time Zone but does not use DST
  Timezone usAZ(usMST, usMST);

  // US Pacific Time Zone (Las Vegas, Los Angeles)
  TimeChangeRule usPDT = {"PDT", Second, dowSunday, Mar, 2, -420};
  TimeChangeRule usPST = {"PST", First, dowSunday, Nov, 2, -480};
  Timezone usPT(usPDT, usPST);

  //Central European Time (Frankfurt, Paris)
  TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
  TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
  Timezone CE(CEST, CET);
*/

//United Kingdom (London, Belfast)
TimeChangeRule BST = {"BST", Last, Sun, Mar, 1, 60};        //British Summer Time
TimeChangeRule GMT = {"GMT", Last, Sun, Oct, 2, 0};         //Standard Time
Timezone UK(BST, GMT);

TimeChangeRule *tcr;        //pointer to the time change rule, use to get the TZ abbrev

#define TIME_ZONE UK  // change to CE for European local time etc.

/*********************/
/* FUNCTIONS AND ISR */
/*********************/

// ISR variables
volatile uint32_t sysClock, ntpAlarmCounter; // the actual clock reference & fraction in millis
volatile bool outputTimestampEnable = false;  // used to trigger output of the current time
//volatile uint32_t sysFraction;  // for future development

// the RTC 1PPS signal calls this ISR
void rtcIntISR(void) {
  // This increments the sysClock and sets the outputTimestampEnable flag
  sysClock++; // the system timestamp is incremented every second by the RTC
  //sysFraction = millis(); // for future development
  ntpAlarmCounter++; // used to trigger NTP time fetches
  outputTimestampEnable = true; // signals OK to print out the time
}// END OF rtcIntISR

// the NTP client calls this function to read the NTP packetBuffer time values
uint32_t readNtpBuffer(uint8_t * buff, uint8_t _start) {
  // NTP data is sent as Big Endian so we need to convert it for Arduino use.
  // Reads the 4 Bytes from the NTP buffer and returns temp as Little Endian
  uint32_t temp;
  uint8_t* tmp = reinterpret_cast<uint8_t*>(&temp);
  for (uint8_t x = 0; x < 4; x++) tmp[x ^ 3] = buff[_start + x];
  return temp;
}// END OF readNtpBuffer

/*******************************************/
/* MAX7219 8 x 7 SEGMENT LED DISPLAY CLASS */
/*******************************************/

// a simple class to drive a MAX7219 7 Segment x 8 Digit LED display
// This class is a standard module the author uses in a lot of projects

class max7219_class {
  public:

    // define the data and clock pins, CS can be labelled LOAD on some modules
#ifdef ESP8266
    // ESP8266-12 on GPIO 12, 14 and 16
#define LED_DIN_PIN 16  // DataIN
#define LED_CS_PIN  14  // ChipSelect or LOAD
#define LED_CLK_PIN 12  // CLocK
#else
    // UNO on A0 - A3
#define LED_DIN_PIN 14
#define LED_CS_PIN  15
#define LED_CLK_PIN 16
#endif

    // define MAX7219 registers
#define MAX7219_DECODE_MODE   0x09
#define MAX7219_INTENSITY     0x0A
#define MAX7219_SCAN_LIMIT    0x0B
#define MAX7219_SHUT_DOWN     0x0C
#define MAX7219_DISPLAY_TEST  0x0F

    // sends the register address and data to the display as a 16 bit word
    void writeLedReg(uint16_t reg, uint8_t val)
    {
      // Bit-bang transfer is used here
      // val is the 8 bit data word to be sent to register reg
      reg = (reg << 8) | val;  // make the 16 bit data word for transfer MSb first
      digitalWrite(LED_CS_PIN, LOW);  // set LED_CS_PIN LOW
      for (uint8_t i = 0; i < 16; i++)
      {
        digitalWrite(LED_DIN_PIN, (reg >> (i ^ 0x0F)) & 0x0001); // set the LED_DIN_PIN
        digitalWrite(LED_CLK_PIN, HIGH);                  // set LED_CLK_PIN HIGH
        digitalWrite(LED_CLK_PIN, LOW);                   // set LED_CLK_PIN LOW
      }
      // put the pins back to idle state
      digitalWrite(LED_DIN_PIN, LOW);   // LED_DIN_PIN LOW
      digitalWrite(LED_CLK_PIN, LOW);   // LED_CLK_PIN LOW
      digitalWrite(LED_CS_PIN, HIGH);   // LED_CS_PIN HIGH to LOAD the data
    }

    // send value x to digit x with decimal point (true/false)
    void writeDigit(uint8_t dig, uint8_t val, bool dp)
    {
      if (dig > 7) return;           // return if invalid digit number
      // invert the digit number for Left to Right display. The module is Right to Left
      // by default so we reverse the direction by XOR-ing the digit number
      dig ^= 0x07;
      // send the data to the MAX7219, module uses digits 1 - 8, we use 0 - 7, dp is bit 7
      writeLedReg(dig + 1, val | (dp << 7));
    }

    // send char '0' - '9' to digit x with decimal point (true/false)
    void writeDigitChar(uint8_t dig, char ch, bool dp) {
      writeDigit(dig, ch - '0', dp);
    }

    // write MAX7219_LED_BLANK to ALL digits
    void blank()
    {
      for (uint8_t _i = 0; _i < 8; _i++) {
        writeDigit(_i, 0x0F, false);
      }
    }

    // initialise the MAX7219 display
    void begin()
    {
      // configure the pins
      pinMode(LED_CLK_PIN, OUTPUT);   // make the CLK pin an OUTPUT
      pinMode(LED_DIN_PIN, OUTPUT);   // make the DATA pin an OUTPUT
      pinMode(LED_CS_PIN, OUTPUT);    // make the CS pin an OUTPUT
      digitalWrite(LED_CS_PIN, HIGH); // set the CS or LOAD pin HIGH
      digitalWrite(LED_CLK_PIN, LOW); // set the CLK pin LOW
      digitalWrite(LED_DIN_PIN, LOW); // set the DATA pin LOW
      writeLedReg(MAX7219_SHUT_DOWN, true);       // normal operation
      writeLedReg(MAX7219_DISPLAY_TEST, true);    // Test Mode On, turn all segments on
      delay(500);
      writeLedReg(MAX7219_DISPLAY_TEST, false);    // Test Mode Off, turn all segments off
      writeLedReg(MAX7219_DECODE_MODE, 0xFF);      // set "B" decode mode characters
      writeLedReg(MAX7219_SCAN_LIMIT, 0x07);       // set the number of digits to scan 1 - 8
      writeLedReg(MAX7219_INTENSITY, 0x07);        // set intensity to 50%
      blank();
    }
} led;

/**************/
/* NTP CLIENT */
/*************/

// an array of NTP servers. The specific server is specified by the index number
// in the call to sendNTPrequest(uint8_t index)
const char *ntpServer[3] = {"time.windows.com", "0.uk.pool.ntp.org", "0.jp.pool.ntp.org"}; // the NTP servers for testing

#define NTP_TIMEOUT 250       // maximum time to wait for an NTP server response in milliseconds
#define NTP_ROUNDTRIP_MAX 30  // the maximinun NTP round-trip in milliseconds
#define NTP_FETCH_PERIOD 60   // how often to fetch NTP time in seconds
#define NTP_FAIL_COUNT 5     // the number of times to retry an NTP fetch before giving up
#define NTP_FAIL_RETRY  2     // time in seconds to wait before trying an NTP re-fetch
#define NTP_PACKET_SIZE 48    // the size of the NTP packet
#define NTP_70_YEARS 2208988800UL // seconds between 1900 (NTP) & 1970 (UNIX)
#define SVR_TIME_SECS 40      // buffer offset for NTP timestamp
#define SVR_TIME_FRAC 44      // buffer offset for NTP fraction
uint32_t ntpTime, ntpFraction, ntpRoundtrip;
uint8_t packetBuffer[NTP_PACKET_SIZE];

// send an NTP request to the time server at the given address (ntpServer[index])
uint32_t sendNTPrequest(uint8_t index)
{
  // we can send an empty packet with just the first byte header to the NTP server
  memset(&packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  // send the NTP packet to the server
  ntpClient.flush(); // discard any previously received packet data
  ntpFraction = 0;
  // send the packet and exit if error
  if (!ntpClient.beginPacket(ntpServer[index], 123)) return 1;
  ntpClient.write(packetBuffer, NTP_PACKET_SIZE);
  if (!ntpClient.endPacket()) return 2;
  // now wait for the response from the server
  ntpRoundtrip = millis();
  bool _timeOut = true;
  uint8_t packetSize;
  uint32_t timeOutTimer = millis();
  while ((millis() - timeOutTimer) < NTP_TIMEOUT && _timeOut)
  {
    delay(0); // for ESP8266 compatibility
    if (packetSize = ntpClient.parsePacket()) _timeOut = false;
  }
  ntpRoundtrip = millis() - ntpRoundtrip;
  if (_timeOut) return 3;
  if (packetSize != NTP_PACKET_SIZE) return 4;
  // we got a valid response packet so carry on
  ntpClient.flush();
  // extract the timestamp and fractional seconds. Convert NTP timestamp to UNIX
  while (ntpClient.available()) ntpClient.read(packetBuffer, 48);
  ntpTime = readNtpBuffer(packetBuffer, SVR_TIME_SECS) - NTP_70_YEARS;
  // the timestamp fractional seconds are picoseconds so convert to milliseconds
  // ntpFraction * 10 ^ 6 / 2 ^ 32 /1000 = milliseconds
  ntpFraction = (uint32_t)((readNtpBuffer(packetBuffer, SVR_TIME_FRAC) * pow(10, 6)) / pow(2 , 32)) / 1000UL;
  return 0;
}// END OF sendNTPrequest

/*********/
/* SETUP */
/*********/

void setup()
{
#ifdef ESP8266
  WiFi.disconnect();
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_4S);
#else
  wdt_disable();
  wdt_enable(WDTO_4S);
#endif

#if DEBUG
  Serial.begin(57600);
  Serial << F("A.N.T. Accurate Ntp Time (C) Phil Morris 2018 <www.lydiard.plus.com>") << endl;
#endif

  // configure the LED or LCD display
#ifdef LED_DISPLAY
  led.begin();
#elif defined LCD_DISPLAY
  lcd.begin(16, 2);
#endif

#ifdef ESP8266
  WiFi.mode(WIFI_STA);  // set ESP8266 as a WorkSTAtion & Server
  WiFi.begin(ssid, password); // start the network for DHCP
  while (WiFi.status() != WL_CONNECTED)
#else
  while (!Ethernet.begin(MAC))
#endif
  {
#if DEBUG
    Serial.print(".");  // print a dot on the monitor
#endif
    delay(500); // wait 500 mS
  }
#ifdef ESP8266 && DEBUG
  Serial << (F("IP address is ")) << WiFi.localIP() << endl;
#elif not defined ESP8266 && DEBUG
  Serial << (F("IP address is ")) << Ethernet.localIP() << endl;
#endif

  ntpClient.begin(UDP_LISTEN_PORT);

  // set up the RTC (DS3231 or DS3232 or similar)
  RTC.writeRTC(RTC_CONTROL, 0); // clear the RTC control register
  // set RTC to trigger the RTC_INTERRUPT_PIN pin every second (1PPS)
  RTC.squareWave(SQWAVE_1_HZ);

  // set up the RTC interrupt. MUST BE A VALID EXTERNAL INTERRUPT!
  // the Arduino ESP8266 IDE can support digitalPinToInterrupt()
  // but, to be safe...
#ifdef ESP8266
  attachInterrupt(RTC_INTERRUPT_PIN, rtcIntISR, FALLING);
#else
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcIntISR, FALLING);
#endif

  // the RTC Interrupt output is open collector so set the pullup resistor on the interrupt pin
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);

  sysClock = RTC.get(); // synchronise the sysClock with the RTC for initial time setting
  ntpAlarmCounter = NTP_FETCH_PERIOD - 2;  // trigger an NTP fetch on first-run

}// END OF setup

/********/
/* LOOP */
/********/

#define MILLIS_MIN 2000UL // 2 seconds
#define MILLIS_MAX 4294967295UL - MILLIS_MIN  // value - 2000 (2 seconds)

const char* weekdays[8] = {"Err", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char* months[13] = {"Err", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

bool ntpFetchFlag = false, ntpSetFlag = false, timeSynced = false, ntpFailFlag = false;
uint32_t rtcSetDelay = 0;
uint8_t ntpFailCounter = 0;

void loop()
{
#ifdef ESP8266
  ESP.wdtFeed();
#else
  wdt_reset();
#endif

  // ntpAlarmCounter increments each second and sets the ntpFetchFlag
  // when the required time period has elapsed.
  if (!ntpFetchFlag && ntpAlarmCounter == NTP_FETCH_PERIOD) ntpFetchFlag = true;  // fetch the NTP time

  //fetch the NTP time if millis haven't rolled over.
  // The valid window is millis > 2000 and millis < 4294967295 - 2000 and we also check that the NTP
  // round-trip time is acceptable. A basic retry counter attempts to fetch the NTP time a number of times
  // before giving up until the next scheduled NTP fetch time. The LED's first digit decimal point is lit
  // if the NTP fetch fails completely. On an LCD display, the first digit becomes '*'
  if (ntpFetchFlag && millis() > MILLIS_MIN && millis() < MILLIS_MAX) {
    // fetch the NTP timestamp from the indexed NTP server, returns 0 if successful
    // The index allows you to choose an NTP server from the server names array easily
    uint8_t result = sendNTPrequest(0);
    if (!result && ntpRoundtrip <= NTP_ROUNDTRIP_MAX) {
      // calculate the delay before updating the RTC and sysClock
      rtcSetDelay = (uint32_t)(millis() + (1000UL - ntpFraction)) + (ntpRoundtrip >> 1);
      ntpSetFlag = true;    // sync can take place
      ntpFailFlag = false;  // clear the ntpFailFlag
      ntpAlarmCounter = 0;  // clear the ntpAlarmCounter
      ntpTime++; // add 1 second as we're setting the sysClock 1 second late
    }
    else {
      // the NTP request returned an error status
#if DEBUG
      switch (result) {
        case 1:
          Serial << F("NTP beginPacket Failure!") << endl;
          break;
        case 2:
          Serial << F("NTP Client endPacket Failure!") << endl;
          break;
        case 3:
          Serial << F("NTP Timeout Error!") << endl;
          break;
        case 4:
          Serial << F("NTP Packet Size Error!") << endl;
          break;
        default:
          Serial << F("NTP Round-tript too long, aborting!") << endl;
      }
#endif
      // the NTP fetch has failed, so start counting the failures
      if (ntpFailCounter++ == NTP_FAIL_COUNT) {
        // total NTP fetch failure
        ntpAlarmCounter = 0;  // reset the ntpAlarmCounter
        ntpFailCounter = 0;   // reset the ntpFailCounter
        ntpFailFlag = true;   // set the ntpFailFlag
#if DEBUG
        Serial << F("Total NTP Fetch Failure!") << endl;
#endif
      }
      else ntpAlarmCounter -= NTP_FAIL_RETRY; // try an NTP fetch again in NTP_FAIL_RETRY seconds
    }
    ntpFetchFlag = false; // clear the fetch Alarm flag
  }

  // sync everything when rtcSetDelay has expired
  if (ntpSetFlag && millis() >= rtcSetDelay) {
    RTC.set(ntpTime); // set the RTC
    sysClock = ntpTime; // set sysClock
    ntpSetFlag = false; // clear the sync flag
    timeSynced = true;  // time DEBUG printout includes NTP timing details
    rtcSetDelay = 0;
  }

  // print out the sysClock timestamp
  if (outputTimestampEnable) {
    outputTimestampEnable = false;  // clear the trigger flag
    char buff[32];
    tmElements_t tm;
    // make the current local time using the Timezone library
    breakTime(TIME_ZONE.toLocal(sysClock, &tcr), tm);
    sprintf(buff, "%02u:%02u:%02u %s  %02u-%s-20%02u", tm.Hour, tm.Minute, tm.Second, weekdays[tm.Wday], tm.Day, months[tm.Month], tm.Year - 30);
#if DEBUG
    if (timeSynced) {
      Serial << buff  << F(" * NTP Fraction: ") << ntpFraction << " mS - " << F("NTP Round-trip: ") << ntpRoundtrip << " mS" <<  endl;
    }
    else Serial << buff  << endl;
#endif

#ifdef LED_DISPLAY
    // display the time on the LED panel
    led.writeDigitChar(0, buff[0], ntpFailFlag);  // the digit 1 decimal point is ON if ntpFailFlag is true
    led.writeDigitChar(1, buff[1], false);
    led.writeDigitChar(3, buff[3], false);
    led.writeDigitChar(4, buff[4], false);
    led.writeDigitChar(6, buff[6], false);
    led.writeDigitChar(7, buff[7], false);
#elif defined LCD_DISPLAY
    lcd.setCursor(0, 0);
    if (ntpFailFlag) lcd.print("*");
    else lcd.print(" ");
    lcd.setCursor(4, 0);
    for (uint8_t x = 0; x < 8; x++) lcd.print(buff[x]);
    static uint8_t displayDay = 0;  // only change the displayed date if it has changed
    if (day(sysClock) != displayDay) {
      displayDay = day(sysClock);
      lcd.setCursor(0, 1);
      for (uint8_t x = 9; x < 25; x++) lcd.print(buff[x]);
    }
#endif
    timeSynced = false;
  }
}// END OF loop


