# Arduino-ESP8266-Accurate-NTP-Clock
An accurate NTP Clock for Arduino or ESP8266 using a DS3231 or similar RTC module

A.N.T. Accurate Ntp Time

A Proof Of Concept attempt to get accurate(ish) time from an NTP server. The result gives the time within
+-10 milliseconds of the time as referenced to the DCF77 time signal from Frankfurt, Germany.
NTP request round-trip should be in the range 12 - 20 mS to achieve the accuracy stated above!

In use the actual discrepancy can be as low as +- 1mS. +-5mS to +-10 mS is typical.

Change the "const char *ntpServer = "time.windows.com";"  NTP server entries in the NTP Client function
to time servers close to your location.

The DS3231 (or similar) RTC is used to generate a 1 Pulse Per Second output to increment the sysClock variable.
The 1PPS signal is really a 1Hz square wave but we only use the falling edge so it's 1PPS to us.
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

ALL ESP8266 PIN NUMBERS ARE THE GPIO NUMBERS, NODEMCU PIN NUMBERS OR ARDUINO PIN NUMBERS!
