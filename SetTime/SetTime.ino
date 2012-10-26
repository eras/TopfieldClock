// time routines extracted from: http://www.jbox.dk/sanos/source/lib/time.c.html
// Copyright (C) 2002 Michael Ringgaard. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
// 1. Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.  
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.  
// 3. Neither the name of the project nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
// 

// from http://embeddedgurus.com/barr-code/2009/09/binary-literals-in-c/
#define HEX__(n) 0x##n##LU
#define B8__(x) ((x&0x0000000FLU)?1:0) \
+((x&0x000000F0LU)?2:0) \
+((x&0x00000F00LU)?4:0) \
+((x&0x0000F000LU)?8:0) \
+((x&0x000F0000LU)?16:0) \
+((x&0x00F00000LU)?32:0) \
+((x&0x0F000000LU)?64:0) \
+((x&0xF0000000LU)?128:0)

// User-visible Macros
#define B8(d) ((uint8_t)B8__(HEX__(d)))

/* Time manipulation code */

struct tm {
  int tm_year;
  int tm_yday;
  int tm_mon;
  int tm_mday;
  int tm_wday;
  int tm_hour;
  int tm_min;
  int tm_sec;
};

#define SUNDAY 0
#define MONDAY 1
#define TUESDAY 2
#define WEDNESDAY 3
#define THURSDAY 4
#define FRIDAY 5
#define SATURDAY 6

#define EPOCH_YR                1970
#define SECS_DAY                (24L * 60L * 60L)
#define LEAPYEAR(year)          (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)          (LEAPYEAR(year) ? 366 : 365)
#define FIRSTSUNDAY(timp)       (((timp)->tm_yday - (timp)->tm_wday + 420) % 7)
#define FIRSTDAYOF(timp)        (((timp)->tm_wday - (timp)->tm_yday + 420) % 7)

#define TIME_MAX                2147483647L

#define TZ_OFFSET (3600 * 3)

int _daylight = 0;                  // Non-zero if daylight savings time is used
long _dstbias = 0;                  // Offset for Daylight Saving Time
long _timezone = 0;                 // Difference in seconds between GMT and local time
char *_tzname[2] = {"GMT", "GMT"};  // Standard/daylight savings time zone names

const int yearDays[2][12] = {
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

struct tm *gmtime_r(const uint32_t time, struct tm *tmbuf) {
  unsigned long dayclock, dayno;
  int year = EPOCH_YR;

  dayclock = (unsigned long) time % SECS_DAY;
  dayno = (unsigned long) time / SECS_DAY;

  tmbuf->tm_sec = dayclock % 60;
  tmbuf->tm_min = (dayclock % 3600) / 60;
  tmbuf->tm_hour = dayclock / 3600;
  tmbuf->tm_wday = (dayno + 4) % 7; // Day 0 was a thursday
  while (dayno >= (unsigned long) YEARSIZE(year)) {
    dayno -= YEARSIZE(year);
    year++;
  }
  tmbuf->tm_year = year;
  tmbuf->tm_yday = dayno;
  tmbuf->tm_mon = 0;
  while (dayno >= (unsigned long) yearDays[LEAPYEAR(year)][tmbuf->tm_mon]) {
    dayno -= yearDays[LEAPYEAR(year)][tmbuf->tm_mon];
    tmbuf->tm_mon++;
  }
  tmbuf->tm_mday = dayno + 1;
  return tmbuf;
}

int countsOfWeekdayBeforeNextMonth(struct tm* tm, int wday)
{
  int count = 0;
  int curMday = tm->tm_mday;
  int curWday = tm->tm_wday;
  while (curMday <= yearDays[LEAPYEAR(tm->tm_year)][tm->tm_mon]) {
    if (curWday == wday) {
      ++count;
    }
    ++curMday;
    ++curWday;
    if (curWday == 7) {
      curWday = 0;
    }
  }
  return count;
}

bool afterOrAtLastSunday(struct tm* tm)
{
  return
    (tm->tm_wday == 0 && countsOfWeekdayBeforeNextMonth(tm, SUNDAY) == 1) ||
    (countsOfWeekdayBeforeNextMonth(tm, SUNDAY) == 0);
}

int tzOffset(struct tm* tm)
{
  bool isSummerTime =
    (tm->tm_mon > 2 ||
     (tm->tm_mon == 2 && /* maaliskuu, tm_mon alkaa nollasta */
      afterOrAtLastSunday(tm)
     )
    ) && (
     tm->tm_mon < 9 ||
     (tm->tm_mon == 9 && /* lokakuu, tm_mon alkaa nollasta */
      !afterOrAtLastSunday(tm)
     )
    );
  if (isSummerTime) {
    return 3600 * 3;
  } else {
    return 3600 * 2;
  }
}

/* End of time manipulation code */

/* Networking code */ 
#include "etherShield.h"
#include "ETHER_28J60.h"

#define TRANSMIT_TIMESTAMP_P 40
#define NTP_PACKET_SIZE 48

static uint8_t mac[6] = {0x54,0x55,0x58,0x10,0x00,0x24}; 
static uint8_t ip[4] = {192,168,11,60};
static uint16_t port = 80;

static const int bufferSize = 200;
static uint8_t buf[bufferSize];

extern EtherShield es;

static const int ntpPort = 123;

ETHER_28J60 ethernet;

/* End of networking code */

int out = 3;
int in = 2;
int debug = 4;

/* Bit sequence sending declarations */

uint8_t setHHMM[] = { 4, 174, 230, 0, 31, 224, 1, 126, 128, 241, 127, 3, 255 };

uint8_t bitsOfDigit[11] = {
  B8(00000001),
  B8(01001111),
  B8(00010010),
  B8(00000110),
  B8(01001100),
  B8(00100100),
  B8(00100000),
  B8(00001111),
  B8(00000000),
  B8(00000100),
  B8(01111110)
};

volatile uint8_t* sequence;

/* End of bit sequence sending declarations */

/* Timer interrupt code */
volatile int writeBitsLeft = 0;
volatile int writeCurBit = 0;
volatile int writeCurByte = 0;

uint32_t previousNtp = 0;
uint32_t previousNtpFractions = 0;
volatile uint32_t curTime = 0;
volatile uint32_t curTimeFractions = 0;
bool knowTime = false;

static const int prescaler = 1;
static const double systemHz = 16000000;
static const double bitLength = 52.0 / 1000000.0;
static const unsigned int finetuning = 50; // fine-tuned with an oscilloscope
static const unsigned int timerPreload = 65536 - systemHz / prescaler / (1 / bitLength) + finetuning;

static const double two_to_32 = 4294967296.0;

// how much to advance curTimeFractions in the interrupt handler
volatile uint32_t fractionsPerOverflow = two_to_32 * bitLength;

void setupTimer()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 34286;            // preload timer 65536-16MHz/256/2Hz
//  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TCCR1B |= (1 << CS10);    // no prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

static int flag = 0;

ISR(TIMER1_OVF_vect)
{
  TCNT1 = timerPreload;            // preload timer
  curTimeFractions += fractionsPerOverflow;
  if (curTimeFractions < fractionsPerOverflow) {
    ++curTime;
  }
  if (writeBitsLeft > 0) {
    --writeBitsLeft;
    int value = ((sequence[writeCurByte] >> writeCurBit) & 1) ? HIGH : LOW;
    digitalWrite(out, value);
    if (++writeCurBit == 8) {
      writeCurBit = 0;
      ++writeCurByte;
    }
  } else {
    digitalWrite(out, HIGH);
  }
}

/* End of timer interrupt code */

// the loop routine runs over and over again forever:
void setup() {
  pinMode(out, OUTPUT);
  pinMode(debug, OUTPUT);
  pinMode(in, INPUT);
  setupTimer();
  digitalWrite(out, HIGH);
  digitalWrite(debug, LOW);
  delay(500);
  setDigits(10, 10, 10, 10, B8(0000000));
  ethernet.setup(mac, ip, port);
  Serial.begin(9600);
  Serial.println("Start.");
  Serial.print("Fractions per overflow: "); Serial.println(fractionsPerOverflow);
}

// sends stuff from sequence, sequenceAtBit/sequenceAtByte
void sendBits(uint8_t* ptr, int bits)
{
  noInterrupts();
  writeBitsLeft = bits;
  writeCurBit = 0;
  writeCurByte = 0;
  sequence = ptr;
  interrupts();
  
  while (writeBitsLeft > 0) {
    // wait for writing to go through
  }

#if 0
  writeCurByte = 0;
  writeCurBit = 0;
  
  for (int c = 0; c < bits; ++c) {
    int value = ((sequence[writeCurByte] >> writeCurBit) & 1) ? HIGH : LOW;
    Serial.print(value ? "1" : "0");
    if (++writeCurBit == 8) {
      writeCurBit = 0;
      ++writeCurByte;
    }
  }
  Serial.println();
#endif
}

void insertBits(int n, uint8_t* dst, int dstBit, uint8_t data)
{
  for (int bitN = n - 1; bitN >= 0; --bitN) {
    *dst = *dst & ~(1 << dstBit) | ((!!(data & (1 << bitN))) << dstBit);
    if (++dstBit == 8) {
      ++dst;
      dstBit = 0;
    }
  }
}

inline uint8_t digit(int n, bool dot)
{
  return bitsOfDigit[n] << 1 | (dot ? 0 : 1);
}

void setDigits(int h1, int h2, int m1, int m2, int dots)
{
  insertBits(8, setHHMM + 3, 1, digit(h1, dots & 8));
  insertBits(8, setHHMM + 4, 6, digit(h2, dots & 4));
  insertBits(8, setHHMM + 6, 2, digit(m1, dots & 2));
  insertBits(8, setHHMM + 8, 0, digit(m2, dots & 1));
  insertBits(1, setHHMM + 9, 2, ~(dots >> 4));
  insertBits(1, setHHMM + 9, 3, ~(dots >> 5));
  sendBits(setHHMM, sizeof(setHHMM) * 8);
}

void sendHHMM(int hh, int mm, int dots)
{
  int h1 = hh / 10;
  int h2 = hh % 10;
  int m1 = mm / 10;
  int m2 = mm % 10;
  setDigits(h1, h2, m1, m2, dots);
}

static void checkTime()
{
  static int checkInterval = 0;
  static uint32_t prevTime = 0;
  if (++checkInterval == 1000) {
    checkInterval = 0;
    if (knowTime) {
      noInterrupts();
      uint32_t now = curTime;
      interrupts();
      if (now != prevTime) {
        prevTime = now;
        struct tm tm;
        gmtime_r(now, &tm);
        int offset = tzOffset(&tm);
        gmtime_r(now + offset, &tm);        
        int colon = (now & 2) == 0 ? B8(0110000) : 0;
        sendHHMM(tm.tm_hour, tm.tm_min, colon);
        Serial.print(tm.tm_year);
        Serial.print("-");
        Serial.print(tm.tm_mon + 1);
        Serial.print("-");
        Serial.print(tm.tm_mday);
        Serial.print(" ");
        Serial.print(tm.tm_wday);
        Serial.print(" ");
        Serial.print(tm.tm_hour);
        Serial.print(":");
        Serial.print(tm.tm_min);
        Serial.print(":");
        Serial.print(tm.tm_sec);
        Serial.println();
      }
    }
  }
}

// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1207226548
void printDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
	 mult *=10;

    if(val >= 0)
	frac = (val - int(val)) * mult;
    else
	frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
	padding--;
    while(  padding--)
	Serial.print("0");
    Serial.print(frac,DEC) ;
  }
}

double fixedPointDistance(uint32_t a1, uint32_t a2, uint32_t b1, uint32_t b2)
{
  return ((int64_t) b2 - (int64_t) b1) / two_to_32 + (int32_t) (b1 - a1);
}

void loop() {
  checkTime();
  
  uint16_t length = es.ES_enc28j60PacketReceive(bufferSize, buf);
  if (length && length >= 42 /* IP packet min length */ && length <= UDP_DATA_P + NTP_PACKET_SIZE) {
    if (es.ES_eth_type_is_arp_and_my_ip(buf, length)) {
      es.ES_make_arp_answer_from_request(buf);
    } else if (es.ES_eth_type_is_ip_and_my_ip(buf, length) &&
               buf[IP_PROTO_P]==IP_PROTO_ICMP_V &&
               buf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V) {
      es.ES_make_echo_reply_from_request(buf, length);
    } else if (buf[IP_PROTO_P] == IP_PROTO_UDP_V && buf[UDP_DST_PORT_L_P] == ntpPort && length == UDP_DATA_P + NTP_PACKET_SIZE) {
      digitalWrite(debug, HIGH);
      // NOTE: requires disabling the pass-only-arp-broadcast filter from enc28j60
      uint32_t a = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 0];
      uint32_t b = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 1];
      uint32_t c = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 2];
      uint32_t d = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 3];
      uint32_t t1 = ((a << 24) | (b << 16) | (c << 8) | d) - 2208988800ul;
      a = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 4];
      b = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 5];
      c = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 6];
      d = buf[UDP_DATA_P + TRANSMIT_TIMESTAMP_P + 7];
      uint32_t t2 = ((a << 24) | (b << 16) | (c << 8) | d);
      noInterrupts();
      uint32_t oldTime = curTime;
      uint32_t oldTimeFractions = curTimeFractions;
      curTime = t1;
      curTimeFractions = t2;
      interrupts();
      digitalWrite(debug, LOW);
      if (knowTime) {
        Serial.print(oldTime);
        Serial.print(" ");
        Serial.print(oldTimeFractions);
        Serial.print(" ");
        Serial.print(t1);
        Serial.print(" ");
        Serial.print(t2);
        Serial.print(" ");
        Serial.print(t1 - oldTime);
        double error = fixedPointDistance(t1, t2, oldTime, oldTimeFractions);
        Serial.print(" ");
        printDouble(error, 6);
        double sinceLastNtp = fixedPointDistance(t1, t2, previousNtp, previousNtpFractions);
        Serial.print(" ");
        printDouble(sinceLastNtp, 6);
        Serial.print(" ");
        double errorPerSecond = error / sinceLastNtp;
        printDouble(errorPerSecond, 9);
        Serial.println();
        fractionsPerOverflow += errorPerSecond * 100000;
        Serial.print("Fractions per overflow: ");
        Serial.println(fractionsPerOverflow);
      }
      previousNtp = t1;
      previousNtpFractions = t2;
      knowTime = true;
    }
  }
  digitalWrite(debug, LOW);
}
