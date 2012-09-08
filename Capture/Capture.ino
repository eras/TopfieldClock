int in = 3;
int debug = 4;

static const int prescaler = 1;
static const double systemHz = 16000000;
static const double bitLength = 52.0 / 1000000.0;
static const unsigned int finetuning = 50; // fine-tuned with an oscilloscope
static const unsigned int timerPreload = 65536 - systemHz / prescaler / (1 / bitLength) + finetuning;
static const unsigned int timerPreload2 = 65536 - systemHz / prescaler / (1 / (bitLength / 2)) + finetuning;

static const int sampleLength = 52;
static const int maxSampleBytes = 500;

volatile bool intrReading = false;
volatile bool intrFirst;
volatile int intrAtBit;
volatile int intrAtByte;
volatile int intrOnes;
volatile int intrZeroes;
unsigned char intrSamples[maxSampleBytes];

// the loop routine runs over and over again forever:
void setup() {
  pinMode(debug, OUTPUT);
  pinMode(in, INPUT);
  digitalWrite(debug, HIGH);
  setupTimer();
  Serial.begin(115200);
  Serial.println("Start");
}

void setupTimer()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = timerPreload2;            // preload timer 65536-16MHz/256/2Hz
//  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TCCR1B |= (1 << CS10);    // no prescaler 
//  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

void enableTimer()
{
  noInterrupts();
  TIMSK1 |= (1 << TOIE1);
  intrReading = true;
  intrFirst = true;
  intrAtBit = 0;
  intrAtByte = 0;
  intrOnes = 0;
  intrZeroes = 0;
  intrSamples[0] = 0;
  interrupts();
}

void disableTimer()
{
  intrReading = false;
  TIMSK1 &= ~(1 << TOIE1);
}

ISR(TIMER1_OVF_vect)
{
  TCNT1 = timerPreload;            // preload timer
#if 1
  int value = digitalRead(in) == HIGH ? 1 : 0;
  digitalWrite(debug, value);
  intrSamples[intrAtByte] |= value << intrAtBit;
  if (++intrAtBit == 8) {
    ++intrAtByte;
    intrSamples[intrAtByte] = 0;
    intrAtBit = 0;
  }
  if (value) {
    ++intrOnes;
    intrZeroes = 0;
  } else {
    ++intrZeroes;
    intrOnes = 0;
  }
  if (intrOnes > 20 || intrZeroes > 20 || intrAtByte == maxSampleBytes) {
    disableTimer();
  }
#else
  static int value = 0;
  value ^= 1;
  digitalWrite(out, value);
  digitalWrite(debug, value);
#endif
}

void dump() {
  enableTimer();
  while (intrReading) {
    // wait
  }
  for (int c = 0; c < intrOnes; ++c) {
    if (intrAtBit-- == 0) {
      intrAtBit = 7;
      --intrAtByte;
    }
  }
  for (int atByte = 0, atBit = 0;
       atByte != intrAtByte || atBit != intrAtBit;
       (++atBit == 8) ? (atBit = 0, ++atByte) : 0) {
    if (atByte && atBit == 0) {
      Serial.print(" ");
    }
    Serial.print((intrSamples[atByte] & (1 << atBit)) ? '1' : '0');
  }
  if (intrZeroes == 20) {
    Serial.print("/*..*/");
  }
  Serial.println();
}

void loop() {
  static bool started = false;
  if (!started) {
    int value = digitalRead(in);
    if (value == HIGH) {
      started = true;
    }
  } else {
    int value = digitalRead(in);
    if (value == LOW) {
      dump();
      started = false;
    }
  }
}
