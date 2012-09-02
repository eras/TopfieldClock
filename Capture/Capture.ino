int in = 2;
int debug = 4;

static const int sampleLength = 52;

// the loop routine runs over and over again forever:
void setup() {
  pinMode(debug, OUTPUT);
  pinMode(in, INPUT);
  digitalWrite(debug, HIGH);
  Serial.begin(115200);
  Serial.println("Start");
}

unsigned long time;

void dump() {
  bool samples[200];
  unsigned n = 0;
  unsigned ones = 0;
  unsigned zeroes = 0;
  while (ones < 20 && zeroes < 20 && n < sizeof(samples) / sizeof(*samples)) {
    int value = digitalRead(in);
    digitalWrite(debug, value);
    samples[n] = value == HIGH;
    time += sampleLength;
    unsigned sleepTime = time - micros();
    delayMicroseconds(sleepTime);
    if (value == HIGH) {
      ++ones;
      zeroes = 0;
    } else {
      ++zeroes;
      ones = 0;
    }
    ++n;
  }
  Serial.print('{');
  for (int c = 0; c < n - ones; ++c) {
    if (c) {
      Serial.print(",");
    }
    Serial.print(samples[c] ? '1' : '0');
  }
  if (zeroes == 20) {
    Serial.print("/*..*/");
  }
  Serial.println("};");
}

void loop() {
  static bool started = false;
  int value = digitalRead(in);
  if (!started) {
    if (value == HIGH) {
      started = true;
    }
  } else {
    time = micros() + sampleLength / 2;
    if (value == LOW) {
      dump();
      started = false;
    }
  }
}
