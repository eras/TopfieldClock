int out = 2;
int in = 3;
int debug = 4;

const unsigned sequenceSize = 200;
unsigned char sequence[sequenceSize + 1];
int sequenceAt = 0; // for reading sequences
int sequenceAtBit = 0;
int prevSequenceSize = 0;

int inBinarySequence = 0;

volatile int writeBitsLeft = 0;
volatile int writeCurBit = 0;
volatile int writeCurByte = 0;

unsigned char initsequence[] = {
#if 0
0,0,1,0,0,0,0,0,0,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,1,1,0,0,0,1,1,1,1,0,1,0,0,1,1,0,0,0,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,1,0,0,0,0,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,1,0,1,1,1,0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,1,0,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,0,1,0,0,0,0,1,1,1,0,0,0,0,1,1,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,0,1,0,1,1,1,1,0,1,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,0,0,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,1,1,1,0,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,1,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,1,1,0,0,1,1,1,0,1,1,0,0,0,0,0,0,0,1,1,1,0,0,1,0,0,0,0,0,0,1,1,1,1,0,1,0,0,0,1,0,0,0,1,1,1,0,1,0,0,0,0,0,0,1,1,1,1,1,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,1,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,0,1,1,0,0,0,1,1,1,1,1,1,0,1,1,0,1,0,1,0,1,1,1,1,1,0,2,
0,1,1,0,0,0,0,0,0,2,

0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,0,1,1,0,0,0,1,1,1,1,1,1,0,1,1,0,1,0,1,0,1,1,1,1,1,1,0,2,
0,1,1,0,0,0,0,0,1,1,1,1,0,0,1,0,0,0,0,0,0,1,1,1,0,0,1,0,0,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,1,0,0,1,0,0,0,0,2,
0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,0,1,0,1,0,0,1,1,1,1,1,0,0,1,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,2,
0,0,0,0,0,0,0,0,1,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,0,0,1,0,1,0,1,1,1,0,0,1,0,1,1,0,1,0,1,1,1,1,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,1,0,1,0,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,0,0,0,0,0,0,2,

0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,1,0,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,2,
0,0,1,0,1,0,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,2,
//0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,2,
  0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,2,

0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,2,
0,1,0,0,0,0,0,0,0,2, 
#endif

#if 0
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,2,;
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,2,

0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,1,1,0,1,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,0,1,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,

// the following makes 01:59 appear on the screen:
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,0,0,0,0,1,0,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,1,0,0,0,0,0,1,0,0,1,1,1,1,0,0,0,2,
0,1,1,0,0,0,0,0,0,

1,1,1,0,0,1,0,0,1,1,1,0,1,1,1,
0,1,1,1,1,1,1,1,1,1,1,1,1,
0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0,1,1,1,1,
0,0,1,0,0,0,0,0,0,2,
0,0,0,0,0,0,0,0,0,2,

0,0,1,0,0,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,1,0,1,1,1,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,0,0,0,1,0,0,1,1,1,1,1,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,1,0,1,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,1,0,1,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,1,0,0,1,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,1,0,0,1,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,0,1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,0,0,1,0,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,1,1,1,0,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,0,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,0,1,1,0,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,1,1,0,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,0,1,0,0,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,1,0,0,1,1,1,0,2,
0,1,0,0,0,1,1,0,1,1,1,1,0,1,0,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,1,1,1,0,0,1,0,0,1,1,1,0,2,
0,0,0,0,0,1,1,0,2,
0,0,1,1,0,1,0,0,0,1,1,1,1,1,0,1,1,1,1,0,1,0,0,1,1,1,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,1,0,0,1,1,0,0,1,1,1,1,1,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,0,0,0,1,1,1,0,1,1,1,1,0,2,
0,0,0,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,0,0,1,0,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,0,1,0,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,1,0,0,0,0,0,0,1,0,1,1,1,1,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,0,0,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,0,0,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,0,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,1,0,0,1,1,0,0,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,0,1,0,0,1,1,0,0,1,1,1,1,0,1,0,0,0,1,1,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,0,0,0,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,2,
0,0,1,0,0,0,0,0,1,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,2,
#endif

3};

#if 0
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,1,0,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,2,
0,0,1,0,1,0,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,0,2,

0,1,1,0,0,0,0,0,0,2,
0,0,1,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,2,
0,1,0,0,0,0,0,0,0,2, ;


#endif

static const int prescaler = 1;
static const double systemHz = 16000000;
static const double bitLength = 52.0 / 1000000.0;
static const unsigned int finetuning = 50; // fine-tuned with an oscilloscope
static const unsigned int timerPreload = 65536 - systemHz / prescaler / (1 / bitLength) + finetuning;
//static const unsigned int = 0;

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

ISR(TIMER1_OVF_vect)
{
  TCNT1 = timerPreload;            // preload timer
#if 1
  if (writeBitsLeft > 0) {
    --writeBitsLeft;
    int value = ((sequence[writeCurByte] >> writeCurBit) & 1) ? HIGH : LOW;
    digitalWrite(out, value);
    digitalWrite(debug, value);
    if (++writeCurBit == 8) {
      writeCurBit = 0;
      ++writeCurByte;
    }
  } else {
    digitalWrite(out, HIGH);
    digitalWrite(debug, HIGH);
  }
#else
  static int value = 0;
  value ^= 1;
  digitalWrite(out, value);
  digitalWrite(debug, value);
#endif
}

// the loop routine runs over and over again forever:
void setup() {
  pinMode(out, OUTPUT);
  pinMode(debug, OUTPUT);
  pinMode(in, INPUT);
  digitalWrite(out, HIGH);
  digitalWrite(debug, HIGH);
  Serial.begin(9600);
  Serial.print("Start. timerPreload=");
  Serial.println(timerPreload);
  setupTimer();
}

void sendSequence(const unsigned char* seq)
{
  Serial.println("Sending sequence");
  static unsigned long time = micros();
  for (int c = 0; seq[c] != 3; ++c) {
    if (seq[c] == 2) {
      digitalWrite(out, HIGH);
      digitalWrite(debug, HIGH);
      delayMicroseconds(52 * 12);
      time = micros();
    } else {
      digitalWrite(out, seq[c]);
      digitalWrite(debug, seq[c]);
      time += 52;
      unsigned int delta = time - micros();
      delayMicroseconds(delta);
    }
  }
  digitalWrite(out, HIGH);
  digitalWrite(debug, HIGH);
  Serial.println("Done sending sequence");
}

// sends stuff from sequence, sequenceAtBit/sequenceAtByte
void sendBinarySequenceHw()
{
  noInterrupts();
  writeBitsLeft = sequenceAt * 8 + sequenceAtBit;
  writeCurBit = 0;
  writeCurByte = 0;
  interrupts();
  
  while (writeBitsLeft > 0) {
    // wait for writing to go through
  }
  Serial.println(".");
}

void doit() {
  sendSequence(initsequence);
}

void baseLoop() {
  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'i': {
        doit();
      } break;
      case '0': {
        if (sequenceAt < sequenceSize) {
          sequence[sequenceAt++] = 0;
        } else {
          Serial.println("Sequence is too long");
        }
      } break;
      case '1': {
        if (sequenceAt < sequenceSize) {
          sequence[sequenceAt++] = 1;
        } else {
          Serial.println("Sequence is too long");
        }
      } break;
      case 'b': {
        sequenceAt = 0;
        sequenceAtBit = 0;
        inBinarySequence = true;
      } break;
      case ';': {
        if (!prevSequenceSize || sequenceAt > 0) {
          sequence[sequenceAt++] = 3;
        } else {
          sequenceAt = prevSequenceSize;
        }
        Serial.println(sequenceAt);
        for (int c = 0; c < sequenceAt; ++c) {
          Serial.print((int) sequence[c]);
        }
        Serial.println();
        sendSequence(sequence);
        prevSequenceSize = sequenceAt;
        sequenceAt = 0;
      } break;
    }
  }
}

void advanceBit() {
  if (++sequenceAtBit == 8) {
    ++sequenceAt;
    sequenceAtBit = 0;
  }
  if (sequenceAt == sequenceSize) {
    inBinarySequence = false;
  }
}

void binarySequenceLoop() {
  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case '1': {
        sequence[sequenceAt] |= (1 << sequenceAtBit);
        advanceBit();
      } break;
      case '0': {
        sequence[sequenceAt] &= ~(1 << sequenceAtBit);
        advanceBit();
      } break;
      case ';': {
#if 0
        Serial.print("Sending binary ");
        Serial.print(sequenceAt * 8);
        Serial.print("+");
        Serial.print(sequenceAtBit);
        Serial.println(" bits");
#endif        
        //sendBinarySequence(sequence, sequenceAt * 8 + sequenceAtBit);
        sendBinarySequenceHw();
        inBinarySequence = false;
      } break;
      default:
        Serial.println("!");
        inBinarySequence = false;
    }
  }
}

void loop() {
  if (inBinarySequence) {
    binarySequenceLoop();
    if (!inBinarySequence) {
      sequenceAt = 0;
      prevSequenceSize = 0;
    }
  } else {
    baseLoop();
  }
}
