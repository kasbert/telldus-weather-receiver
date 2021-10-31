
/*
   Manchester receiver for 433MHz radio sensors.
   Pulses are 500us and 1000us

   Received data is sent to the host using hex strings to serial port
   Each message consists of
   * 2 byte length of bits (N)
   * 1 space
   * (N + 7)/8 bytes data
   * 1 space
   * 1 byte crc8(polynomial=0x8c, init = 0)
   * 2 byte line end "\r\n"

   First received bit is the most significant bit in the byte.

   For example
   0076 5320502C4494015E0D9173FFEFEC28 B4
     0x76 = 118 bits = 15 bytes (-2 bits)
     data:
     01010011 00100000 01010000 00101100 01000100 10010100 00000001 01011110
     00001101 10010001 01110011 11111111 11101111 11101100 001010ZZ
     The last byte has 2 filler zero bits (Z) as the lowest bits (0x03).
     crc is used to detect possible errors in serial communication.
*/


const int rxPin = 8;

#define IS_SHORT(x) ((x >= 300) && (x <= 750))
#define IS_LONG(x) ((x > 750) && (x <= 1150))
#define MAX_PULSE 2000 // us
#define MAX_DATA 256 // bytes


// States
#define NOSYNC0 0 // Have not received a long pulse
#define NOSYNC8 8 // Received 8 short pulses in a row
#define T1 50 // In sync, short received
#define T2 51 // In sync, long received

void setup() {
  pinMode(rxPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Manchester receiver v0.1");
}

static void emit_bit(unsigned char *data, unsigned int *outcount, int level) {
  int bitindex = (*outcount) & 7;
  int byteindex = (*outcount) / 8;
  if (!bitindex) {
    data[byteindex] = 0;
  }
  bitindex = 7 - bitindex;
  if (level) {
    data[byteindex] |= 1 << bitindex;
  } else {
    data[byteindex] &= ~(1 << bitindex);
  }
  (*outcount)++;
}

// Return new state
unsigned char manchester_decode(unsigned long pulse, int level, unsigned char *data, unsigned int *outcount, unsigned char state) {
  int is_long = IS_LONG(pulse);
  int is_short = IS_SHORT(pulse);

  switch (state) {
    case T2: // In sync
      emit_bit(data, outcount, level);
      if (is_short) {
        return T1;
      }
      if (!is_long) {
        return NOSYNC0; // Error, sync dropped
      }
      return T2;

    case T1: // Previous was short
      if (is_long) {
        return NOSYNC0; // Sync dropped
      }
      if (!is_short) {
        return NOSYNC0; // Error, sync dropped
      }
      return T2;

    case NOSYNC8:
      if (is_short) {
        return state; // Go on, wait for long pulse
      }
      if (!is_long) {
        return NOSYNC0; // Error, sync dropped
      }
      return T2;

    default: // NOSYNC < 8
      if (is_short) {
        return state + 1; // Count 8 short pulses
      }
      return NOSYNC0; // No sync, need 8 shorts first
  }
  return NOSYNC0;
}


uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
  uint8_t crc = init;
  unsigned byte, bit;

  for (byte = 0; byte < nBytes; ++byte) {
    crc ^= message[byte];
    for (bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ polynomial;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

void handle_data(uint8_t *data, const unsigned int bitcount)
{
  if (bitcount < 16) {
    // Not interested in short messages
    return;
  }
  if (bitcount < 0x1000) {
    Serial.print('0');
  }
  if (bitcount < 0x100) {
    Serial.print('0');
  }
  if (bitcount < 0x10) {
    Serial.print('0');
  }
  Serial.print(bitcount, HEX);
  Serial.print(' ');
  int bytecount = (bitcount + 7) / 8;
  for (int i = 0; i < bytecount; i++) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
  }
  uint8_t crc = crc8(data, bytecount, 0x8C, 0);
  Serial.print(' ');
  if (crc < 0x10) {
    Serial.print('0');
  }
  Serial.print(crc, HEX);
  Serial.println("");
}

void loop() {
  unsigned int bitcount = 0;
  uint8_t data[MAX_DATA];

  unsigned char state = NOSYNC0;
  unsigned long t = 0;
  unsigned long prev_us = micros();
  int prev_rx = digitalRead(rxPin);
  if (prev_rx) {
    // Start with high, skip one ?
    // state = manchester_decode (500, level, data, &bitcount, state);
  }

  do
  {
    int rx;
    unsigned long us;
    do
    {
      rx = digitalRead(rxPin);
      us = micros();
      t = us - prev_us;
    } while (prev_rx == rx && t < MAX_PULSE);
    prev_us = us;
    prev_rx = rx;
    if (t >= MAX_PULSE) rx = ~rx;

    state = manchester_decode (t, rx, data, &bitcount, state);
    digitalWrite(LED_BUILTIN, state < T1 ? LOW : HIGH);

    if ((bitcount >= sizeof(data) * 8) // Data buffer overflow
      || (state < T1 && bitcount)) {
      // Sync is lost, handle the data so far
      handle_data(data, bitcount);
      bitcount = 0;
    }
  } while (1 /*|| IS_SHORT(t) || IS_LONG(t)*/);

}
