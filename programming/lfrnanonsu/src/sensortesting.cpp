// =============================================================
//  ARDUINO NANO - 16-CHANNEL MUX RAW READ / SERIAL MONITOR TEST
// =============================================================
//  Purpose: read all 16 channels of the analog mux (CD74HC4067-
//  style) and print their raw ADC values to the Serial Monitor.
//  No motors, no PID, no calibration.
//
//  This matches the PID file's hardware EXACTLY:
//    - SIG -> A0 (single shared ADC input, fixed on ADC0 forever)
//    - S0-S3 -> 4 address pins on PORTC, same pins as the PID file
//    - MuxSelectChannel() is copied verbatim from the PID sketch
//
//  HOW TO USE THIS SKETCH:
//  Only 8 of the 16 physical mux inputs have sensors wired to
//  them; the rest will just read floating/noise values. Upload
//  this, open Serial Monitor, and cover each physical sensor by
//  hand one at a time. Watch which of the 16 printed columns (CH0
//  ... CH15) dips/rises in response. Write down those 8 channel
//  numbers in physical left-to-right order - that's exactly the
//  list that goes into sensorMuxChannel[] in the PID sketch.
// =============================================================

#include <Arduino.h>

// ================= PIN DEFINITIONS (16-ch mux) ================
// Identical to the PID sketch - same SIG pin, same address pins,
// same bit-to-select mapping. Keeping these identical is what
// makes the channel numbers you read here valid there too.
#define MUX_SIG A0
#define MUX_S0  A4   // PC4 - address bit 0
#define MUX_S1  A3   // PC3 - address bit 1
#define MUX_S2  A2   // PC2 - address bit 2
#define MUX_S3  A1   // PC1 - address bit 3
#define MUX_SELECT_BITS 0x1E   // PC1|PC2|PC3|PC4 - the 4 mux address bits on PORTC
#define MUX_SETTLE_US 10       // RC settle time after switching address - do not remove

#define NUM_SENSORS 8

const uint8_t sensorChannels[NUM_SENSORS] = {
  0, 2, 4, 6, 8, 10, 12, 14
};

uint16_t sensorValue[NUM_SENSORS];
#define PRINT_INTERVAL_MS 150   // how often to print a new line of readings
uint32_t lastPrintTime = 0;

// ================= MUX ADDRESS SELECT (same as PID sketch) =====
// One PORTC write per channel switch, instead of 4x digitalWrite()
// calls - digitalWrite is too slow to call repeatedly per scan on
// the real PID hot path, so this sketch mirrors it for consistency,
// even though timing doesn't matter for a Serial print test.
static inline void MuxSelectChannel(uint8_t ch) __attribute__((always_inline));
static inline void MuxSelectChannel(uint8_t ch) {
  uint8_t bits = 0;
  if (ch & 0x01) bits |= (1 << PC4);   // S0
  if (ch & 0x02) bits |= (1 << PC3);   // S1
  if (ch & 0x04) bits |= (1 << PC2);   // S2
  if (ch & 0x08) bits |= (1 << PC1);   // S3
  PORTC = (PORTC & ~MUX_SELECT_BITS) | bits;
}

// ================= FUSED MUX-SELECT + READ ======================
// Same technique as SENSOR_STEP() in the PID sketch: switch the mux
// address, wait for its RC settle, then convert on the fixed ADC0
// input. The ADC itself never changes channel - only the mux does.
static inline uint16_t ReadMuxChannelRaw(uint8_t ch) {
  MuxSelectChannel(ch);
  delayMicroseconds(MUX_SETTLE_US);
  ADCSRA |= (1 << ADSC);          // start conversion
  while (ADCSRA & (1 << ADSC));   // wait for it to finish
  return ADC;
}

void ReadAllChannels() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorValue[i] = ReadMuxChannelRaw(sensorChannels[i]);
  }
}
void PrintChannelValues() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print("CH");
    Serial.print(sensorChannels[i]);   // Prints CH0 CH2 CH4 ...
    Serial.print(":");
    Serial.print(sensorValue[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  // MUX_SIG (A0) stays a plain ADC input; mux EN pin is assumed
  // hardwired to GND (always enabled), same assumption as the PID sketch.

  ADMUX = (1 << REFS0) | 0;   // fixed on ADC0 (A0/SIG) forever - the mux switches channels, not us
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // ADC on, prescaler 64

  Serial.println("16-channel mux raw-value test starting...");
  Serial.println("Cover each physical sensor and note which CHx column responds.");
}

void loop() {
  uint32_t now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    ReadAllChannels();
    PrintChannelValues();
  }
}
