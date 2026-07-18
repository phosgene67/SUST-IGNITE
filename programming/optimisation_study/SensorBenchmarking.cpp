#include <Arduino.h>

#define NUM_SENSORS 6

const uint8_t sensorPins[NUM_SENSORS] = {A5, A4, A3, A2, A1, A0};
uint16_t sensorValues[NUM_SENSORS];

#define TEST_COUNT 100

void setup() {
    Serial.begin(115200);

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    Serial.println("Sensor Benchmark");

    // ----------For FASTER ADC---------\\\\\\\\
    // initADC();
    // Serial.println();
    // Serial.println("========== FAST ADC BENCHMARK ==========");
}

void loop() {

    unsigned long totalTime = 0;

    // Benchmark  for   analogRead()
    for (uint16_t n = 0; n < TEST_COUNT; n++) {

        unsigned long start = micros();

        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            sensorValues[i] = analogRead(sensorPins[i]);
        }

        totalTime += micros() - start;
    }
   /*
         // Benchmark for fast adc
    for (uint16_t i = 0; i < TEST_COUNT; i++)
    {
        unsigned long start = micros();

        readSensors();

        totalTime += micros() - start;
    }
        */
    float averageTime = (float)totalTime / TEST_COUNT;

    // Print latest sensor values
    Serial.print("Sensors: ");

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }

    Serial.print(" Avg Read Time: ");
    Serial.print(averageTime, 2);
    Serial.println(" us");

    delay(200);
}


/*
 * ==========================================================
 * FAST ADC SENSOR TEST
 * Arduino Nano (ATmega328P)
 * Reads 6 analog sensors using direct ADC registers
 * Prints sensor values and scan time
 * ==========================================================
 */

//------------------------------------------------------------
// Initialize ADC
//------------------------------------------------------------
void initADC()
{
    // AVcc reference
    ADMUX = (1 << REFS0);
    // Enable ADC
    // Prescaler = 32
    // ADC Clock = 16MHz / 32 = 500kHz
    ADCSRA =
        (1 << ADEN)  |
        (1 << ADPS2) |
        (1 << ADPS0);
}
//------------------------------------------------------------
// Fast ADC Read
//------------------------------------------------------------
inline uint16_t readADC(uint8_t channel)
{
    // Keep reference bits, change only channel
    ADMUX = (ADMUX & 0xF0) | channel;
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait until conversion complete
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void readSensors()
{
    sensorValues[0] = readADC(5);
    sensorValues[1] = readADC(4);
    sensorValues[2] = readADC(3);
    sensorValues[3] = readADC(2);
    sensorValues[4] = readADC(1);
    sensorValues[5] = readADC(0);
}



