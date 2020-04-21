#include <Arduino.h>
#include <TeensyTimerTool.h>
using namespace TeensyTimerTool;

/*
   Cam - Chip - Teensy
    D9 - 6.27 - 21 - MSB
    D8 - 6.26 - 20
    D7 - 6.19 - 15
    D6 - 6.18 - 14
    D5 - 6.17 - 18
    D4 - 6.16 - 19
    D3 - 6.03 - 0
    D2 - 6.02 - 1 - LSB
*/

#define clk 10 // PWM Clock Pin - Timer 1, Channel 0

uint8_t data = 0;
unsigned long count = 0;

elapsedMicros microseconds;
elapsedMillis milliseconds;

Timer t1(TMR1);

int row = 0;
const int height = 24;
int col = 0;
const int width = 32;
uint8_t image[height][width];
bool done = false;

// void writeReg(uint8_t addr, uint8_t val) {}

inline uint8_t read8bitfast() {
    return (GPIO6_PSR & 0b00000000'00000000'00000000'00001100) >> 2  | // 1, 0
           (GPIO6_PSR & 0b00000000'00001111'00000000'00000000) >> 14 | // 19, 18, 14, 15
           (GPIO6_PSR & 0b00001100'00000000'00000000'00000000) >> 20;  // 20, 21
}

void isr() {
    // data = read8bitfast();
    image[row][col] = read8bitfast();
    col++;

    if(col == width) {
        col = 0;
        row++;
        if(row == height) {
            row = 0;
            done = true;
            analogWrite(clk, 0);
        }
    }
    count++;
}

void captureImage() {
    analogWrite(clk, 128);
    while(!done) {
        delayMicroseconds(500);
        Serial.println(done);
    }
    analogWrite(clk, 0);
    done = false;
}

// void displayImage() {}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Serial initialized.");
    t1.beginPeriodic(isr, 1'000'000); // Set up interrupt function
    pinMode(14, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    pinMode(16, INPUT_PULLUP);
    pinMode(17, INPUT_PULLUP);
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(20, INPUT_PULLUP);
    pinMode(21, INPUT_PULLUP);

    // analogWriteFrequency(clk, 6'000'000); // Set interrupt and PWM frequency
    analogWriteFrequency(clk, 6'000); // Set interrupt and PWM frequency
    analogWrite(clk, 0); // Turn off PWM
    Serial.println("Setup done");
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    milliseconds = 0;
    count = 0;
    // while(milliseconds < 1000) {isr();};
    // delay(1000);
    Serial.println("Capturing Image...");
    captureImage();
    // noInterrupts();
    Serial.print("Count: ");
    Serial.println(count);
    Serial.print("Milliseconds: ");
    Serial.println(milliseconds);
    delay(1000);
    // interrupts();
}
