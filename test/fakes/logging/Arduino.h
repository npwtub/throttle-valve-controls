#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <string>

using String = std::string;
using __FlashStringHelper = char;
#define F(value) value
#ifndef A2
constexpr int A2 = 56;
#endif
#ifndef A3
constexpr int A3 = 57;
#endif
#ifndef A4
constexpr int A4 = 58;
#endif
constexpr int SS = 53, OUTPUT = 1, HIGH = 1;
static uint32_t fakeUs = 1000000;
static int fakeAnalog[64] = {};
static unsigned fakeAnalogReads[64] = {};
inline uint32_t micros() { return fakeUs; }
inline uint32_t millis() { return fakeUs / 1000; }
inline int analogRead(int pin) { ++fakeAnalogReads[pin]; return fakeAnalog[pin]; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
template<class T> T constrain(T value, T minimum, T maximum) {
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

class Print {
public:
    template<class T> void print(T) {}
    template<class T> void print(T, int) {}
    template<class T> void println(T) {}
};
class FakeSerial : public Print {
public:
    void begin(unsigned long) {}
    int available() { return 0; }
    int read() { return -1; }
};
static FakeSerial Serial;
