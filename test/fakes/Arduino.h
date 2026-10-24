#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static uint32_t fakeUs = 0;
static uint32_t micros() { return fakeUs; }
static uint32_t millis() { return fakeUs / 1000; }
constexpr int SS = 53;
constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
static void pinMode(int, int) {}
static void digitalWrite(int, int) {}
