#pragma once
#include <stdint.h>
#include <string.h>

static inline uint32_t millis() { return 1000; }

#define HIGH 1
#define LOW  0
#define OUTPUT 1

static inline void pinMode(int, int) {}
static inline void digitalWrite(int, int) {}
