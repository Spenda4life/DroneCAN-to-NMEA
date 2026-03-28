#pragma once
#include <stdint.h>

typedef int portMUX_TYPE;

#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux)      ((void)(mux))
#define portEXIT_CRITICAL(mux)       ((void)(mux))
#define portMAX_DELAY                0xFFFFFFFF

typedef uint32_t TickType_t;
typedef void*    TaskHandle_t;
