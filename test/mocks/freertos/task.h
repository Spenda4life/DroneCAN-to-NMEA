#pragma once
#include "FreeRTOS.h"

static inline void vTaskDelay(int) {}

typedef void (*TaskFunction_t)(void*);

static inline int xTaskCreatePinnedToCore(TaskFunction_t, const char*, int, void*, int, TaskHandle_t*, int) { return 1; }
