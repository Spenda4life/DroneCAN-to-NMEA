#pragma once
static inline int esp_task_wdt_init(int, bool) { return 0; }
static inline int esp_task_wdt_add(void*)       { return 0; }
static inline int esp_task_wdt_reset()           { return 0; }
