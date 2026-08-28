#ifndef SCHEDULER_VALIDATION_H
#define SCHEDULER_VALIDATION_H

#include <stdint.h>

#define SCHEDULER_VALIDATION_LOG_SIZE 32U

typedef enum {
    SCHEDULER_LOG_HIGH_START = 1U,
    SCHEDULER_LOG_HIGH_DELAY = 2U,
    SCHEDULER_LOG_LOW_START = 3U,
    SCHEDULER_LOG_LOW_DELAY = 4U
} scheduler_log_event_t;

typedef struct {
    uint32_t tick;
    uint32_t event;
    uint32_t high_counter;
    uint32_t low_counter;
} scheduler_log_entry_t;

void scheduler_validation_start(void);
uint32_t scheduler_validation_log_count(void);
const scheduler_log_entry_t *scheduler_validation_log_get(uint32_t index);

#endif