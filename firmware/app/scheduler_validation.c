#include "scheduler_validation.h"

#include "FreeRTOS.h"
#include "task.h"

static scheduler_log_entry_t log_entries[SCHEDULER_VALIDATION_LOG_SIZE];
static volatile uint32_t log_count;
static volatile uint32_t high_counter;
static volatile uint32_t low_counter;

static void scheduler_log(scheduler_log_event_t event)
{
    uint32_t index = log_count;

    if (index < SCHEDULER_VALIDATION_LOG_SIZE) {
        log_entries[index].tick = (uint32_t) xTaskGetTickCount();
        log_entries[index].event = (uint32_t) event;
        log_entries[index].high_counter = high_counter;
        log_entries[index].low_counter = low_counter;
        log_count = index + 1U;
    }
}

static void high_priority_validation_task(void *argument)
{
    (void) argument;
    scheduler_log(SCHEDULER_LOG_HIGH_START);

    for (;;) {
        high_counter++;
        scheduler_log(SCHEDULER_LOG_HIGH_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

static void low_priority_validation_task(void *argument)
{
    (void) argument;
    scheduler_log(SCHEDULER_LOG_LOW_START);

    for (;;) {
        low_counter++;
        scheduler_log(SCHEDULER_LOG_LOW_DELAY);
        vTaskDelay(pdMS_TO_TICKS(50U));
    }
}

void scheduler_validation_start(void)
{
    log_count = 0U;
    high_counter = 0U;
    low_counter = 0U;

    if (xTaskCreate(high_priority_validation_task, "sched_high", 256,
        NULL, 4, NULL) != pdPASS) {
        return;
    }

    (void) xTaskCreate(low_priority_validation_task, "sched_low", 256,
        NULL, 2, NULL);
}

uint32_t scheduler_validation_log_count(void)
{
    return log_count;
}

const scheduler_log_entry_t *scheduler_validation_log_get(uint32_t index)
{
    if (index >= log_count || index >= SCHEDULER_VALIDATION_LOG_SIZE) {
        return 0;
    }

    return &log_entries[index];
}