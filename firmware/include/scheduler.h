#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <pico/time.h>
#include <pico/types.h>
#include <stdint.h>

#include "error.h"

#define MAX_NUM_TASKS 5

typedef void (*scheduler_task_cb)();

typedef struct {
    uint8_t priority;
    absolute_time_t period;  // us
    scheduler_task_cb callback;
    absolute_time_t next_tick;
    uint64_t total_exec_time_us;
    uint32_t exec_count;
    uint64_t last_exec_time_us;
} Task_t;

typedef struct {
    uint64_t total_busy_time_us;
    uint64_t total_idle_time_us;
    absolute_time_t measurement_start;
} scheduler_stats_t;

ErrorCode_e scheduler_init(void);
ErrorCode_e scheduler_add_task(uint32_t period, uint8_t priority, scheduler_task_cb cb);
void scheduler_remove_task();
void scheduler_run();
float scheduler_get_cpu_usage(void);
void scheduler_print_stats(void);
void scheduler_reset_stats(void);

#endif  // !SCHEDULER_H
