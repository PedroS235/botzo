#include "scheduler.h"

#include <pico/time.h>
#include <pico/types.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include "error.h"

Task_t tasks[MAX_NUM_TASKS] = {0};
uint8_t num_of_tasks = 0;
scheduler_stats_t stats = {0};

ErrorCode_e scheduler_init() {
    num_of_tasks = 0;
    stats.total_busy_time_us = 0;
    stats.total_idle_time_us = 0;
    stats.measurement_start = get_absolute_time();

    return OK;
}

ErrorCode_e scheduler_add_task(uint32_t period,
                               uint8_t priority,
                               scheduler_task_cb cb) {
    if (num_of_tasks >= MAX_NUM_TASKS) return ERR_SCHEDULER_MAX_TASKS_REACHED;

    tasks[num_of_tasks].period = period * 1000;
    tasks[num_of_tasks].priority = priority;
    tasks[num_of_tasks].callback = cb;
    tasks[num_of_tasks].next_tick = get_absolute_time();
    tasks[num_of_tasks].total_exec_time_us = 0;
    tasks[num_of_tasks].exec_count = 0;
    tasks[num_of_tasks].last_exec_time_us = 0;
    num_of_tasks++;

    for (int i = num_of_tasks - 1; i > 0 && tasks[i].priority > tasks[i - 1].priority;
         i--) {
        Task_t temp = tasks[i];
        tasks[i] = tasks[i - 1];
        tasks[i - 1] = temp;
    }

    return OK;
}

void scheduler_run() {
    absolute_time_t loop_start = get_absolute_time();
    absolute_time_t now = loop_start;
    bool any_task_ran = false;

    for (int i = 0; i < num_of_tasks; i++) {
        if (now >= tasks[i].next_tick) {
            absolute_time_t task_start = get_absolute_time();
            tasks[i].callback();
            absolute_time_t task_end = get_absolute_time();

            uint64_t exec_time = absolute_time_diff_us(task_start, task_end);

            tasks[i].last_exec_time_us = exec_time;
            tasks[i].total_exec_time_us += exec_time;
            tasks[i].exec_count++;

            stats.total_busy_time_us += exec_time;

            tasks[i].next_tick = tasks[i].next_tick + tasks[i].period;
            any_task_ran = true;

            now = get_absolute_time();
        }
    }

    if (!any_task_ran) {
        absolute_time_t loop_end = get_absolute_time();
        stats.total_idle_time_us += absolute_time_diff_us(loop_start, loop_end);
    }
}

float scheduler_get_cpu_usage(void) {
    uint64_t total_time = stats.total_busy_time_us + stats.total_idle_time_us;
    if (total_time == 0) return 0.0f;
    return (stats.total_busy_time_us * 100.0f) / total_time;
}

void scheduler_print_stats(void) {
    printf("=== Scheduler Stats ===\n");
    printf("CPU Usage: %.2f%%\n", scheduler_get_cpu_usage());
    printf("\nPer-task breakdown:\n");
    for (int i = 0; i < num_of_tasks; i++) {
        if (tasks[i].exec_count > 0) {
            uint64_t avg = tasks[i].total_exec_time_us / tasks[i].exec_count;
            printf("Task %d: avg=%lluus, last=%lluus, count=%u\n",
                   i, avg, tasks[i].last_exec_time_us, tasks[i].exec_count);
        }
    }
}

void scheduler_reset_stats(void) {
    stats.total_busy_time_us = 0;
    stats.total_idle_time_us = 0;
    stats.measurement_start = get_absolute_time();
    for (int i = 0; i < num_of_tasks; i++) {
        tasks[i].total_exec_time_us = 0;
        tasks[i].exec_count = 0;
    }
}
