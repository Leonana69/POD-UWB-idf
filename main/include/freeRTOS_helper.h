#ifndef __FREERTOS_HELPER_H__
#define __FREERTOS_HELPER_H__

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "freertos/FreeRTOSConfig.h"


// Static Task Timer
#define TASK_TIMER_DEF(NAME, FREQ) \
    TickType_t NAME##_last_wake = xTaskGetTickCount(); \
    const TickType_t NAME##_period = configTICK_RATE_HZ / (FREQ)

#define TASK_TIMER_WAIT(NAME) \
    vTaskDelayUntil(&NAME##_last_wake, NAME##_period)

// Static Queue
#define STATIC_QUEUE_DEF(NAME, SIZE, TYPE) \
    static QueueHandle_t NAME##_handle; \
    static TYPE NAME##_buffer[SIZE]; \
    static StaticQueue_t NAME##_queue_cb; \
    static void NAME##_queue_init(void) { \
        NAME##_handle = xQueueCreateStatic( \
            SIZE, \
            sizeof(TYPE), \
            (uint8_t*)NAME##_buffer, \
            &NAME##_queue_cb \
        ); \
    }

#define STATIC_QUEUE_INIT(NAME) \
    NAME##_queue_init()

#define STATIC_QUEUE_SEND(NAME, DATA, TIMEOUT) \
    xQueueSend(NAME##_handle, DATA, TIMEOUT)

#define STATIC_QUEUE_RECEIVE(NAME, DATA, TIMEOUT) \
    xQueueReceive(NAME##_handle, DATA, TIMEOUT)

#define STATIC_QUEUE_IS_EMPTY(NAME) \
    xQueueIsQueueEmptyFromISR(NAME##_handle)

// Static Task
#define STATIC_TASK_DEF(NAME, PRIORITY, STACK_SIZE) \
    static void NAME(void *argument); \
    static TaskHandle_t NAME##_handle; \
    static StackType_t NAME##_stack[STACK_SIZE]; \
    static StaticTask_t NAME##_task_cb; \
    static void NAME##_task_init(void *argument) { \
        NAME##_handle = xTaskCreateStatic( \
            NAME,              /* Task function */ \
            #NAME,             /* Name (debug only) */ \
            STACK_SIZE,        /* Stack size in words, not bytes */ \
            argument,          /* Task parameter */ \
            PRIORITY,          /* Task priority */ \
            NAME##_stack,      /* Stack buffer */ \
            &NAME##_task_cb    /* Task control block */ \
        ); \
    }

#define STATIC_TASK_INIT(NAME, ARGUMENT) \
    NAME##_task_init(ARGUMENT)

// Static Mutex
#define STATIC_MUTEX_DEF(NAME) \
    static SemaphoreHandle_t NAME##_handle; \
    static StaticSemaphore_t NAME##_mutex_cb; \
    static void NAME##_mutex_init(void) { \
        NAME##_handle = xSemaphoreCreateMutexStatic(&NAME##_mutex_cb); \
    }

#define STATIC_MUTEX_INIT(NAME) \
    NAME##_mutex_init()

#define STATIC_MUTEX_LOCK(NAME, TIMEOUT) \
    xSemaphoreTake(NAME##_handle, TIMEOUT)

#define STATIC_MUTEX_UNLOCK(NAME) \
    xSemaphoreGive(NAME##_handle)

// Static Semaphore
#define STATIC_SEMAPHORE_DEF(NAME, MAX_COUNT) \
    static SemaphoreHandle_t NAME##_handle; \
    static StaticSemaphore_t NAME##_semaphore_cb; \
    static uint8_t NAME##_semaphore_buffer[MAX_COUNT]; /* optional padding buffer */ \
    static void NAME##_semaphore_init(UBaseType_t init_count) { \
        NAME##_handle = xSemaphoreCreateCountingStatic( \
            MAX_COUNT, \
            init_count, \
            &NAME##_semaphore_cb \
        ); \
    }

#define STATIC_SEMAPHORE_INIT(NAME, MAX_COUNT, INIT_COUNT) \
    NAME##_semaphore_init(MAX_COUNT, INIT_COUNT)

#define STATIC_SEMAPHORE_WAIT(NAME, TIMEOUT) \
    xSemaphoreTake(NAME##_handle, TIMEOUT)

#define STATIC_SEMAPHORE_RELEASE(NAME) \
    xSemaphoreGive(NAME##_handle)

#endif // __FREERTOS_HELPER_H__