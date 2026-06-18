
#include "app_event_bus.h"

#define DEBUG_LVL   LVL_WARNING   /**< logging level */
#define MOD_NAME    "event bus"       /**< module name */
#include "ez_logging.h"
#include "app_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define BUFF_SIZE 1024

static ezEventBus_t app_event_bus;
static uint8_t event_buff[BUFF_SIZE];
static StaticSemaphore_t app_event_bus_mutex_storage;
static SemaphoreHandle_t app_event_bus_mutex;

static void appEventBus_Task(void* arg);

bool appEventBus_Init(void)
{
    app_event_bus_mutex = xSemaphoreCreateMutexStatic(&app_event_bus_mutex_storage);
    if(app_event_bus_mutex == NULL)
    {
        EZERROR("Failed to create event bus mutex");
        return false;
    }

    if(ezEventBus_CreateBus(&app_event_bus, event_buff, BUFF_SIZE) != ezSUCCESS)
    {
        EZERROR("Failed to create event bus");
        return false;
    }

    xTaskCreate(appEventBus_Task,
        "event bus task",
        2048, NULL,
        1, NULL);

    return true;
}


ezSTATUS appEventBus_Subscribe(ezEventListener_t *listener)
{
    ezSTATUS status = ezFAIL;

    if((listener == NULL) || (app_event_bus_mutex == NULL))
    {
        return ezSTATUS_ARG_INVALID;
    }

    if(xSemaphoreTake(app_event_bus_mutex, portMAX_DELAY) == pdTRUE)
    {
        status = ezEventBus_Listen(&app_event_bus, listener);
        xSemaphoreGive(app_event_bus_mutex);
    }

    return status;
}


void appEventBus_Notify(uint32_t event_code, void *event_data, size_t event_data_size)
{
    EZDEBUG("Notify event code %d, data size %d", event_code, event_data_size);
    if(app_event_bus_mutex == NULL)
    {
        EZERROR("Event bus mutex is not initialized");
        return;
    }

    if(xSemaphoreTake(app_event_bus_mutex, portMAX_DELAY) != pdTRUE)
    {
        EZERROR("Failed to lock event bus for event=%d", event_code);
        return;
    }

    if(ezEventBus_SendEvent(&app_event_bus, event_code, event_data, event_data_size) == false)
    {
        EZERROR("Failed to send event=%d", event_code);
    }

    xSemaphoreGive(app_event_bus_mutex);
}

static void appEventBus_Task(void* arg)
{
    (void)arg;
    while(1)
    {
        if((app_event_bus_mutex != NULL) && (xSemaphoreTake(app_event_bus_mutex, portMAX_DELAY) == pdTRUE))
        {
            if(ezEventBus_Run(&app_event_bus) != ezSUCCESS)
            {
                EZERROR("Failed to run event bus");
            }

            xSemaphoreGive(app_event_bus_mutex);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}