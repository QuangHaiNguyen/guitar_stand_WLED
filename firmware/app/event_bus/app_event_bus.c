
#include "app_event_bus.h"

#define DEBUG_LVL   LVL_WARNING   /**< logging level */
#define MOD_NAME    "event bus"       /**< module name */
#include "ez_logging.h"
#include "app_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define BUFF_SIZE 1024

static ezEventBus_t app_event_bus;
static uint8_t event_buff[BUFF_SIZE];

static void appEventBus_Task(void* arg);

bool appEventBus_Init(void)
{
    if(ezEventBus_CreateBus(&app_event_bus, event_buff, BUFF_SIZE) != ezSUCCESS)
    {
        EZERROR("Failed to create event bus");
        return false;
    }

    xTaskCreate(appEventBus_Task,
        "event bus task",
        1024, NULL,
        1, NULL);

    return true;
}


ezSTATUS appEventBus_Subscribe(ezEventListener_t *listener)
{
    return ezEventBus_Listen(&app_event_bus, listener);
}


void appEventBus_Notify(uint32_t event_code, void *event_data, size_t event_data_size)
{
    EZDEBUG("Notify event code %d, data size %d", event_code, event_data_size);
    if(ezEventBus_SendEvent(&app_event_bus, event_code, event_data, event_data_size) == false)
    {
        EZERROR("Failed to send event");
    }
}

static void appEventBus_Task(void* arg)
{
    (void)arg;
    while(1)
    {
        if(ezEventBus_Run(&app_event_bus) != ezSUCCESS)
        {
            EZERROR("Failed to run event bus");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}