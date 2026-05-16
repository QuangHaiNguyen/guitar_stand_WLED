#ifndef APP_EVENT_BUS_H_
#define APP_EVENT_BUS_H_

#include <stdbool.h>
#include <stdint.h>
#include "ez_event_bus.h"

typedef enum{
    APP_EVENT_GPIO_PRESS,
    APP_EVENT_GPIO_LONG_PRESS,
    APP_EVENT_LED_GREEN,
    APP_EVENT_LED_RED,
    APP_EVENT_LED_BLUE,
    APP_EVENT_WIFI_CONNECTED,
    APP_EVENT_WIFI_DISCONNECTED,
    APP_EVENT_WS2812_COLOR_UPDATE,
    APP_EVENT_NONE
}APP_EVENT_TYPE;


bool appEventBus_Init(void);
ezSTATUS appEventBus_Subscribe(ezEventListener_t *listener);
void appEventBus_Notify(uint32_t event_code, void *event_data, size_t event_data_size);

#endif //APP_EVENT_BUS_H_