#ifndef APP_EVENT_BUS_H_
#define APP_EVENT_BUS_H_

#include <stdbool.h>
#include <stdint.h>
#include "ez_event_notifier.h"

typedef enum{
    APP_EVENT_GPIO_PRESS,
    APP_EVENT_GPIO_LONG_PRESS,
    APP_EVENT_LED_RED_ON,
    APP_EVENT_LED_RED_OFF,
    APP_EVENT_LED_BLUE_ON,
    APP_EVENT_LED_BLUE_OFF,
    APP_EVENT_LED_GREEN_ON,
    APP_EVENT_LED_GREEN_OFF,
    APP_EVENT_NONE
}APP_EVENT_TYPE;


bool appEventBus_Init(void);
ezSTATUS appEventBus_Subscribe(ezObserver *subcriber);
void appEventBus_Notify(uint32_t event_code, void *param1, void *param2);

#endif //APP_EVENT_BUS_H_