#ifndef APP_GPIO_H_
#define APP_GPIO_H_

#include <stdbool.h>
#include <stdint.h>
#include "ez_event_notifier.h"

typedef enum{
    GPIO_EVENT_SOURCE_BUTTON,
    GPIO_EVENT_SOURCE_NONE,
}GPIO_EVENT_SOURCE;


bool appGpio_Init(void);
ezSTATUS appGpio_SubscribeToEvent(ezObserver *subcriber);

#endif // APP_GPIO_H_