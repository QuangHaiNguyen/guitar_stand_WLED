#ifndef GPIO_PORT_H
#define GPIO_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*gpio_event_callback_t)(uint16_t pin_index, uint32_t pin_state);

bool gpioPort_Init(void);
bool gpioPort_RegisterEventCallback(gpio_event_callback_t callback);

#endif // GPIO_PORT_H