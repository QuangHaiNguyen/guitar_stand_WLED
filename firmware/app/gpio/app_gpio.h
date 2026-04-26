/**
 * @file app_gpio.h
 * @brief Application-level GPIO module for button handling and LED control.
 *
 * This module initialises board GPIOs, debounces the service button,
 * emits button press events on the application event bus, and reacts to LED
 * control events from other modules.
 */

#ifndef APP_GPIO_H_
#define APP_GPIO_H_

#include <stdbool.h>
#include <stdint.h>
#include "ez_event_bus.h"

/**
 * @brief Source identifier attached to GPIO-originated application events.
 */
typedef enum{
    GPIO_EVENT_SOURCE_BUTTON, /**< Event originated from the service button. */
    GPIO_EVENT_SOURCE_NONE,   /**< No valid GPIO event source. */
}GPIO_EVENT_SOURCE;


/**
 * @brief Initialise GPIO hardware, event subscriptions, and button task.
 *
 * Performs GPIO port setup, LED/button pin configuration, event-bus listener
 * registration, and creates the FreeRTOS button debouncing task.
 *
 * @return @c true on success.
 * @return @c false if any GPIO/event initialisation step fails.
 */
bool appGpio_Init(void);

#endif // APP_GPIO_H_