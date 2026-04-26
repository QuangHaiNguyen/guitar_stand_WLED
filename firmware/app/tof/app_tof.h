/**
 * @file app_tof.h
 * @brief Application-level driver for the VL53L0X Time-of-Flight (ToF) distance sensor.
 *
 * This module initialises the VL53L0X sensor over I2C, performs reference
 * calibration, and spawns a FreeRTOS task that continuously measures distance
 * and publishes LED control events on the application event bus.
 *
 * Distance thresholds:
 *  - < THRESHOLD_1_MM (50 mm)  → blue LED on,  red LED off
 *  - < THRESHOLD_2_MM (90 mm)  → red  LED on,  blue LED off
 *  - >= THRESHOLD_2_MM         → both LEDs off
 */

#ifndef APP_TOF_H_
#define APP_TOF_H_

#include <stdbool.h>
#include <stdint.h>
#include "ez_event_bus.h"

/**
 * @brief Initialise the ToF sensor module.
 *
 * Performs the following sequence:
 *  1. Initialises I2C communication with the VL53L0X at address 0x29.
 *  2. Reads and validates device information (expects product cut 1.1).
 *  3. Runs reference calibration (VHV + phase cal) and SPAD management.
 *  4. Configures single-ranging mode and limit checks.
 *  5. Creates a FreeRTOS task that samples the sensor every 50 ms and
 *     publishes @c APP_EVENT_LED_BLUE / @c APP_EVENT_LED_RED events.
 *
 * @return @c true  on success.
 * @return @c false if calibration or device setup fails.
 */
bool appTof_Init(void);


#endif // APP_TOF_H_