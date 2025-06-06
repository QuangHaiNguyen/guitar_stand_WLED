#ifndef APP_TOF_H_
#define APP_TOF_H_

#include <stdbool.h>
#include <stdint.h>
#include "ez_event_notifier.h"

#define TOF_SENSOR_I2C_ADDRESS 0x29  // Example I2C address for the TOF sensor

bool appTof_Init(void);


#endif // APP_TOF_H_