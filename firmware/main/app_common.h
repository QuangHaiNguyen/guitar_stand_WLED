#ifndef _APP_COMMON_H_
#define _APP_COMMON_H_


#define LED_RED             3
#define LED_BLUE            5
#define LED_GREEN           4
#define SERVICE_BUTTON      9

#define WS2812_STRIP_GPIO       10
#define WS2812_STRIP_LED_COUNT  8
#define WS2812_RMT_RES_HZ  (10 * 1000 * 1000)

#define GPIO_DRIVER_NAME    "ESP32_GPIO"
#define I2C_DRIVER_NAME     "ESP32_I2C"

#endif // _APP_COMMON_H_