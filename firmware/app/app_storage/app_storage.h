#ifndef _APP_STORAGE_H_
#define _APP_STORAGE_H_

typedef enum{
    STORAGE_TYPE_WIFI_SSID,
    STORAGE_TYPE_WIFI_PASSWORD,
    STORAGE_TYPE_HOST_NAME,
    STORAGE_TYPE_NUM_OF_LEDS,
    STORAGE_TYPE_LED_COLORS,
    STORAGE_TYPE_END,
}STORAGE_TYPE;


bool appStorage_Init(void);
bool appStorage_GetData(STORAGE_TYPE type, uint8_t* buffer, size_t* length, uint32_t timeout_ms);
bool appStorage_SetData(STORAGE_TYPE type, const uint8_t* data, size_t length, uint32_t timeout_ms);
bool appStorage_DeleteData(STORAGE_TYPE type, uint32_t timeout_ms);


#endif /* _APP_STORAGE_H_ */