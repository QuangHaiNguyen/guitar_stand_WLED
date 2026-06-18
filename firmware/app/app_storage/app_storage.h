#ifndef _APP_STORAGE_H_
#define _APP_STORAGE_H_

/**
 * @brief Identifies the type of data to store or retrieve in persistent storage.
 */
typedef enum{
    STORAGE_TYPE_METADATA,
    STORAGE_TYPE_WIFI_SSID,
    STORAGE_TYPE_WIFI_PASSWORD,
    STORAGE_TYPE_HOST_NAME,
    STORAGE_TYPE_NUM_OF_LEDS,
    STORAGE_TYPE_LED_COLORS,
    STORAGE_TYPE_END,
}STORAGE_TYPE;

/**
 * @brief Initializes the persistent storage subsystem.
 *
 * Must be called once before any other appStorage_* functions.
 *
 * @return true if initialization succeeded, false otherwise.
 */
bool appStorage_Init(void);

/**
 * @brief Retrieves data of the specified type from persistent storage.
 *
 * @param type[in]       The storage entry to read.
 * @param buffer[out]    Caller-allocated buffer to receive the data.
 * @param length[out]    On entry, the size of @p buffer in bytes; on exit,
 *                       the number of bytes actually written into @p buffer.
 * @param timeout_ms[in] Maximum time in milliseconds to wait for the storage
 *                       mutex before returning failure.
 * @return true if data was read successfully, false otherwise.
 */
bool appStorage_GetData(STORAGE_TYPE type, uint8_t* buffer, size_t* length, uint32_t timeout_ms);

/**
 * @brief Writes data of the specified type to persistent storage.
 *
 * @param type[in]       The storage entry to write.
 * @param data[in]       Pointer to the data to store.
 * @param length[in]     Number of bytes in @p data.
 * @param timeout_ms[in] Maximum time in milliseconds to wait for the storage
 *                       mutex before returning failure.
 * @return true if data was written successfully, false otherwise.
 */
bool appStorage_SetData(STORAGE_TYPE type, const uint8_t* data, size_t length, uint32_t timeout_ms);

/**
 * @brief Deletes the stored entry of the specified type from persistent storage.
 *
 * @param type[in]       The storage entry to delete.
 * @param timeout_ms[in] Maximum time in milliseconds to wait for the storage
 *                       mutex before returning failure.
 * @return true if the entry was deleted successfully, false otherwise.
 */
bool appStorage_DeleteData(STORAGE_TYPE type, uint32_t timeout_ms);


#endif /* _APP_STORAGE_H_ */