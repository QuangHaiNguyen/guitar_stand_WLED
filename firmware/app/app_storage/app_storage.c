/*****************************************************************************
* Includes
*****************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nvs_flash.h"


#define DEBUG_LVL   LVL_DEBUG   /**< logging level */
#define MOD_NAME    "storage"       /**< module name */
#include "ez_logging.h"
#include "app_storage.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/*****************************************************************************
* Component Preprocessor Macros
*****************************************************************************/
#define APP_STORAGE_NVS_NAMESPACE      "wifi_creds"


/*****************************************************************************
* Component Typedefs
*****************************************************************************/
/* None */

/*****************************************************************************
* Component Variable Definitions
*****************************************************************************/
static SemaphoreHandle_t xSemaphore = NULL;
static nvs_handle_t nvs;

static const char* storage_type_look_up[STORAGE_TYPE_END] = {
    "wifi_ssid",
    "wifi_password",
    "host_name",
    "num_of_leds",
    "led_colors",
};


/*****************************************************************************
* Public Functions
*****************************************************************************/
bool appStorage_Init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        EZERROR("Failed to initialize NVS flash: %s", esp_err_to_name(err));
        return false;
    }

    xSemaphore = xSemaphoreCreateMutex();
    if(xSemaphore == NULL)
    {
        EZERROR("Failed to create mutex");
        return false;
    }
    
    return true;
}

bool appStorage_GetData(STORAGE_TYPE type, uint8_t* buffer, size_t* length, uint32_t timeout_ms)
{
    esp_err_t err = ESP_OK;
    bool success = false;
     
    if(buffer == NULL || length == NULL || *length == 0)
    {
        EZWARNING("Invalid buffer or length");
        return success;
    }

    if(xSemaphore == NULL)
    {
        EZERROR("Storage not initialized");
        return success;
    }

    if(xSemaphoreTake(xSemaphore, (TickType_t)(timeout_ms/portTICK_PERIOD_MS)) != pdTRUE )
    {
        EZWARNING("Failed to take semaphore");
        return success;
    }

    err = nvs_open(APP_STORAGE_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if(err != ESP_OK)
    {
        EZERROR("Failed to open NVS namespace: %s", esp_err_to_name(err));
        xSemaphoreGive(xSemaphore);
        return success;
    }

    switch(type)
    {
        case STORAGE_TYPE_WIFI_SSID:
            err = nvs_get_str(nvs, storage_type_look_up[type], (char*)buffer, length);
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_WIFI_PASSWORD:
            err = nvs_get_str(nvs, storage_type_look_up[type], (char*)buffer, length);
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_HOST_NAME:
            err = nvs_get_str(nvs, storage_type_look_up[type], (char*)buffer, length);
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_NUM_OF_LEDS:
            break;
        case STORAGE_TYPE_LED_COLORS:
            break;
        default:
            EZWARNING("Invalid storage type");
    }

    nvs_close(nvs);

    if(xSemaphoreGive(xSemaphore) != pdTRUE)
    {
        EZWARNING("Failed to give semaphore");
        return success;
    }

    return success;
}


bool appStorage_SetData(STORAGE_TYPE type, const uint8_t* data, size_t length, uint32_t timeout_ms)
{
    esp_err_t err = ESP_OK;
    bool success = false;

    if(data == NULL || length == 0)
    {
        EZWARNING("Invalid data or length");
        return success;
    }

    if(xSemaphore == NULL)
    {
        EZERROR("Storage not initialized");
        return success;
    }

    if(xSemaphoreTake(xSemaphore, (TickType_t)(timeout_ms/portTICK_PERIOD_MS)) != pdTRUE )
    {
        EZWARNING("Failed to take semaphore");
        return success;
    }

    err = nvs_open(APP_STORAGE_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if(err != ESP_OK)
    {
        EZERROR("Failed to open NVS namespace: %s", esp_err_to_name(err));
        xSemaphoreGive(xSemaphore);
        return success;
    }

    switch(type)
    {
        case STORAGE_TYPE_WIFI_SSID:
            err = nvs_set_str(nvs, storage_type_look_up[type], (const char*)data);
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_WIFI_PASSWORD:
            err = nvs_set_str(nvs, storage_type_look_up[type], (const char*)data);
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_HOST_NAME:
            err = nvs_set_str(nvs, storage_type_look_up[type], (const char*)data);
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_NUM_OF_LEDS:
            break;
        case STORAGE_TYPE_LED_COLORS:
            break;
        default:
            EZWARNING("Invalid storage type");
    }

    if(xSemaphoreGive(xSemaphore) != pdTRUE)
    {
        EZWARNING("Failed to give semaphore");
        return success;
    }

    return success;
}


bool appStorage_DeleteData(STORAGE_TYPE type, uint32_t timeout_ms)
{
    if(xSemaphore == NULL)
    {
        EZERROR("Storage not initialized");
        return false;
    }

    if(xSemaphoreTake(xSemaphore, (TickType_t)(timeout_ms/portTICK_PERIOD_MS)) != pdTRUE )
    {
        EZWARNING("Failed to take semaphore");
        return false;
    }

    switch(type)
    {
        case STORAGE_TYPE_WIFI_SSID:
            break;
        case STORAGE_TYPE_WIFI_PASSWORD:
            break;
        case STORAGE_TYPE_HOST_NAME:
            break;
        case STORAGE_TYPE_NUM_OF_LEDS:
            break;
        case STORAGE_TYPE_LED_COLORS:
            break;
        default:
            EZWARNING("Invalid storage type");
            return false;
    }

    if(xSemaphoreGive(xSemaphore) != pdTRUE)
    {
        EZWARNING("Failed to give semaphore");
        return false;
    }

    return true;
}