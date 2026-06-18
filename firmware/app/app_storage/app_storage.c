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
typedef struct{
    uint32_t magic;
    bool first_boot;
    bool wifi_credentials_stored;
    bool hostname_stored;
    bool led_configuration_stored;
    bool led_colors_stored;
}Metadata_t;


/*****************************************************************************
* Component Variable Definitions
*****************************************************************************/
static SemaphoreHandle_t xSemaphore = NULL;
static nvs_handle_t nvs;
static Metadata_t metadata = {0};

static const char* storage_type_look_up[STORAGE_TYPE_END] = {
    "metadata",
    "wifi_ssid",
    "wifi_password",
    "host_name",
    "num_of_leds",
    "led_colors",
};


/*****************************************************************************
* Component Function Prototypes
*****************************************************************************/
static bool appStorage_SetDefaultMetadata(Metadata_t* meta, uint32_t timeout_ms);


/*****************************************************************************
* Public Functions
*****************************************************************************/
bool appStorage_Init(void)
{
    esp_err_t err;
    xSemaphore = xSemaphoreCreateMutex();
    if(xSemaphore == NULL)
    {
        EZERROR("Failed to create mutex");
        return false;
    }

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();

        if (err != ESP_OK) {
            EZERROR("Failed to initialize NVS flash: %s", esp_err_to_name(err));
            return false;
        }

        if(appStorage_SetDefaultMetadata(&metadata, 1000) == false)
        {
            EZERROR("Failed to set default metadata");
            return false;
        }

        return true;
    }

    size_t required_size = sizeof(Metadata_t);
    if(appStorage_GetData(STORAGE_TYPE_METADATA, (uint8_t*)&metadata, &required_size, 1000) == false)
    {
        if(appStorage_SetDefaultMetadata(&metadata, 1000) == false)
        {
            EZERROR("Failed to set default metadata");
            return false;
        }
    }
    else if(metadata.magic != 0xDEADBEEF)
    {
        EZWARNING("Metadata magic mismatch, possible data corruption");        
        if(appStorage_SetDefaultMetadata(&metadata, 1000) == false)
        {
            EZERROR("Failed to set default metadata");
            return false;
        }
    }
    else
    {
        EZDEBUG("Metadata loaded successfully");
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
        case STORAGE_TYPE_METADATA:
            err = nvs_get_blob(nvs, storage_type_look_up[type], &metadata, length);
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_WIFI_SSID:
            if(metadata.wifi_credentials_stored == false)
            {
                EZWARNING("Metadata indicates WiFi credentials not stored");
            }
            else
            {
                err = nvs_get_str(nvs, storage_type_look_up[type], (char*)buffer, length);
                if(err == ESP_OK)
                {
                    success = true;
                }
            }
            break;
        case STORAGE_TYPE_WIFI_PASSWORD:
            if(metadata.wifi_credentials_stored == false)
            {
                EZWARNING("Metadata indicates WiFi credentials not stored");
            }
            else
            {
                err = nvs_get_str(nvs, storage_type_look_up[type], (char*)buffer, length);
                if(err == ESP_OK)
                {
                    success = true;
                }
            }
            break;
        case STORAGE_TYPE_HOST_NAME:
            if(metadata.hostname_stored == false)
            {
                EZWARNING("Metadata indicates hostname not stored");
            }
            else
            {
                err = nvs_get_str(nvs, storage_type_look_up[type], (char*)buffer, length);
                if(err == ESP_OK)
                {
                    success = true;
                }
            }
            break;
        case STORAGE_TYPE_NUM_OF_LEDS:
            if(metadata.led_configuration_stored == false)
            {
                EZWARNING("Metadata indicates LED configuration not stored");
            }
            else
            {
                err = nvs_get_u32(nvs, storage_type_look_up[type], (uint32_t*)buffer);
                if(err == ESP_OK)
                {
                    success = true;
                }
            }
            break;
        case STORAGE_TYPE_LED_COLORS:
            if(metadata.led_configuration_stored == false)
            {
                EZWARNING("Metadata indicates LED colors not stored");
            }
            else
            {
                err = nvs_get_blob(nvs, storage_type_look_up[type], buffer, length);
                if(err == ESP_OK)
                {
                    success = true;
                }
            }
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
    bool metadata_updated = false;
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
        case STORAGE_TYPE_METADATA:
            err = nvs_set_blob(nvs, storage_type_look_up[type], &metadata, sizeof(metadata));
            if(err == ESP_OK)
            {
                success = true;
            }
            break;
        case STORAGE_TYPE_WIFI_SSID:
            err = nvs_set_str(nvs, storage_type_look_up[type], (const char*)data);
            if(err == ESP_OK)
            {
                success = true;
                metadata.wifi_credentials_stored = true;
                metadata_updated = true;
            }
            break;
        case STORAGE_TYPE_WIFI_PASSWORD:
            err = nvs_set_str(nvs, storage_type_look_up[type], (const char*)data);
            if(err == ESP_OK)
            {
                success = true;
                metadata.wifi_credentials_stored = true;
                metadata_updated = true;
            }
            break;
        case STORAGE_TYPE_HOST_NAME:
            err = nvs_set_str(nvs, storage_type_look_up[type], (const char*)data);
            if(err == ESP_OK)
            {
                success = true;
                metadata.hostname_stored = true;
                metadata_updated = true;
            }
            break;
        case STORAGE_TYPE_NUM_OF_LEDS:
            err = nvs_set_u32(nvs, storage_type_look_up[type], *(const uint32_t*)data);
            if(err == ESP_OK)
            {
                success = true;
                metadata.led_configuration_stored = true;
                metadata_updated = true;
            }
            break;
        case STORAGE_TYPE_LED_COLORS:
            err = nvs_set_blob(nvs, storage_type_look_up[type], data, length);
            if(err == ESP_OK)
            {
                success = true;
                metadata.led_colors_stored = true;
                metadata_updated = true;
            }
            break;
        default:
            EZWARNING("Invalid storage type");
    }

    if(xSemaphoreGive(xSemaphore) != pdTRUE)
    {
        EZWARNING("Failed to give semaphore");
        return success;
    }

    if(metadata_updated == true)
    {
        appStorage_SetData(STORAGE_TYPE_METADATA, (const uint8_t*)&metadata, sizeof(Metadata_t), 0);
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

static bool appStorage_SetDefaultMetadata(Metadata_t* meta, uint32_t timeout_ms)
{
    if(meta == NULL)
    {
        EZWARNING("Invalid metadata pointer");
        return false;
    }

    meta->magic = 0xDEADBEEF;
    meta->first_boot = true;
    meta->wifi_credentials_stored = false;
    meta->led_configuration_stored = false;
    meta->led_colors_stored = false;
    meta->hostname_stored = false;

    return appStorage_SetData(STORAGE_TYPE_METADATA, (const uint8_t*)meta, sizeof(Metadata_t), timeout_ms);
}
