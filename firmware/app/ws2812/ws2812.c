/*****************************************************************************
* Includes
*****************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_event_bus.h"
#include "app_storage.h"
#include "app_common.h"
#include "led_strip.h"

#include "driver/spi_common.h"

#define DEBUG_LVL   LVL_DEBUG   /**< logging level */
#define MOD_NAME    "ws2812"       /**< module name */
#include "ez_logging.h"
#include "ez_event_bus.h"
#include "ws2812.h"

/*****************************************************************************
* Component Preprocessor Macros
*****************************************************************************/
/* None */


/*****************************************************************************
* Component Typedefs
*****************************************************************************/
/* None */


/*****************************************************************************
* Component Variable Definitions
*****************************************************************************/
static ezEventListener_t event_listener;
static led_strip_handle_t led_strip_handle;
static size_t num_leds = 0;
static uint8_t led_color_data[WS2812_STRIP_LED_COUNT * 3] = {0};

static int ws2812_OnReceiveEventCallback(uint32_t event_code, const void *data, size_t data_size);
static bool ws2812_ConfigureStrip(void);
static bool ws2812_ApplyColors(const uint8_t *color_data, size_t color_data_size);


/*****************************************************************************
* Public Functions
*****************************************************************************/
bool ws2812_Init(void)
{
    if(!ws2812_ConfigureStrip())
    {
        EZERROR("Failed to configure LED strip");
        return false;
    }

    if(ezEventBus_CreateListener(&event_listener, ws2812_OnReceiveEventCallback) != ezSUCCESS)
    {
        EZERROR("Failed to create event listener");
        return false;
    }

    if(appEventBus_Subscribe(&event_listener) != ezSUCCESS)
    {
        EZERROR("Failed to subscribe to event bus");
        return false;
    }

    size_t len = sizeof(num_leds);
    if(!appStorage_GetData(STORAGE_TYPE_NUM_OF_LEDS, (uint8_t *)&num_leds, &len, 0))
    {
        num_leds = WS2812_STRIP_LED_COUNT;
    }

    len = sizeof(led_color_data);
    if(!appStorage_GetData(STORAGE_TYPE_LED_COLORS, led_color_data, &len, 0))
    {
        memset(led_color_data, 0x00, sizeof(led_color_data));
    }
    return true;
}

uint8_t *ws2812_GetLedColor(size_t *len)
{
    if(len == NULL)
    {
        return NULL;
    }

    *len = num_leds*3;
    return led_color_data;
}

static bool ws2812_ConfigureStrip(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_STRIP_GPIO,
        .max_leds = WS2812_STRIP_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .flags.with_dma = true,
        .spi_bus = SPI2_HOST,
    };
    

    if(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip_handle) != ESP_OK)
    {
        return false;
    }

    if(led_strip_clear(led_strip_handle) != ESP_OK)
    {
        EZERROR("Failed to clear LED strip after initialization");
        return false;
    }

    EZINFO("Configured WS2812 strip on GPIO %d with %d LEDs", WS2812_STRIP_GPIO, WS2812_STRIP_LED_COUNT);
    return true;
}


static bool ws2812_ApplyColors(const uint8_t *color_data, size_t color_data_size)
{
    size_t led_count = 0;
    size_t led_index = 0;

    if((color_data == NULL) || (color_data_size == 0) || ((color_data_size % 3U) != 0U))
    {
        return false;
    }

    if(led_strip_handle == NULL)
    {
        return false;
    }

    led_count = color_data_size / 3U;
    if(led_count > WS2812_STRIP_LED_COUNT)
    {
        EZWARNING("Received %d LEDs but strip supports %d", (int)led_count, WS2812_STRIP_LED_COUNT);
        return false;
    }

    for(led_index = 0; led_index < led_count; led_index++)
    {
        size_t offset = led_index * 3U;
        if(led_strip_set_pixel(led_strip_handle,
                               led_index,
                               color_data[offset],
                               color_data[offset + 1U],
                               color_data[offset + 2U]) != ESP_OK)
        {
            EZERROR("Failed to set pixel %d", (int)led_index);
            return false;
        }
    }

    for(; led_index < WS2812_STRIP_LED_COUNT; led_index++)
    {
        if(led_strip_set_pixel(led_strip_handle, led_index, 0, 0, 0) != ESP_OK)
        {
            EZERROR("Failed to clear pixel %d", (int)led_index);
            return false;
        }
    }

    if(led_strip_refresh(led_strip_handle) != ESP_OK)
    {
        EZERROR("Failed to refresh LED strip");
        return false;
    }

    EZDEBUG("Applied colors to %d LEDs", (int)led_count);

    num_leds = led_count;
    memcpy(led_color_data, color_data, color_data_size);

    if(appStorage_SetData(STORAGE_TYPE_LED_COLORS, color_data, color_data_size, 0) == false)
    {
        EZERROR("Failed to save LED colors to storage");
    }

    if(appStorage_SetData(STORAGE_TYPE_NUM_OF_LEDS, (const uint8_t*)&led_count, sizeof(led_count), 0) == false)
    {
        EZERROR("Failed to save number of LEDs to storage");
    }

    return true;
}


static int ws2812_OnReceiveEventCallback(uint32_t event_code, const void *data, size_t data_size)
{
    switch (event_code)
    {
    case APP_EVENT_WS2812_COLOR_UPDATE:
        EZDEBUG("Received event: APP_EVENT_WS2812_COLOR_UPDATE");
        if((data == NULL) || (data_size == 0))
        {
            EZWARNING("Invalid WS2812 color update data");
            break;
        }

        EZDEBUG("Received WS2812 color size=%d bytes", (int)data_size);
        if(!ws2812_ApplyColors((const uint8_t*)data, data_size))
        {
            EZERROR("Failed to apply WS2812 colors");
        }
        break;
    
    default:
        break;
    }
    return 0;
}