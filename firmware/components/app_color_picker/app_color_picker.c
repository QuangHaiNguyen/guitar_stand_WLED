
/*****************************************************************************
* Includes
*****************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_color_picker.h"
#include "app_event_bus.h"
#include "esp_err.h"
#include "esp_http_server.h"

#define DEBUG_LVL   LVL_DEBUG   /**< logging level */
#define MOD_NAME    "color_picker"       /**< module name */
#include "ez_logging.h"
#include "ez_event_bus.h"


#define COLOR_PICKER_MAX_LEDS     50
#define COLOR_PICKER_MAX_BODY_LEN (COLOR_PICKER_MAX_LEDS * 3)


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
static httpd_handle_t http_server_h = NULL;
static bool is_webserver_started = false;

extern const char color_picker_start[] asm("_binary_color_picker_html_start");
extern const char color_picker_end[] asm("_binary_color_picker_html_end");

int appColorPicker_OnReceiveEventCallback(uint32_t event_code, const void *data, size_t data_size);
static bool appColorPicker_StartWebserver(httpd_handle_t handle);
static esp_err_t appColorPicker_GetPageHandler(httpd_req_t *req);
static esp_err_t appColorPicker_PostColorsHandler(httpd_req_t *req);
static bool appColorPicker_ReadBody(httpd_req_t *req, uint8_t *buffer, int buffer_len);

static const httpd_uri_t http_get_color_picker = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = appColorPicker_GetPageHandler
};

static const httpd_uri_t http_post_led_colors = {
    .uri = "/api/led/colors",
    .method = HTTP_POST,
    .handler = appColorPicker_PostColorsHandler
};

/*****************************************************************************
* Public Functions
*****************************************************************************/
bool appColorPicker_Init(void)
{
    if(ezEventBus_CreateListener(&event_listener, appColorPicker_OnReceiveEventCallback) != ezSUCCESS)
    {
        EZERROR("Failed to create event listener");
        return false;
    }

    if(appEventBus_Subscribe(&event_listener) != ezSUCCESS)
    {
        EZERROR("Failed to subscribe to event bus");
        return false;
    }

    EZINFO("appColorPicker_Init() success");
    return true;
}


/*****************************************************************************
* Local Functions
*****************************************************************************/
static bool appColorPicker_StartWebserver(httpd_handle_t handle)
{
    esp_err_t err = ESP_OK;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    // Start the httpd server
    EZDEBUG("Starting server on port: '%d'", config.server_port);
    if (httpd_start(&handle, &config) != ESP_OK) {
        EZERROR("Failed to start HTTP server");
        return false;
    }

    err = httpd_register_uri_handler(handle, &http_get_color_picker);
    if((err != ESP_OK) && (err != ESP_ERR_HTTPD_HANDLER_EXISTS))
    {
        EZERROR("Failed to register /: %s", esp_err_to_name(err));
        return false;
    }

    err = httpd_register_uri_handler(handle, &http_post_led_colors);
    if((err != ESP_OK) && (err != ESP_ERR_HTTPD_HANDLER_EXISTS))
    {
        EZERROR("Failed to register /api/led/colors: %s", esp_err_to_name(err));
        return false;
    }

    is_webserver_started = true;
    EZINFO("Color picker webserver started");
    return true;
}


static esp_err_t appColorPicker_GetPageHandler(httpd_req_t *req)
{
    const size_t page_len = (size_t)(color_picker_end - color_picker_start);

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, color_picker_start, page_len);
    return ESP_OK;
}


static bool appColorPicker_ReadBody(httpd_req_t *req, uint8_t *buffer, int buffer_len)
{
    int recv_len = 0;
    int total = 0;

    if((req == NULL) || (buffer == NULL) || (buffer_len <= 0))
    {
        return false;
    }

    if(req->content_len > buffer_len)
    {
        return false;
    }

    while(total < req->content_len)
    {
        recv_len = httpd_req_recv(req, (char *)(buffer + total), req->content_len - total);
        if(recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        {
            continue;
        }
        if(recv_len <= 0)
        {
            return false;
        }
        total += recv_len;
    }

    return total == req->content_len;
}


static esp_err_t appColorPicker_PostColorsHandler(httpd_req_t *req)
{
    uint8_t payload[COLOR_PICKER_MAX_BODY_LEN] = {0};
    int32_t led_count = 0;

    if((req->content_len <= 0) || (req->content_len > COLOR_PICKER_MAX_BODY_LEN))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }

    if((req->content_len % 3) != 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload must be RGB byte stream");
        return ESP_FAIL;
    }

    if(!appColorPicker_ReadBody(req, payload, sizeof(payload)))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }

    led_count = req->content_len / 3;

    // TODO: Apply payload bytes to WS2812 driver/task: payload = RGBRGB... (3 bytes per LED).
    EZINFO("Received color byte stream for %ld LEDs", (long)led_count);
    EZHEXDUMP(payload, req->content_len);

    appEventBus_Notify(APP_EVENT_WS2812_COLOR_UPDATE, (void*)payload, req->content_len);

    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}


int appColorPicker_OnReceiveEventCallback(uint32_t event_code, const void *data, size_t data_size)
{
    switch (event_code)
    {
    case APP_EVENT_WIFI_CONNECTED:
        EZDEBUG("Received event: APP_EVENT_WIFI_CONNECTED");

        if((data == NULL) || (data_size != sizeof(httpd_handle_t)))
        {
            EZWARNING("Invalid HTTP server data");
            break;
        }

        if(is_webserver_started == false)
        {
            if(appColorPicker_StartWebserver(http_server_h) == false)
            {
                EZERROR("Failed to start color picker webserver");
            }
        }
        break;

    case APP_EVENT_WIFI_DISCONNECTED:
        EZDEBUG("Received event: APP_EVENT_WIFI_DISCONNECTED");
        http_server_h = NULL;
        is_webserver_started = false;
        break;

    default:
        break;
    }

    return 0;
}

