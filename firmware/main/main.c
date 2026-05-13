
#define DEBUG_LVL   LVL_TRACE   /**< logging level */
#define MOD_NAME    "main"       /**< module name */
#include "ez_logging.h"

#include "app_gpio.h"
#include "app_wifi_manager.h"
#include "app_tof.h"
#include "app_common.h"
#include "app_event_bus.h"
#include "app_color_picker.h"
#include "ws2812.h"


void app_main(void)
{
    if(appEventBus_Init() == false)
    {
        EZERROR("Failed to initialize event bus");
    }

    if (appColorPicker_Init() == false)
    {
        EZERROR("Failed to initialize color picker");
    }
    
    if(appGpio_Init() == false)
    {
        EZERROR("Failed to initialize GPIO");
    }

    if(ws2812_Init() == false)
    {
        EZERROR("Failed to initialize WS2812 driver");
    }

    if(wifi_manager_Init() == false)
    {
        EZERROR("Failed to initialize WiFi manager");
    }
    
    if(appTof_Init() == false)
    {
        EZERROR("Failed to initialize TOF sensor");
    }
}

