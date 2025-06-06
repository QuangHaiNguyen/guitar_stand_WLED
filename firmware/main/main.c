
#include "ez_easy_embedded.h"
#define DEBUG_LVL   LVL_TRACE   /**< logging level */
#define MOD_NAME    "main"       /**< module name */
#include "ez_logging.h"

#include "app_gpio.h"
#include "app_wifi_manager.h"
#include "app_tof.h"
#include "app_common.h"
#include "app_event_bus.h"


void app_main(void)
{
    ezEasyEmbedded_Initialize();
    if(appEventBus_Init() == false)
    {
        EZERROR("Failed to initialize event bus");
    }

    appGpio_Init();
    //wifi_manager_Init();

    if(appTof_Init() == false)
    {
        EZERROR("Failed to initialize TOF sensor");
    }
}

