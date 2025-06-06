
#include "app_event_bus.h"

#include "ez_easy_embedded.h"
#define DEBUG_LVL   LVL_DEBUG   /**< logging level */
#define MOD_NAME    "event bus"       /**< module name */
#include "ez_logging.h"
#include "app_common.h"


static ezSubject app_event;

bool appEventBus_Init(void)
{

    if(ezEventNotifier_CreateSubject(&app_event) != ezSUCCESS)
    {
        EZERROR("Failed to create event subject");
        return false;
    }

    return true;
}


ezSTATUS appEventBus_Subscribe(ezObserver *subcriber)
{
    return ezEventNotifier_SubscribeToSubject(&app_event, subcriber);
}


void appEventBus_Notify(uint32_t event_code, void *param1, void *param2)
{
    ezEventNotifier_NotifyEvent(&app_event, event_code, param1, param2);
}

