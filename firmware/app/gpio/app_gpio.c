
#include "app_gpio.h"

#include "ez_easy_embedded.h"
#define DEBUG_LVL   LVL_DEBUG   /**< logging level */
#define MOD_NAME    "app_gpio"       /**< module name */
#include "ez_logging.h"
#include "ez_gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_common.h"
#include "gpio_port.h"
#include "app_event_bus.h"

#define TASK_PRIORITY                       10
#define TASK_STACK_SIZE                     2048
#define GPIO_DEBOUNCE_PRESS_TIME            80
#define GPIO_DEBOUNCE_LONG_PRESS_TIME       2000
#define BUTTON_PRESSED                      0
#define BUTTON_RELEASED                     1

static ezGpioDrvInstance_t gpio_inst;
static ezSubject app_event;
static ezObserver event_bus_sub;
static volatile uint32_t button_state = BUTTON_RELEASED;
static uint32_t button_debounce = 0;
static uint32_t red_led_status = 0;
static uint32_t blue_led_status = 0;

static void appGpio_ButtoTask(void* arg);
static int appGpio_EventCallback(uint32_t event_code, void *param1, void *param2);
static int appGpio_EventBusCallback(uint32_t event_code, void *param1, void *param2);

bool appGpio_Init(void)
{
    ezHwGpioConfig_t gpio_config;
    button_state = BUTTON_RELEASED;

    if(gpioPort_Init() == false)
    {
        EZERROR("Failed to initialize hardware GPIO");
        return false;
    }

    if(ezGpio_RegisterInstance(&gpio_inst, GPIO_DRIVER_NAME, appGpio_EventCallback) != STATUS_OK)
    {
        EZERROR("Failed to register GPIO driver");
        return false;
    }

    gpio_config.mode = EZ_GPIO_MODE_OUTPUT;
    gpio_config.pull = EZ_GPIO_PULL_UP;

    if(ezGpio_Initialize(&gpio_inst, LED_RED, &gpio_config) != STATUS_OK 
        || ezGpio_Initialize(&gpio_inst, LED_BLUE, &gpio_config) != STATUS_OK
        || ezGpio_Initialize(&gpio_inst, LED_GREEN, &gpio_config) != STATUS_OK)
    {
        EZERROR("Failed to initialize GPIO driver");
        return false;
    }

    gpio_config.mode = EZ_GPIO_MODE_INPUT;
    gpio_config.pull = EZ_GPIO_PULL_UP;
    gpio_config.intr_mode = EZ_INTTERTUP_BOTH;

    if(ezGpio_Initialize(&gpio_inst, SERVICE_BUTTON, &gpio_config) != STATUS_OK)
    {
        EZERROR("Failed to initialize GPIO driver");
        return false;
    }

    if(ezGpio_WritePin(&gpio_inst, LED_RED, EZ_GPIO_PIN_LOW) != STATUS_OK
        || ezGpio_WritePin(&gpio_inst, LED_BLUE, EZ_GPIO_PIN_LOW) != STATUS_OK
        || ezGpio_WritePin(&gpio_inst, SERVICE_BUTTON, EZ_GPIO_PIN_LOW) != STATUS_OK)
    {
        EZERROR("Failed to write GPIO driver");
        return false;
    }

    if(ezEventNotifier_CreateSubject(&app_event) != ezSUCCESS)
    {
        EZERROR("Failed to create event subject");
        return false;
    }

    if(ezEventNotifier_CreateObserver(&event_bus_sub, appGpio_EventBusCallback) != ezSUCCESS)
    {
        EZERROR("Failed to create event observer");
        return false;
    }
    else
    {
        appEventBus_Subscribe(&event_bus_sub);
    }

    xTaskCreate(appGpio_ButtoTask,
        "button_task",
        TASK_STACK_SIZE, NULL,
        TASK_PRIORITY, NULL);
    return true;
}


ezSTATUS appGpio_SubscribeToEvent(ezObserver *subcriber)
{
    return ezEventNotifier_SubscribeToSubject(&app_event, subcriber);
}


static int appGpio_EventCallback(uint32_t event_code, void *param1, void *param2)
{
    button_state = *(uint32_t*)param1;
    EZDEBUG("got pin %ld, state %ld", event_code, button_state);
    return 0;
}


static void appGpio_ButtoTask(void* arg)
{
    while(1){
        if(button_state == BUTTON_PRESSED)
        {
            button_debounce += 10;
        }
        else
        {
            GPIO_EVENT_SOURCE event_sourve = GPIO_EVENT_SOURCE_BUTTON;
            if(button_debounce > GPIO_DEBOUNCE_LONG_PRESS_TIME)
            {
                EZDEBUG("Long press detected");
                blue_led_status = !blue_led_status;
                ezGpio_WritePin(&gpio_inst, LED_BLUE, blue_led_status);
                ezEventNotifier_NotifyEvent(&app_event, APP_EVENT_GPIO_LONG_PRESS, &event_sourve, NULL);
            }
            else if(button_debounce > GPIO_DEBOUNCE_PRESS_TIME)
            {
                EZDEBUG("Short press detected");
                red_led_status = !red_led_status;
                ezGpio_WritePin(&gpio_inst, LED_RED, red_led_status);
                ezEventNotifier_NotifyEvent(&app_event, APP_EVENT_GPIO_PRESS, &event_sourve, NULL);
            }
            else
            {
                /* do nothing */
            }
            button_debounce = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


static int appGpio_EventBusCallback(uint32_t event_code, void *param1, void *param2)
{
    switch (event_code)
    {
        case APP_EVENT_LED_RED_ON:
            EZDEBUG("Turning on red LED");
            ezGpio_WritePin(&gpio_inst, LED_RED, EZ_GPIO_PIN_HIGH);
            break;
        case APP_EVENT_LED_RED_OFF:
            EZDEBUG("Turning off red LED");
            ezGpio_WritePin(&gpio_inst, LED_RED, EZ_GPIO_PIN_LOW);
            break;
        case APP_EVENT_LED_BLUE_ON:
            EZDEBUG("Turning on blue LED");
            ezGpio_WritePin(&gpio_inst, LED_BLUE, EZ_GPIO_PIN_HIGH);
            break;
        case APP_EVENT_LED_BLUE_OFF:
            EZDEBUG("Turning off blue LED");
            ezGpio_WritePin(&gpio_inst, LED_BLUE, EZ_GPIO_PIN_LOW);
            break;
        default:
            EZDEBUG("Event bus: Unknown event %ld", event_code);
            break;
    }
}