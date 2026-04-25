
#include "app_gpio.h"

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
static ezEventListener_t event_listener;
static volatile uint32_t button_state = BUTTON_RELEASED;
static uint32_t button_debounce = 0;
static uint32_t red_led_status = 0;
static uint32_t blue_led_status = 0;

static void appGpio_ButtonTask(void* arg);
static void appGpio_EventCallback(uint8_t event_code, void *param1, void *param2);
static int appGpio_EventBusCallback(uint32_t event_code, const void *data, size_t data_size);

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

    if(ezEventBus_CreateListener(&event_listener, appGpio_EventBusCallback) != ezSUCCESS)
    {
        EZERROR("Failed to create event observer");
        return false;
    }
    else
    {
        appEventBus_Subscribe(&event_listener);
    }

    xTaskCreate(appGpio_ButtonTask,
        "button_task",
        TASK_STACK_SIZE, NULL,
        TASK_PRIORITY, NULL);
    return true;
}


ezSTATUS appGpio_SubscribeToEvent(ezEventListener_t *listener)
{
    //return ezEventNotifier_SubscribeToSubject(&app_event, subcriber);
    return ezFAIL;
}


static void appGpio_EventCallback(uint8_t event_code, void *param1, void *param2)
{
    button_state = *(uint32_t*)param1;
    EZDEBUG("got pin %ld, state %ld", event_code, button_state);
}


static void appGpio_ButtonTask(void* arg)
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
                appEventBus_Notify(APP_EVENT_GPIO_LONG_PRESS, &event_sourve, sizeof(event_sourve));
            }
            else if(button_debounce > GPIO_DEBOUNCE_PRESS_TIME)
            {
                EZDEBUG("Short press detected");
                red_led_status = !red_led_status;
                ezGpio_WritePin(&gpio_inst, LED_RED, red_led_status);
                appEventBus_Notify(APP_EVENT_GPIO_PRESS, &event_sourve, sizeof(event_sourve));
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


static int appGpio_EventBusCallback(uint32_t event_code, const void *data, size_t data_size)
{
    EZDEBUG("Received event code %d, data size %d", event_code, data_size);
    bool led_on = *(bool*)data;
    switch (event_code)
    {
        case APP_EVENT_LED_RED:
            EZDEBUG("setting red LED to %s", led_on ? "ON" : "OFF");
            if(ezGpio_WritePin(&gpio_inst, LED_RED, led_on ? EZ_GPIO_PIN_HIGH : EZ_GPIO_PIN_LOW) != STATUS_OK)
            {
                EZERROR("Failed to write GPIO pin");
            }
            break;
        case APP_EVENT_LED_BLUE:
            EZDEBUG("setting blue LED to %s", led_on ? "ON" : "OFF");
            if(ezGpio_WritePin(&gpio_inst, LED_BLUE, led_on ? EZ_GPIO_PIN_HIGH : EZ_GPIO_PIN_LOW) != STATUS_OK)
            {
                EZERROR("Failed to write GPIO pin");
            }
            break;
        default:
            EZDEBUG("Event bus: Unknown event %ld", event_code);
            break;
    }
}