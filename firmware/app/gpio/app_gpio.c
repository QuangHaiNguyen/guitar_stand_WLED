
/**
 * @file app_gpio.c
 * @brief Implementation of GPIO setup, button debouncing, and LED event handling.
 */

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

#define TASK_PRIORITY                       10    /**< FreeRTOS priority for button task. */
#define TASK_STACK_SIZE                     2048  /**< Stack size (bytes) for button task. */
#define GPIO_DEBOUNCE_PRESS_TIME            80    /**< Minimum press duration (ms) for short press. */
#define GPIO_DEBOUNCE_LONG_PRESS_TIME       2000  /**< Minimum press duration (ms) for long press. */
#define BUTTON_PRESSED                      0     /**< Logical level for pressed button. */
#define BUTTON_RELEASED                     1     /**< Logical level for released button. */
#define STATE_ON                            1     /**< Logical level for LED on. */
#define STATE_OFF                           0     /**< Logical level for LED off. */

typedef struct{
    uint8_t state;
    uint32_t duration_ms;
}LedStep_t;

typedef struct{
    uint8_t step_count;
    LedStep_t *steps;
}LedPattern_t;

typedef struct
{
    uint32_t pin_index;
    LedPattern_t *pattern;
    uint8_t curr_step;
    uint32_t next_tick_ms;
}LedContext_t;



static ezGpioDrvInstance_t gpio_inst;
static ezEventListener_t event_listener;
static volatile uint32_t button_state = BUTTON_RELEASED;
static uint32_t button_debounce = 0;
static uint32_t red_led_status = 0;
static uint32_t blue_led_status = 0;
static uint32_t green_led_status = 0;

static LedStep_t HeartBeatPattern[] = {
    {STATE_ON, 100},
    {STATE_OFF, 100},
    {STATE_ON, 100},
    {STATE_OFF, 700},
};

static LedPattern_t HeartBeatLedPattern = {
    .step_count = sizeof(HeartBeatPattern) / sizeof(LedStep_t),
    .steps = HeartBeatPattern,
};

static LedStep_t BlinkShortPattern[] = {
    {STATE_ON, 100},
    {STATE_OFF, 900}, 
};

static LedPattern_t BlinkShortLedPattern = {
    .step_count = sizeof(BlinkShortPattern) / sizeof(LedStep_t),
    .steps = BlinkShortPattern, 
};

static LedStep_t BlinkLongPattern[] = {
    {STATE_ON, 800},
    {STATE_OFF, 200}, 
};

static LedPattern_t BlinkLongLedPattern = {
    .step_count = sizeof(BlinkLongPattern) / sizeof(LedStep_t),
    .steps = BlinkLongPattern, 
};

static LedStep_t NoBlinkPattern[] = {
    {STATE_OFF, 1000},
};

static LedPattern_t NoBlinkLedPattern = {
    .step_count = 1,
    .steps = NoBlinkPattern,
};

static LedContext_t wifi_led_ctx = {
    .pin_index = LED_WIFI,
    .pattern = &NoBlinkLedPattern,
    .curr_step = 0,
    .next_tick_ms = 0,
};


static LedContext_t calib_led_ctx = {
    .pin_index = LED_GREEN,
    .pattern = &NoBlinkLedPattern,
    .curr_step = 0,
    .next_tick_ms = 0,
};


static void appGpio_ButtonTask(void* arg);
static void appGpio_EventCallback(uint8_t event_code, void *param1, void *param2);
static void appGpio_EventCallbackDirect(uint16_t pin_index, uint32_t pin_state);
static int appGpio_EventBusCallback(uint32_t event_code, const void *data, size_t data_size);
static void appGpio_UpdateContext(LedContext_t *ctx, uint32_t curr_tick_ms);

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
        || ezGpio_Initialize(&gpio_inst, LED_GREEN, &gpio_config) != STATUS_OK
        || ezGpio_Initialize(&gpio_inst, LED_WIFI, &gpio_config) != STATUS_OK)
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
        || ezGpio_WritePin(&gpio_inst, LED_GREEN, EZ_GPIO_PIN_LOW) != STATUS_OK
        || ezGpio_WritePin(&gpio_inst, LED_WIFI, EZ_GPIO_PIN_LOW) != STATUS_OK)
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

    gpioPort_RegisterEventCallback(appGpio_EventCallbackDirect);

    xTaskCreate(appGpio_ButtonTask,
        "button_task",
        TASK_STACK_SIZE, NULL,
        TASK_PRIORITY, NULL);
    return true;
}

/**
 * @brief Generic GPIO driver callback.
 *
 * Updates the module button state using the pin state passed via @p param1.
 *
 * @param[in] event_code GPIO event code (currently unused).
 * @param[in] param1 Pointer to a @c uint32_t pin state value.
 * @param[in] param2 Additional callback data (unused).
 */
static void appGpio_EventCallback(uint8_t event_code, void *param1, void *param2)
{
    button_state = *(uint32_t*)param1;
}

/**
 * @brief Direct callback from GPIO port abstraction.
 *
 * @param[in] pin_index GPIO index that changed.
 * @param[in] pin_state New logical pin state.
 */
static void appGpio_EventCallbackDirect(uint16_t pin_index, uint32_t pin_state)
{
    EZDEBUG("GPIO event callback: pin index %d, pin state %d", pin_index, pin_state);
    button_state = pin_state;
}


/**
 * @brief FreeRTOS task that debounces button input and emits press events.
 *
 * Polls button state every 10 ms. On release, classifies the press duration as
 * short or long, toggles corresponding LED state, and publishes
 * @c APP_EVENT_GPIO_PRESS or @c APP_EVENT_GPIO_LONG_PRESS.
 *
 * @param[in] arg Unused task argument.
 */
static void appGpio_ButtonTask(void* arg)
{
    uint32_t curr_tick_ms = 0;
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


        appGpio_UpdateContext(&wifi_led_ctx, curr_tick_ms);
        appGpio_UpdateContext(&calib_led_ctx, curr_tick_ms);
        curr_tick_ms += 10;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


/**
 * @brief Application event-bus listener for LED control events.
 *
 * Handles @c APP_EVENT_LED_RED and @c APP_EVENT_LED_BLUE by driving the
 * respective GPIO output pin according to the boolean payload.
 *
 * @param[in] event_code Application event identifier.
 * @param[in] data Pointer to a @c bool payload indicating LED on/off state.
 * @param[in] data_size Payload size in bytes.
 * @return Returns 0.
 */
static int appGpio_EventBusCallback(uint32_t event_code, const void *data, size_t data_size)
{
    EZTRACE("Received event code %d, data size %d", event_code, data_size);
    uint8_t *calib_status = NULL;
    switch (event_code)
    {
        case APP_EVENT_LED_RED:
        {
            bool led_on = *(bool*)data;
            EZTRACE("setting red LED to %s", led_on ? "ON" : "OFF");
            if(ezGpio_WritePin(&gpio_inst, LED_RED, led_on ? EZ_GPIO_PIN_HIGH : EZ_GPIO_PIN_LOW) != STATUS_OK)
            {
                EZERROR("Failed to write GPIO pin");
            }
            break;
        }
        case APP_EVENT_LED_BLUE:
        {
            bool led_on = *(bool*)data;
            EZTRACE("setting blue LED to %s", led_on ? "ON" : "OFF");
            if(ezGpio_WritePin(&gpio_inst, LED_BLUE, led_on ? EZ_GPIO_PIN_HIGH : EZ_GPIO_PIN_LOW) != STATUS_OK)
            {
                EZERROR("Failed to write GPIO pin");
            }
            break;
        }
        case APP_EVENT_WIFI_CONNECTED:
            wifi_led_ctx.pattern = &BlinkShortLedPattern;
            wifi_led_ctx.next_tick_ms = 0;
            wifi_led_ctx.curr_step = 0;
            break;
        case APP_EVENT_WIFI_START_AP_MODE:
            wifi_led_ctx.pattern = &HeartBeatLedPattern;
            wifi_led_ctx.next_tick_ms = 0;
            wifi_led_ctx.curr_step = 0;
            break;
        case APP_EVENT_TOF_START_CALIB:
            calib_led_ctx.pattern = &BlinkShortLedPattern;
            calib_led_ctx.next_tick_ms = 0;
            calib_led_ctx.curr_step = 0;
            break;
        case APP_EVENT_TOF_CALIB_STATUS:
            calib_status = (uint8_t*)data;
            if(calib_status != NULL && (*calib_status == 0 || *calib_status == -1))
            {
                calib_led_ctx.pattern = &NoBlinkLedPattern;
                calib_led_ctx.next_tick_ms = 0;
                calib_led_ctx.curr_step = 0;
            }
            break;
        default:
            EZTRACE("Event bus: Unknown event %ld", event_code);
            break;
    }
}

static void appGpio_UpdateContext(LedContext_t *ctx, uint32_t curr_tick_ms)
{
    if(ctx == NULL)
    {
        return;
    }

    if(ctx->next_tick_ms > curr_tick_ms)
    {
        return;
    }

    ctx->next_tick_ms = curr_tick_ms + ctx->pattern->steps[ctx->curr_step].duration_ms;
    ezGpio_WritePin(&gpio_inst, ctx->pin_index, ctx->pattern->steps[ctx->curr_step].state);
    ctx->curr_step++;
    if(ctx->curr_step >= ctx->pattern->step_count)
    {
        ctx->curr_step = 0; 
    }
}
