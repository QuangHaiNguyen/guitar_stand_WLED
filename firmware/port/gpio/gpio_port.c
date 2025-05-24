#include "gpio_port.h"

#include "ez_easy_embedded.h"
#define DEBUG_LVL   LVL_ERROR   /**< logging level */
#define MOD_NAME    "gpio"      /**< module name */
#include "ez_logging.h"
#include "ez_gpio.h"

#include "esp_attr.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_common.h"

#define ESP_INTR_FLAG_DEFAULT   0
#define NUM_QUEUE_ITEMS         10
#define TASK_PRIORITY           10
#define EVENT_HANDLE_TASK_SIZE  2048

static struct ezGpioDriver driver;
static gpio_config_t io_conf;
static QueueHandle_t gpio_evt_queue = NULL;

static EZ_GPIO_PIN_STATE esp32Gpio_ReadPin(uint16_t pin_index);
static EZ_DRV_STATUS esp32Gpio_WritePin(uint16_t pin_index, EZ_GPIO_PIN_STATE state);
static EZ_DRV_STATUS esp32Gpio_Initialize(uint16_t pin_index, ezHwGpioConfig_t *config);
static void gpio_task_event_handling(void* arg);
static void IRAM_ATTR gpio_isr_handler(void* arg);

bool gpioPort_Init(void)
{
    EZTRACE("gpioPort_Init()");
    if(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT) != ESP_OK)
    {
        EZERROR("Failed to install ISR service");
        return false;
    }

    EZ_DRV_STATUS status = STATUS_OK;
    driver.interface.read_pin = esp32Gpio_ReadPin;
    driver.interface.write_pin = esp32Gpio_WritePin;
    driver.interface.toggle_pin = NULL;
    driver.interface.init_pin = esp32Gpio_Initialize;
    driver.initialized = false;

    driver.common.name = GPIO_DRIVER_NAME;
    driver.common.version[0] = 1;
    driver.common.version[1] = 0;
    driver.common.version[2] = 0;

    if(ezGpio_SystemRegisterHwDriver(&driver) != STATUS_OK)
    {
        EZERROR("Register driver with error code = %d", status);
        return false;
    }

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_event_handling,
        "event_handling",
        EVENT_HANDLE_TASK_SIZE, NULL,
        TASK_PRIORITY, NULL);

    return true;
}

static EZ_DRV_STATUS esp32Gpio_Initialize(uint16_t pin_index, ezHwGpioConfig_t *config)
{
    EZTRACE("esp32Gpio_Initialize(index = %d)", pin_index);
    if(pin_index >= GPIO_NUM_MAX)
    {
        EZERROR("Invalid pin index %d", pin_index);
        return STATUS_ERR_ARG;
    }

    io_conf.pin_bit_mask = (1ULL << pin_index);
    if(config->mode == EZ_GPIO_MODE_OUTPUT)
    {
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    else if(config->mode == EZ_GPIO_MODE_INPUT)
    {
        io_conf.mode = GPIO_MODE_INPUT;
        switch(config->intr_mode)
        {
        case EZ_INTTERTUP_RISING:
            io_conf.intr_type = GPIO_INTR_POSEDGE;
            gpio_isr_handler_add(pin_index, gpio_isr_handler, (void*) (uintptr_t)pin_index);
            break;
        case EZ_INTTERTUP_FALLING:
            io_conf.intr_type = GPIO_INTR_NEGEDGE;
            gpio_isr_handler_add(pin_index, gpio_isr_handler, (void*) (uintptr_t)pin_index);
            break;
        case EZ_INTTERTUP_BOTH:
            io_conf.intr_type = GPIO_INTR_ANYEDGE;
            gpio_isr_handler_add(pin_index, gpio_isr_handler, (void*) (uintptr_t)pin_index);
            break;
        case EZ_INTTERTUP_NONE:
            io_conf.intr_type = GPIO_INTR_DISABLE;
            gpio_isr_handler_remove(pin_index);
            break;
        default:
            gpio_isr_handler_remove(pin_index);
            break;
        }
    }
    else
    {
        EZERROR("Invalid mode %d", config->mode);
        return STATUS_ERR_ARG;
    }

    if(config->pull == EZ_GPIO_PULL_UP)
    {
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    }
    else if(config->pull == EZ_GPIO_PULL_DOWN)
    {
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    }
    else
    {
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    }

    gpio_config(&io_conf);
    return STATUS_OK;
}

static EZ_GPIO_PIN_STATE esp32Gpio_ReadPin(uint16_t pin_index)
{
    EZTRACE("esp32Gpio_ReadPin(index = %d)", pin_index);
    if(pin_index < GPIO_NUM_MAX)
    {
        if (gpio_get_level(pin_index) == 1)
        {
            return EZ_GPIO_PIN_HIGH;
        }
        else
        {
            return EZ_GPIO_PIN_LOW;
        }
    }
    return EZ_GPIO_PIN_ERROR;
}


static EZ_DRV_STATUS esp32Gpio_WritePin(uint16_t pin_index, EZ_GPIO_PIN_STATE state)
{
    EZTRACE("esp32Gpio_WritePin(index = %d, state = %d)", pin_index, state);
    if(pin_index >= GPIO_NUM_MAX)
    {
        return STATUS_ERR_ARG;
    }

    if(state == EZ_GPIO_PIN_HIGH)
    {
        gpio_set_level(pin_index, 1);
        return STATUS_OK;
    }
    else if(state == EZ_GPIO_PIN_LOW)
    {
        gpio_set_level(pin_index, 0);
        return STATUS_OK;
    }
    else
    {
        EZERROR("Invalid state %d", state);
        return STATUS_ERR_ARG;
    }
    return STATUS_ERR_GENERIC;
}

static void gpio_task_event_handling(void* arg)
{
    uint32_t io_num = 0;
    int pin_state = 0;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, 10)) {
            pin_state = gpio_get_level(io_num);
            ezEventNotifier_NotifyEvent(&driver.gpio_event, io_num, &pin_state, NULL);
        }
    }
}

static void gpio_isr_handler(void* arg)
{
    int32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
