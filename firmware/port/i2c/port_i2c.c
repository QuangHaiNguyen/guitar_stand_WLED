#include "port_i2c.h"

#include "ez_easy_embedded.h"
#define DEBUG_LVL   LVL_ERROR   /**< logging level */
#define MOD_NAME    "hw i2c"      /**< module name */
#include "ez_logging.h"
#include "ez_i2c.h"

#include "esp_attr.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_common.h"

#define ESP_INTR_FLAG_DEFAULT   0
#define NUM_QUEUE_ITEMS         10
#define TASK_PRIORITY           10
#define EVENT_HANDLE_TASK_SIZE  2048
#define SDA_GPIO_NUM         1
#define SCL_GPIO_NUM         2

static struct ezI2cDriver driver;
static i2c_config_t esp_i2c_conf;


//static void gpio_task_event_handling(void* arg);
//static void IRAM_ATTR gpio_isr_handler(void* arg);


static EZ_DRV_STATUS espI2c_Initialize(ezI2cConfig_t *config);
static EZ_DRV_STATUS espI2c_TransmitSync(uint16_t address,
                                          const uint8_t *data,
                                          size_t length,
                                          uint32_t timeout_millis);

static EZ_DRV_STATUS espI2c_TransmitAsync(uint16_t address,
                                           const uint8_t *data,
                                           size_t length);

static EZ_DRV_STATUS espI2c_ReceiveSync(uint16_t address,
                                         uint8_t *data,
                                         size_t length,
                                         uint32_t timeout_millis);

static EZ_DRV_STATUS espI2c_ReceiveAsync(uint16_t address,
                                          uint8_t *data,
                                          size_t length);

static EZ_DRV_STATUS espI2c_Probe(uint16_t address,
                                  uint32_t timeout_millis);

bool portI2c_Init(void)
{
    driver.interface.initialize = espI2c_Initialize;
    driver.interface.transmit_sync = espI2c_TransmitSync;
    //driver.interface.transmit_async = espI2c_TransmitAsync;
    driver.interface.receive_sync = espI2c_ReceiveSync;
    //driver.interface.receive_async = espI2c_ReceiveAsync;
    driver.interface.probe = espI2c_Probe;
    driver.initialized = false;
    driver.common.name = I2C_DRIVER_NAME;
    driver.common.version[0] = 1;
    driver.common.version[1] = 0;
    driver.common.version[2] = 0;
    if (ezI2c_SystemRegisterHwDriver(&driver) != STATUS_OK) {
        EZERROR("Failed to register I2C driver");
        return false;
    }

    return true;
}


static EZ_DRV_STATUS espI2c_Initialize(ezI2cConfig_t *config)
{
    esp_err_t err = ESP_OK;
    EZTRACE("espI2c_Initialize()");
    if(config == NULL)
    {
        EZERROR("Invalid configuration");
        return STATUS_ERR_ARG;
    }

    if(config->mode == EZ_I2C_MODE_MASTER)
    {
        esp_i2c_conf.mode = I2C_MODE_MASTER;
        esp_i2c_conf.master.clk_speed = config->speed;
    }
    
    esp_i2c_conf.sda_io_num = SDA_GPIO_NUM;
    esp_i2c_conf.scl_io_num = SCL_GPIO_NUM;
    esp_i2c_conf.sda_pullup_en = true;
    esp_i2c_conf.scl_pullup_en = true;

    err = i2c_param_config(I2C_NUM_0, &esp_i2c_conf);
    if(err != ESP_OK)
    {
        EZERROR("Failed to configure I2C parameters: %s", esp_err_to_name(err));
        return STATUS_ERR_ARG;
    }

    err = i2c_driver_install(I2C_NUM_0, esp_i2c_conf.mode, 0, 0, 0);
    if(err != ESP_OK)
    {
        EZERROR("Failed to install I2C driver: %s", esp_err_to_name(err));
        return STATUS_ERR_ARG;
    }

    return STATUS_OK;
}

static EZ_DRV_STATUS espI2c_TransmitSync(uint16_t address,
                                          const uint8_t *data,
                                          size_t length,
                                          uint32_t timeout_millis)
{
    EZTRACE("espI2c_TransmitSync(address = %d, length = %d)", address, length);
    if (data == NULL || length == 0) {
        EZERROR("Invalid data or length");
        return STATUS_ERR_ARG;
    }
    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, address, data, length, timeout_millis / portTICK_PERIOD_MS);

    if(err == ESP_OK)
    {
        return STATUS_OK;
    }
    else if (err == ESP_ERR_TIMEOUT)
    {
        EZERROR("I2C transmit timeout");
        return STATUS_TIMEOUT;
    }
    else
    {
        EZERROR("I2C transmit error: %s", esp_err_to_name(err));
        return STATUS_ERR_GENERIC;
    }
    return STATUS_ERR_GENERIC;
}

static EZ_DRV_STATUS espI2c_TransmitAsync(uint16_t address,
                                           const uint8_t *data,
                                           size_t length)
{
    return STATUS_ERR_GENERIC;
}

static EZ_DRV_STATUS espI2c_ReceiveSync(uint16_t address,
                                         uint8_t *data,
                                         size_t length,
                                         uint32_t timeout_millis)
{
    EZTRACE("espI2c_ReceiveSync(address = %d, length = %d)", address, length);
    if (data == NULL || length == 0) {
        EZERROR("Invalid data or length");
        return STATUS_ERR_ARG;
    }
    esp_err_t err = i2c_master_read_from_device(I2C_NUM_0, address, data, length, timeout_millis / portTICK_PERIOD_MS);
    if(err == ESP_OK)
    {
        return STATUS_OK;
    }
    else if (err == ESP_ERR_TIMEOUT)
    {
        EZERROR("I2C receive timeout");
        return STATUS_TIMEOUT;
    }
    else
    {
        EZERROR("I2C receive error: %s", esp_err_to_name(err));
        return STATUS_ERR_GENERIC;
    }
    return STATUS_ERR_GENERIC;
}

static EZ_DRV_STATUS espI2c_ReceiveAsync(uint16_t address,
                                          uint8_t *data,
                                          size_t length)
{
    return STATUS_ERR_GENERIC;
}

static EZ_DRV_STATUS espI2c_Probe(uint16_t address,
                                  uint32_t timeout_millis)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, timeout_millis / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if(ret == ESP_OK)
    {
        return STATUS_OK;
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        EZERROR("I2C probe timeout");
        return STATUS_TIMEOUT;
    }
    else
    {
        EZERROR("I2C probe error: %s", esp_err_to_name(ret));
        return STATUS_ERR_GENERIC;
    }

    STATUS_ERR_GENERIC;
}