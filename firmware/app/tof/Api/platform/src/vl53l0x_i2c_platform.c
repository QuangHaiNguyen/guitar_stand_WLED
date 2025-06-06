/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/*!
 * \file   VL53L0X_platform.c
 * \brief  Code function defintions for Ewok Platform Layer
 *
 */


//#include <windows.h>
#include <stdio.h>    // sprintf(), vsnprintf(), printf()

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

//#include "ranging_sensor_comms.h"
//#include "comms_platform.h"

#include "vl53l0x_platform_log.h"

#include "ez_easy_embedded.h"
#define DEBUG_LVL   LVL_DEBUG           /**< logging level */
#define MOD_NAME    "platform_i2c"      /**< module name */
#include "ez_logging.h"
#include "ez_i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_common.h"
#include "port_i2c.h"


#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_PLATFORM, level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) trace_print_module_function(TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif

#define BUFF_SIZE 512

static uint8_t buff[BUFF_SIZE];

char  debug_string[VL53L0X_MAX_STRING_LENGTH_PLT];

#define TOF_SENSOR_I2C_ADDRESS      0x29
#define MIN_COMMS_VERSION_MAJOR     1
#define MIN_COMMS_VERSION_MINOR     8
#define MIN_COMMS_VERSION_BUILD     1
#define MIN_COMMS_VERSION_REVISION  0

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

static int platform_i2c_callback(uint32_t event_code, void *param1, void *param2);
static ezI2cDrvInstance_t instance;
static ezI2cConfig_t i2c_config = {
    .mode = EZ_I2C_MODE_MASTER,
    .speed = 100000, // 100 kHz
    .addressing_mode = EZ_I2C_ADDRESSING_MODE_7BIT
};


bool_t _check_min_version(void)
{
    #if 0
    bool_t min_version_comms_dll = FALSE;
    int32_t status   = STATUS_OK;
    COMMS_VERSION_INFO comms_version_info;


    status = RANGING_SENSOR_COMMS_Get_Version( &comms_version_info );
    if(status != STATUS_OK)
    {
        RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
    }
    else
    {
        // combined check of major, minor and build
        if (
            (comms_version_info.major > MIN_COMMS_VERSION_MAJOR) ||
            ((comms_version_info.major == MIN_COMMS_VERSION_MAJOR) && (comms_version_info.minor > MIN_COMMS_VERSION_MINOR)) ||
            ((comms_version_info.major == MIN_COMMS_VERSION_MAJOR) && (comms_version_info.minor == MIN_COMMS_VERSION_MINOR) && (comms_version_info.build >= MIN_COMMS_VERSION_BUILD))
            )
        {
            min_version_comms_dll = TRUE;
        }

        // subsequent check of svn revision
        if ( comms_version_info.revision < MIN_COMMS_VERSION_REVISION )
        {
            min_version_comms_dll = FALSE;
        }
    }

    return min_version_comms_dll;
    #else
    return 0; // always return true for now
    #endif
}

int32_t VL53L0X_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz)
{
    static const int32_t cmax_devices = 4;

    int32_t status   = STATUS_OK;

    if (comms_type != I2C)
    {
        EZERROR("Unsupported comms type: %d", comms_type);
        return STATUS_FAIL;
    }

    if(portI2c_Init() == false)
    {
        EZERROR("Failed to initialize hardware I2C");
        return false;
    }

    if(ezI2c_RegisterInstance(&instance, I2C_DRIVER_NAME, platform_i2c_callback) != STATUS_OK)
    {
        EZERROR("Failed to register I2C driver");
        return false;
    }

    i2c_config.speed = comms_speed_khz * 1000; // Convert kHz to Hz
    if(ezI2c_Initialize(&instance, &i2c_config) != STATUS_OK)
    {
        EZERROR("Failed to initialize I2C driver");
        return false;
    }

    if(ezI2c_Probe(&instance, TOF_SENSOR_I2C_ADDRESS, 1000) != STATUS_OK)
    {
        EZERROR("Failed to probe TOF sensor at address 0x%02X", TOF_SENSOR_I2C_ADDRESS);
        return false;
    }

    EZINFO("TOF sensor initialized successfully at address 0x%02X", TOF_SENSOR_I2C_ADDRESS);

    return status;
}

int32_t VL53L0X_comms_close(void)
{
    int32_t status = STATUS_OK;
#if 0
    RANGING_SENSOR_COMMS_Fini_V2W8();

    if(status != STATUS_OK)
    {
        RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
    }
#endif
    return status;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    if(count + 1 > BUFF_SIZE)
    {
        EZERROR("Data size exceeds buffer size");
        return STATUS_FAIL;
    }

    // Prepare the buffer for I2C write
    buff[0] = index;
    memcpy(&buff[1], pdata, count);

    // Write the data to the device
    if(ezI2c_TransmitSync(&instance, address, buff, count + 1, 1000) == STATUS_OK)
    {
        status = STATUS_OK;
    }
    else
    {
        EZERROR("Failed to write at index 0x%02X", index);
        status = STATUS_FAIL;
    }

#ifdef VL53L0X_LOG_ENABLE
    int32_t i = 0;
    char value_as_str[VL53L0X_MAX_STRING_LENGTH_PLT];
    char *pvalue_as_str;

    pvalue_as_str =  value_as_str;

    for(i = 0 ; i < count ; i++)
    {
        sprintf(pvalue_as_str,"%02X", *(pdata+i));

        pvalue_as_str += 2;
    }
    trace_i2c("Write reg : 0x%04X, Val : 0x%s\n", index, value_as_str);
#endif
    return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{

#ifdef VL53L0X_LOG_ENABLE
    int32_t      i = 0;
    char   value_as_str[VL53L0X_MAX_STRING_LENGTH_PLT];
    char *pvalue_as_str;
#endif
    if(ezI2c_TransmitSync(&instance, address, &index, 1, 1000) != STATUS_OK)
    {
        return STATUS_FAIL;
    }

    if(ezI2c_ReceiveSync(&instance, address, pdata, count, 1000) != STATUS_OK)
    {
        EZERROR("Failed to read from index 0x%02X", index);
        return STATUS_FAIL;
    }

#if 0
    // read 8-bit standard V2W8 write (not paging mode)
    status = RANGING_SENSOR_COMMS_Read_V2W8(address, 0, index, pdata, count);

    if(status != STATUS_OK)
    {
        RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
    }
#endif
#ifdef VL53L0X_LOG_ENABLE
    // Build  value as string;
    pvalue_as_str =  value_as_str;

    for(i = 0 ; i < count ; i++)
    {
        sprintf(pvalue_as_str, "%02X", *(pdata+i));
        pvalue_as_str += 2;
    }

    trace_i2c("Read  reg : 0x%04X, Val : 0x%s\n", index, value_as_str);
#endif

    return STATUS_OK;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
    const int32_t cbyte_count = 1;

    status = VL53L0X_write_multi(address, index, &data, cbyte_count);

    return status;

}


int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;

    uint8_t  buffer[BYTES_PER_WORD];

    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);

    return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_DWORD];

    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;

}


int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

    return status;

}


int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
	*pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;

}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;

}

static int platform_i2c_callback(uint32_t event_code, void *param1, void *param2)
{
    return 0;
}

#if 0
int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    int32_t status = STATUS_OK;
    float wait_ms = (float)wait_us/1000.0f;

    /*
     * Use windows event handling to perform non-blocking wait.
     */
    HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
    WaitForSingleObject(hEvent, (int)(wait_ms + 0.5f));

#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("Wait us : %6d\n", wait_us);
#endif

    return status;

}


int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    int32_t status = STATUS_OK;

    /*
     * Use windows event handling to perform non-blocking wait.
     */
    HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
    WaitForSingleObject(hEvent, wait_ms);

#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("Wait ms : %6d\n", wait_ms);
#endif

    return status;

}


int32_t VL53L0X_set_gpio(uint8_t level)
{
    int32_t status = STATUS_OK;
    //status = VL53L0X_set_gpio_sv(level);
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// Set GPIO = %d;\n", level);
#endif

    return status;

}


int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    int32_t status = STATUS_OK;
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// Get GPIO = %d;\n", *plevel);
#endif
    return status;
}


int32_t VL53L0X_release_gpio(void)
{
    int32_t status = STATUS_OK;
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// Releasing force on GPIO\n");
#endif
    return status;

}

int32_t VL53L0X_cycle_power(void)
{
    int32_t status = STATUS_OK;
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// cycle sensor power\n");
#endif
    status = RANGING_SENSOR_COMMS_Cycle_Sensor_Power();

	if(status != STATUS_OK)
    {
        RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
    }

	return status;
}


int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
       *ptimer_freq_hz = 0;
       return STATUS_FAIL;
}


int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
       *ptimer_count = 0;
       return STATUS_FAIL;
}
#endif