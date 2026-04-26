/**
 * @file app_tof.c
 * @brief Implementation of the VL53L0X ToF sensor application module.
 *
 * @details
 * Internally the module uses three phases:
 *  - **Initialisation** (`appTof_Init`): comms setup, device info validation,
 *    calibration, ranging configuration.
 *  - **Calibration** (`appTof_Calibration`): static init, reference calibration,
 *    and SPAD management as recommended by the VL53L0X API.
 *  - **Measurement task** (`appTof_Task`): collects 10 consecutive valid samples,
 *    computes the average, and fires event-bus notifications that drive the LEDs.
 */

#include "app_tof.h"
#include "vl53l0x_api.h"

#define DEBUG_LVL   LVL_INFO   /**< logging level */
#define MOD_NAME    "app tof"    /**< module name */
#include "ez_logging.h"
#include "ez_i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_common.h"
#include "port_i2c.h"
#include "vl53l0x_i2c_platform.h"
#include "app_event_bus.h"


#define TASK_PRIORITY                       10    /**< FreeRTOS task priority for the ToF measurement task. */
#define TASK_STACK_SIZE                     4096  /**< Stack size (bytes) allocated for the ToF task. */
#define THRESHOLD_1_MM                      50    /**< Lower distance threshold (mm): below this → blue LED active. */
#define THRESHOLD_2_MM                      90    /**< Upper distance threshold (mm): below this → red LED active. */
#define THRESHOLD_MAX_COUNT                 10    /**< Number of samples averaged before an LED event is published. */
#define TOF_SENSOR_I2C_ADDRESS              0x29  /**< Default I2C address of the VL53L0X sensor. */

static void appTof_PrintError(VL53L0X_Error Status);
static void appTof_Task(void* arg);
static void appTof_PrintRangeStatus(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
static VL53L0X_Error appTof_Calibration(VL53L0X_Dev_t *pMyDevice);
static VL53L0X_Error appTof_SetupDevice(VL53L0X_Dev_t *pMyDevice);
static VL53L0X_Dev_t tof_sensor;


/**
 * @brief Run a quick 10-shot ranging test and print results via the debug log.
 *
 * This function is used for development / validation purposes only.  It is
 * currently excluded from the production build via the `#if 0` guard inside
 * `appTof_Init`.
 *
 * @param[in] pMyDevice  Pointer to an initialised VL53L0X device structure.
 * @return VL53L0X_ERROR_NONE on success, or a VL53L0X error code on failure.
 */
VL53L0X_Error appTof_RangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t ranging_measure_data;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(status == VL53L0X_ERROR_NONE)
    {
        for(uint8_t i = 0; i < 10; i++)
        {
            EZDEBUG ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            status = VL53L0X_PerformSingleRangingMeasurement(
                pMyDevice,
                &ranging_measure_data);

            appTof_PrintError(status);
            appTof_PrintRangeStatus(&ranging_measure_data);

            VL53L0X_GetLimitCheckCurrent(
                pMyDevice,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                &LimitCheckCurrent);
            EZDEBUG("RANGE IGNORE THRESHOLD: %f\n", (float)LimitCheckCurrent/65536.0);

            if (status != VL53L0X_ERROR_NONE)
            {
                break;
            } 
            EZDEBUG("Measured distance: %i\n", ranging_measure_data.RangeMilliMeter);
        }
    }
    return status;
}

bool appTof_Init(void)
{
    VL53L0X_Error           status = VL53L0X_ERROR_NONE;
    VL53L0X_Version_t       version;
    VL53L0X_DeviceInfo_t    sensor_info;

    // Initialize Comms
    tof_sensor.I2cDevAddr      = TOF_SENSOR_I2C_ADDRESS;
    tof_sensor.comms_type      =  I2C;
    tof_sensor.comms_speed_khz =  100;
    status = VL53L0X_comms_initialise(I2C, 100);

    status = VL53L0X_DataInit(&tof_sensor); // Data initialization

    if(status == VL53L0X_ERROR_NONE)
    {
        status = VL53L0X_GetDeviceInfo(&tof_sensor, &sensor_info);
        if(status == VL53L0X_ERROR_NONE)
        {
            EZINFO("VL53L0X_GetDeviceInfo:");
            EZINFO("Device Name : %s", sensor_info.Name);
            EZINFO("Device Type : %s", sensor_info.Type);
            EZINFO("Device ID : %s", sensor_info.ProductId);
            EZINFO("ProductRevisionMajor : %d", sensor_info.ProductRevisionMajor);
            EZINFO("ProductRevisionMinor : %d", sensor_info.ProductRevisionMinor);

            if ((sensor_info.ProductRevisionMinor != 1) && (sensor_info.ProductRevisionMinor != 1))
            {
                EZINFO("Error expected cut 1.1 but found cut %d.%d",
                    sensor_info.ProductRevisionMajor, sensor_info.ProductRevisionMinor);
                    status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
    }

    status = appTof_Calibration(&tof_sensor);
    if(status != VL53L0X_ERROR_NONE)
    {
        return false;
    }

    status = appTof_SetupDevice(&tof_sensor);
    if(status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(status);
        return false;
    }

#if 1
    xTaskCreate(appTof_Task,
        "ToF_Task",
        TASK_STACK_SIZE, NULL,
        TASK_PRIORITY, NULL);
#else
    if(status == VL53L0X_ERROR_NONE)
    {
        status = appTof_RangingTest(&tof_sensor);
    }
#endif
    return true;
}


/**
 * @brief FreeRTOS task: continuous distance measurement and LED event publishing.
 *
 * Samples the VL53L0X sensor every 50 ms.  Valid measurements (RangeStatus == 0)
 * are accumulated; once @c THRESHOLD_MAX_COUNT samples have been collected the
 * average distance is compared against the two thresholds and the appropriate
 * LED event is published on the application event bus.
 *
 * @param[in] arg  Unused task argument (pass @c NULL).
 */
static void appTof_Task(void* arg)
{
    uint8_t count = 0;
    uint32_t sum_distance_mm = 0;
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t measurement;
    bool led_red_on = false;
    bool led_blue_on = false;
    while(1)
    {
        status = VL53L0X_PerformSingleRangingMeasurement(&tof_sensor, &measurement);
        if(status == VL53L0X_ERROR_NONE && measurement.RangeStatus == 0)
        {
            sum_distance_mm = sum_distance_mm + measurement.RangeMilliMeter;
            count++;
            if(count >= 10)
            {
                uint32_t avg_distance_mm = sum_distance_mm/count;
                EZDEBUG("Average distance: %i mm", avg_distance_mm);
                sum_distance_mm = 0;
                count = 0;
                if(avg_distance_mm < THRESHOLD_1_MM)
                {
                    led_red_on = false;
                    led_blue_on = true;
                }
                else if(avg_distance_mm < THRESHOLD_2_MM)
                {
                    led_red_on = true;
                    led_blue_on = false;
                }
                else
                {
                    led_red_on = false;
                    led_blue_on = false;
                }
                appEventBus_Notify(APP_EVENT_LED_BLUE, &led_blue_on, sizeof(led_blue_on));
                appEventBus_Notify(APP_EVENT_LED_RED, &led_red_on, sizeof(led_red_on));
            }
        }
        else
        {
            appTof_PrintError(status);
            appTof_PrintRangeStatus(&measurement);
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}


/**
 * @brief Log a human-readable description of a VL53L0X error code.
 *
 * @param[in] Status  The VL53L0X error code to describe.
 */
static void appTof_PrintError(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    EZDEBUG("API Status: %i : %s\n", Status, buf);
}


/**
 * @brief Perform the VL53L0X reference calibration sequence.
 *
 * Executes, in order:
 *  1. `VL53L0X_StaticInit`            – device static initialisation.
 *  2. `VL53L0X_PerformRefCalibration` – VHV and phase calibration.
 *  3. `VL53L0X_PerformRefSpadManagement` – reference SPAD selection.
 *
 * @param[in] pMyDevice  Pointer to a comms-initialised VL53L0X device structure.
 * @return VL53L0X_ERROR_NONE on success, or the first error code encountered.
 */
static VL53L0X_Error appTof_Calibration(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    EZDEBUG ("Call of VL53L0X_StaticInit\n");
    Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
    appTof_PrintError(Status);
    
    if(Status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(Status);
        return Status;
    }

    EZDEBUG ("Call of VL53L0X_PerformRefCalibration\n");
    Status = VL53L0X_PerformRefCalibration(
        pMyDevice,
        &VhvSettings,
        &PhaseCal); // Device Initialization

    if(Status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(Status);
        return Status;
    }

    EZDEBUG ("Call of VL53L0X_PerformRefSpadManagement\n");
    Status = VL53L0X_PerformRefSpadManagement(
        pMyDevice,
        &refSpadCount,
        &isApertureSpads); // Device Initialization
    EZDEBUG ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);

    if(Status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(Status);
        return Status;
    }

    return Status;
}


/**
 * @brief Configure the VL53L0X ranging mode and measurement limit checks.
 *
 * Sets the device to @c VL53L0X_DEVICEMODE_SINGLE_RANGING and enables:
 *  - Sigma final range check.
 *  - Signal rate final range check.
 *  - Range ignore threshold check (value: 1.5 × 0.023 in FixPoint1616 format).
 *
 * @param[in] pMyDevice  Pointer to a calibrated VL53L0X device structure.
 * @return VL53L0X_ERROR_NONE on success, or the first error code encountered.
 */
static VL53L0X_Error appTof_SetupDevice(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
    EZDEBUG ("Call of VL53L0X_SetDeviceMode\n");
    status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode

    if(status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(status);
        return status;
    }

    // Enable/Disable Sigma and Signal check
    status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if(status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(status);
        return status;
    }

    status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if(status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(status);
        return status;
    }

    status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    if(status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(status);
        return status;
    }

    status = VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));
    if(status != VL53L0X_ERROR_NONE)
    {
        appTof_PrintError(status);
        return status;
    }
}


/**
 * @brief Log the human-readable range status from a measurement result.
 *
 * @param[in] pRangingMeasurementData  Pointer to the measurement data structure.
 *                                     The function returns immediately if @c NULL.
 */
static void appTof_PrintRangeStatus(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    if(pRangingMeasurementData == NULL)
    {
        return;
    }

    VL53L0X_GetRangeStatusString(pRangingMeasurementData->RangeStatus, buf);
    EZDEBUG("Range Status: %i : %s", pRangingMeasurementData->RangeStatus, buf);
}