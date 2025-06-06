
#include "app_tof.h"
#include "vl53l0x_api.h"

#include "ez_easy_embedded.h"
#define DEBUG_LVL   LVL_DEBUG   /**< logging level */
#define MOD_NAME    "app_i2c"       /**< module name */
#include "ez_logging.h"
#include "ez_i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_common.h"
#include "port_i2c.h"
#include "vl53l0x_i2c_platform.h"
#include "app_event_bus.h"


#define TASK_PRIORITY                       10
#define TASK_STACK_SIZE                     4096
#define THRESHOLD_1_MM                      50
#define THRESHOLD_2_MM                      90
#define THRESHOLD_MAX_COUNT                 10

static void print_pal_error(VL53L0X_Error Status);
static void appTof_Task(void* arg);
static void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
static VL53L0X_Error calibration(VL53L0X_Dev_t *pMyDevice);
static VL53L0X_Error setupDevice(VL53L0X_Dev_t *pMyDevice);
static VL53L0X_Dev_t MyDevice;

static uint8_t threshold_count[2] = {0, 0};




VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    int i;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        for(i=0;i<10;i++){
            EZDEBUG ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);

            print_pal_error(Status);
            print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
            		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

            EZDEBUG("RANGE IGNORE THRESHOLD: %f\n", (float)LimitCheckCurrent/65536.0);


            if (Status != VL53L0X_ERROR_NONE) break;

            EZDEBUG("Measured distance: %i\n", RangingMeasurementData.RangeMilliMeter);


        }
    }
    return Status;
}

bool appTof_Init(void)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;

    int32_t status_int;
    int32_t init_done = 0;

    // Initialize Comms
    MyDevice.I2cDevAddr      = 0x29;
    MyDevice.comms_type      =  1;
    MyDevice.comms_speed_khz =  100;
    Status = VL53L0X_comms_initialise(I2C, 100);

    Status = VL53L0X_DataInit(&MyDevice); // Data initialization

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        if(Status == VL53L0X_ERROR_NONE)
        {
            EZDEBUG("VL53L0X_GetDeviceInfo:");
            EZDEBUG("Device Name : %s", DeviceInfo.Name);
            EZDEBUG("Device Type : %s", DeviceInfo.Type);
            EZDEBUG("Device ID : %s", DeviceInfo.ProductId);
            EZDEBUG("ProductRevisionMajor : %d", DeviceInfo.ProductRevisionMajor);
            EZDEBUG("ProductRevisionMinor : %d", DeviceInfo.ProductRevisionMinor);

            if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1))
            {
                EZDEBUG("Error expected cut 1.1 but found cut %d.%d",
                    DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                    Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
    }

    Status = calibration(&MyDevice);
    if(Status != VL53L0X_ERROR_NONE)
    {
        return false;
    }

    Status = setupDevice(&MyDevice);
    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return false;
    }

#if 1
    xTaskCreate(appTof_Task,
        "ToF_Task",
        TASK_STACK_SIZE, NULL,
        TASK_PRIORITY, NULL);
#else
    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = rangingTest(pMyDevice);
    }
#endif
    return true;
}


static void appTof_Task(void* arg)
{
    while(1)
    {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        VL53L0X_RangingMeasurementData_t    RangingMeasurementData;

        Status = VL53L0X_PerformSingleRangingMeasurement(&MyDevice, &RangingMeasurementData);
        
        //print_pal_error(Status);
        //print_range_status(&RangingMeasurementData);

        if(Status == VL53L0X_ERROR_NONE && RangingMeasurementData.RangeStatus == 0)
        {
            EZDEBUG("Measured distance: %i mm", RangingMeasurementData.RangeMilliMeter);
            // Check the measured distance against thresholds
            if (RangingMeasurementData.RangeMilliMeter < THRESHOLD_1_MM)
            {
                threshold_count[0]++;
                if(threshold_count[1] > 0)
                {
                    threshold_count[1]--;
                }
                if (threshold_count[0] >= THRESHOLD_MAX_COUNT)
                {
                    EZDEBUG("at: %d mm", THRESHOLD_1_MM);
                    threshold_count[0] = THRESHOLD_MAX_COUNT;
                    appEventBus_Notify(APP_EVENT_LED_BLUE_ON, NULL, NULL);
                }
            }
            else if (RangingMeasurementData.RangeMilliMeter < THRESHOLD_2_MM)
            {
                threshold_count[1]++;
                if(threshold_count[0] > 0)
                {
                    threshold_count[0]--;
                }
                if (threshold_count[1] >= THRESHOLD_MAX_COUNT)
                {
                    EZDEBUG("at: %d mm", THRESHOLD_2_MM);
                    threshold_count[1] = THRESHOLD_MAX_COUNT; // Reset count
                    appEventBus_Notify(APP_EVENT_LED_RED_ON, NULL, NULL);
                }
            }
            else
            {
                if(threshold_count[0] > 0)
                {
                    threshold_count[0]--;
                }
                if(threshold_count[1] > 0)
                {
                    threshold_count[1]--;
                }
            }
        }

        if(threshold_count[0] == 0)
        {
            appEventBus_Notify(APP_EVENT_LED_BLUE_OFF, NULL, NULL);
        }

        if(threshold_count[1] == 0)
        {
            appEventBus_Notify(APP_EVENT_LED_RED_OFF, NULL, NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 1 second
    }
}


static void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    EZDEBUG("API Status: %i : %s\n", Status, buf);
}


static VL53L0X_Error calibration(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    EZDEBUG ("Call of VL53L0X_StaticInit\n");
    Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
    print_pal_error(Status);
    
    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    EZDEBUG ("Call of VL53L0X_PerformRefCalibration\n");
    Status = VL53L0X_PerformRefCalibration(pMyDevice,
            &VhvSettings, &PhaseCal); // Device Initialization

    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    EZDEBUG ("Call of VL53L0X_PerformRefSpadManagement\n");
    Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
            &refSpadCount, &isApertureSpads); // Device Initialization
    EZDEBUG ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);

    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    return Status;
}


static VL53L0X_Error setupDevice(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
    EZDEBUG ("Call of VL53L0X_SetDeviceMode\n");
    Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode

    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    // Enable/Disable Sigma and Signal check
    Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    Status = VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));
    if(Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }
}


static void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    EZDEBUG("Range Status: %i : %s", RangeStatus, buf);

}