/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "driverlib/interrupt.h"
#include "drivers/rtos_hw_init.h"
#include "event_groups.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "driverlib/pin_map.h"
/*-----------------------------------------------------------*/

/* The system clock frequency. */
uint32_t g_ui32SysClock;

/* Global for binary semaphore shared between tasks. */
volatile SemaphoreHandle_t xButtonSemaphore = NULL;
volatile SemaphoreHandle_t xSensorSemaphore = NULL;
volatile SemaphoreHandle_t xBMI160Semaphore = NULL;
volatile SemaphoreHandle_t xI2CSemaphore = NULL;
volatile SemaphoreHandle_t xMotorTimerSemaphore = NULL;
volatile SemaphoreHandle_t xCurrentSemaphore = NULL;
volatile SemaphoreHandle_t xSHT31Semaphore = NULL;
volatile SemaphoreHandle_t xI2C2Semaphore = NULL;
volatile SemaphoreHandle_t xOpt3001Semaphore = NULL;
volatile SemaphoreHandle_t xTempThresholdMutex = NULL;
volatile SemaphoreHandle_t xI2C2Mutex = NULL;
volatile EventGroupHandle_t xEventGroup = NULL;
// volatile SemaphoreHandle_t xI2C2Semaphore  = NULL;


volatile SemaphoreHandle_t xBMI160ThresholdMutex = NULL;
/* Global Mutexes */
volatile SemaphoreHandle_t xMotorAPIMutex = NULL;

/* Set up the clock and pin configurations to run this example. */;
extern void vCreateMotorTask(void);
extern void vCreateMotorTestTask( void );
extern void vCreateCurrentTask( void );
extern void vCreateLightSensorTask( void );
extern void vCreateAccelerationTask( void );
extern void vCreateTemperatureHumidityTask( void );
extern void vDisplayDemoTask(void);
extern void vCreateLEDTask( void );
extern void vCreateMotorTask( void );
extern void vCreateMotorTestTask( void );
extern void vCreateCurrentTask( void );
extern void vCreateSensorTask( void );
extern void vCreateAccelerationTask( void );
extern void vCreateTemperatureHumidityTask( void );
/*-----------------------------------------------------------*/
int main(void)
{
    // Setup Hardware
    prvSetupHardware();

    // Create Semaphoress
    xButtonSemaphore = xSemaphoreCreateBinary();
    xBMI160Semaphore = xSemaphoreCreateBinary();
    xSensorSemaphore = xSemaphoreCreateBinary();
    xMotorTimerSemaphore = xSemaphoreCreateBinary();
    xI2CSemaphore = xSemaphoreCreateBinary();
    xSHT31Semaphore = xSemaphoreCreateBinary();
    xI2C2Semaphore = xSemaphoreCreateBinary();
    xCurrentSemaphore = xSemaphoreCreateBinary();

    // Create Mutexes
    xMotorAPIMutex = xSemaphoreCreateMutex();
    
    xOpt3001Semaphore = xSemaphoreCreateBinary();
    xI2C2Mutex = xSemaphoreCreateMutex();
    xTempThresholdMutex = xSemaphoreCreateMutex();
    xBMI160ThresholdMutex = xSemaphoreCreateMutex();

    // Create mutex for preventing race conditions between readings using I2C2
    xCurrentSemaphore = xSemaphoreCreateBinary();

    // Create Mutexes
    xMotorAPIMutex = xSemaphoreCreateMutex();

    if (xButtonSemaphore != NULL &&
        xBMI160Semaphore != NULL &&
        xSensorSemaphore != NULL &&
        xMotorTimerSemaphore != NULL &&
        xI2CSemaphore != NULL &&
        xSHT31Semaphore != NULL &&
        xI2C2Semaphore != NULL &&
        xI2C2Mutex != NULL &&
        xOpt3001Semaphore != NULL && 
        xTempThresholdMutex != NULL &&
        xCurrentSemaphore != NULL &&
        xMotorAPIMutex != NULL && 
        xBMI160ThresholdMutex != NULL)
    {
        {
            // all interrupt config must go before this
            IntMasterEnable();

            // Start tasks
            vCreateLightSensorTask();
            vCreateAccelerationTask();
            vCreateTemperatureHumidityTask();
            vDisplayDemoTask();
            // // vCreateMotorTestTask();
            vCreateMotorTask();

            vTaskStartScheduler();
        }
        /* If all is well, the scheduler will now be running, and the following
        line will never be reached.  If the following line does execute, then
        there was insufficient FreeRTOS heap memory available for the idle and/or
        timer tasks to be created.  See the memory management section on the
        FreeRTOS web site for more details. */
        for (;;);
    }
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    IntMasterDisable();
    for (;;);
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for (;;)
        ;
}

/*-----------------------------------------------------------*/

void *malloc(size_t xSize)
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for (;;)
        ;
}

/*-----------------------------------------------------------*/