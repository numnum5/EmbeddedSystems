/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "drivers/sensor_data.h"

/*-----------------------------------------------------------*/
// #define LOW_THRESHOLD 40.95f
// #define HIGH_THRESHOLD 2620.8f // calculated with exponent=6 and mantissa=4095
// volatile uint8_t ui8LEDIndex = 0;
/*
 * Time stamp global variable.
 */
// volatile uint32_t g_ui32TimeStamp = 0;

// Sliding window size for calculating moving average
#define SLIDING_WINDOW_SIZE 5
#define LIGHT_QUEUE_LENGTH 5

extern uint32_t g_ui32SysClock;

volatile bool success;
uint16_t rawData;
float convertedLux;
volatile bool sensorActive = true;

extern volatile SemaphoreHandle_t xI2C2Semaphore;
extern volatile SemaphoreHandle_t xI2C2Mutex;
extern volatile SemaphoreHandle_t xOpt3001Semaphore;

QueueHandle_t xLightSensorQueue = NULL;

//-------------------------------------------------------------
// Public API & Prototype Functions
//-------------------------------------------------------------

void vCreateLightSensorTask(void);

static void prvSensorTask(void *pvParameters);

void xOpt3001TimerHandler(void);

void vCreateLightSensorTask(void)
{
    xLightSensorQueue = xQueueCreate(
                        /* The number of items the queue can hold. */
                        LIGHT_QUEUE_LENGTH,
                        /* Size of each item is big enough to hold the
                        whole structure. */
                        sizeof( OPT3001_Data_t ) );
    xTaskCreate(prvSensorTask,
                "Sensor",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
}

static void prvSensorTask(void *pvParameters)
{
    // Hardware setup already done in main
    // Initialise sensor
    if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorOpt3001Init();
        // Test that sensor is set up correctly
        // UARTprintf("Testing OPT3001 Sensor:\n");
        success = sensorOpt3001Test();
        xSemaphoreGive(xI2C2Mutex);
    }

    // // stay here until sensor is working
    // while (!success)
    // {
    //     // SysCtlDelay(g_ui32SysClock);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     UARTprintf("Test Failed, Trying again\n");
    //     success = sensorOpt3001Test();
    // }

    // UARTprintf("All Tests Passed!\n\n");
    float sliding_window_light[5] = {0.0};
    float lux_filtered = 0.0;
    uint8_t index = 0;
    OPT3001_Data_t sending_data;

    // Loop Forever
    while (1)
    {
        if (sensorActive)
        {
            // SysCtlDelay(g_ui32SysClock / 100);
            // vTaskDelay(pdMS_TO_TICKS(100));

            if (xSemaphoreTake(xOpt3001Semaphore, portMAX_DELAY) == pdPASS)
            {
                if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
                    sensorOpt3001Read(&rawData);
                    xSemaphoreGive(xI2C2Mutex);
                }
                

                sensorOpt3001Convert(rawData, &convertedLux);
                sliding_window_light[index] = convertedLux;
                index = (index + 1) % SLIDING_WINDOW_SIZE;

                lux_filtered = 0;
                for (uint8_t i = 0; i < SLIDING_WINDOW_SIZE; i++)
                {
                    lux_filtered += sliding_window_light[i];
                }
                lux_filtered = lux_filtered / SLIDING_WINDOW_SIZE;

                // int int_part = (int)lux_filtered;
                // int decimal_part = (int)((lux_filtered - int_part) * 100);
                // UARTprintf("%d.%02d Lux\n", int_part, decimal_part);
                sending_data.timestamp = xTaskGetTickCount();
                sending_data.filtered_lux = lux_filtered;

                xQueueSend( xLightSensorQueue,  ( void * )&sending_data, ( TickType_t ) 0);
            }
            
        }
    }
}

//-------------------------------------------------------------
// Interrupt Handlers and Utils
//-------------------------------------------------------------
// void xI2C1IntHandler(void)
// {
//     // Clear the I2C interrupt flag.
//     I2CMasterIntClear(I2C1_BASE);
//     I2CSlaveIntClear(I2C1_BASE);
//     // GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_2);

//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     // Unblock the waiting task.
//     xSemaphoreGiveFromISR(xI2C2Semaphore, &xHigherPriorityTaskWoken);

//     // Perform a context switch if needed.
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

//     /* Update the time stamp. */
//     // g_ui32TimeStamp = xTaskGetTickCount();
//     // UARTprintf("I2C0 Interrupt\n");
// }

// Opt3001 is a polling based interrupt, there is no dedicated
// data ready interrupt. Using a timer to signal task to poll
// the sensor every 2 second
void xOpt3001TimerHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Clear the timer interrupt
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    xSemaphoreGiveFromISR(xOpt3001Semaphore, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// // Same interupt used in temp and humidity sensor add to common file
// void xI2C2IntHandler(void)
// {
//     // Clear the I2C interrupt flag.
//     I2CMasterIntClear(I2C2_BASE);
//     I2CSlaveIntClear(I2C2_BASE);
//     // GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_2);

//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     // Unblock the waiting task.
//     xSemaphoreGiveFromISR(xI2C2Semaphore, &xHigherPriorityTaskWoken);

//     // Perform a context switch if needed.
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

//     /* Update the time stamp. */
//     // g_ui32TimeStamp = xTaskGetTickCount();
//     // UARTprintf("I2C0 Interrupt\n");
// }