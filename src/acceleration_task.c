
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>
/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "drivers/opt3001.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "drivers/bmi160.h"
#include "drivers/i2cOptDriver.h"
#include "drivers/i2cBmi160Driver.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "drivers/sensor_data.h"
#include "motorlib.h"

#define FORGETTING_FACTOR 0.8

            
QueueHandle_t xAccelerationQueue = NULL;
#define ACCELERATION_QUEUE_LENGTH 10
volatile uint32_t g_ui32TimeStamp = 0;
extern uint32_t g_ui32SysClock;
extern volatile SemaphoreHandle_t xBMI160Semaphore;
extern volatile SemaphoreHandle_t xI2CSemaphore;
extern volatile SemaphoreHandle_t xI2C2Semaphore;
extern volatile SemaphoreHandle_t xI2C2Mutex;
extern volatile SemaphoreHandle_t xBMI160ThresholdMutex;
extern uint32_t acceleration_threshold;
extern void motorEStop( void );


void vCreateAccelerlometerTask( void );
// static void GPIOC6_Init(void);
// static void prvConfigureSensor(void);
static void prvAccelerationTask( void *pvParameters );


uint8_t data_g;
// BMI160 Interrupt handler
void xGPIOPCHandler(void){
    BaseType_t xLEDTaskWoken = pdFALSE;
    uint32_t status = GPIOIntStatus(GPIO_PORTC_BASE, true);
    GPIOIntClear(GPIO_PORTC_BASE, status);
    if(status & GPIO_INT_PIN_6){
        // UARTprintf("ad");
        xSemaphoreGiveFromISR(xBMI160Semaphore, &xLEDTaskWoken);
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }
}

void xI2C2IntHandler(void)
{
    I2CMasterIntClear(I2C2_BASE);
    I2CSlaveIntClear(I2C2_BASE);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xI2C2Semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void xGPIOPDHandler(void){
    BaseType_t xLEDTaskWoken = pdFALSE;
    uint32_t status = GPIOIntStatus(GPIO_PORTD_BASE, true);
    GPIOIntClear(GPIO_PORTD_BASE, status);
    if(status & GPIO_INT_PIN_4){
        xSemaphoreGiveFromISR(xBMI160Semaphore, &xLEDTaskWoken);
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }
}


void vCreateAccelerationTask( void ){
    // UARTprintf("Intialised BMI160\n");
    xAccelerationQueue = xQueueCreate(
                          /* The number of items the queue can hold. */
                          ACCELERATION_QUEUE_LENGTH,
                          /* Size of each item is big enough to hold the
                          whole structure. */
                          sizeof( BMI160_Data_t ) );;

    UARTprintf("Intialised BMI160\n");
    xTaskCreate(prvAccelerationTask, "Bmi160", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
}


bool first_reading = true;

// Helper function for calculating exponential weighted moving average
float calculate_weighted_average(float current_reading, float * previous_weight, float * previous_average) {
    float current_weight;
    float current_average;

    if (first_reading) {
        current_average = (float)current_reading;
        current_weight = 1.0; 
        first_reading = false;
    } else {
        // Update the weight
        current_weight = (FORGETTING_FACTOR * *previous_weight) + 1.0;

        // Calculate the moving average
        current_average = ((1.0 - (1.0 / current_weight)) * *previous_average) + ((1.0 / current_weight) * current_reading);
    }

    // Update previous values for the next reading
    *previous_weight = current_weight;
    *previous_average = current_average;

    return current_average;
}


static void prvAccelerationTask( void *pvParameters ){

    // Intialise peripheral
     if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorBmi160Reset();
        xSemaphoreGive(xI2C2Mutex);
    }
   
    // Wait for sensor to reset
    vTaskDelay(pdMS_TO_TICKS(250));

    if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorBmi160SetMode();
        xSemaphoreGive(xI2C2Mutex);
    }
    // Wait for the sensor to set the mode
    vTaskDelay(pdMS_TO_TICKS(250));
    
    if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorBmi160Init();
        xSemaphoreGive(xI2C2Mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    // sensorBmi160SetFOC();
        if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorBmi160SetFOC();
        xSemaphoreGive(xI2C2Mutex);
    }
    //  UARTprintf("Intialised 4\n");
    vTaskDelay(pdMS_TO_TICKS(250));

    //  UARTprintf("Intialised 5\n");
    
    // Check if sensor is intialised properly
    // bool success = sensorBmi160Test();
    // while (!success) {
    //     vTaskDelay(pdMS_TO_TICKS(100));
    //     success = sensorBmi160Test();
    // }
//  UARTprintf("Intialised 6\n");
    uint8_t data[6] = {0};
    BMI160_Data_t sending_data;
    TickType_t time;
    float absolute_acc;
    float filtered_acc;
    float previous_weight;
    float previous_average;
    for (;;){

        if(xSemaphoreTake(xBMI160Semaphore, portMAX_DELAY) == pdPASS){
            // Protect shared resouce (xI2C2 sempahore)
            if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
                sensorBmi160ReadAcceleration(data);
                xSemaphoreGive(xI2C2Mutex);
            }
            // else{
            //     continue;
            // }

            int16_t acc_x = (int16_t)((data[1] << 8) | data[0]);
            int16_t acc_y = (int16_t)((data[3] << 8) | data[2]);
            int16_t acc_z = (int16_t)((data[5] << 8) | data[4]);
            
            // Calcuate the absolute acceleration
            absolute_acc = (abs(acc_x)+ abs(acc_y)+ abs(acc_z)) / 3;
            // Filter the data using exponential weighted moving average
            filtered_acc = calculate_weighted_average(absolute_acc, &previous_weight, &previous_average);
            filtered_acc = convertRawData(filtered_acc);

            if(xSemaphoreTake(xBMI160ThresholdMutex, 1) == pdTRUE){
                if(filtered_acc > (float)acceleration_threshold){
                    UARTprintf("CALLING ESTOP!\n");
                    motorEStop();
                }
                xSemaphoreGive(xBMI160ThresholdMutex);
            }

            time = xTaskGetTickCount();
            sending_data.filtered_acceleration = filtered_acc;
            sending_data.timestamp = time;
            xQueueSend( xAccelerationQueue,  ( void * )&sending_data, ( TickType_t ) 0);
        }
    }
}

void xI2C0IntHandler(void)
{
    // Clear the I2C interrupt flag.
    I2CMasterIntClear(I2C0_BASE);
    I2CSlaveIntClear(I2C0_BASE);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Unblock the waiting task.
    xSemaphoreGiveFromISR(xI2CSemaphore, &xHigherPriorityTaskWoken);

    // Perform a context switch if needed.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
