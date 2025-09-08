/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "drivers/sht31.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "drivers/sensor_data.h"
#include "event_groups.h"
// Sliding window size for calculating moving average
#define SLIDING_WINDOW_SIZE 5
#define TEMP_HUM_QUEUE_LENGTH 5

QueueHandle_t xTemperatureHumidityQueue = NULL;
extern volatile SemaphoreHandle_t xTempThresholdMutex;
extern volatile SemaphoreHandle_t xSHT31Semaphore;
extern volatile SemaphoreHandle_t xI2C2Semaphore;
extern volatile SemaphoreHandle_t xI2C2Mutex;
extern volatile EventGroupHandle_t xEventGroup;
/*-----------------------------------------------------------*/
extern uint32_t g_ui32SysClock;
static void prvSHT31SensorTask( void *pvParameters );
// static void prvConfigureSensor(void);

// static void prvConfigureSht31Timer(void);
void vCreateTemperatureHumidityTask(void);

// SHT31 is a polling based interrupt, there is no dedicated 
// data ready interrupt. Using a timer to signal task to poll
// the sensor every 1 second
void xTemperatureHumidityTimerHandler(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Clear the timer interrupt
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    xSemaphoreGiveFromISR(xSHT31Semaphore, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Function for creating temperature sensor task
void vCreateTemperatureHumidityTask(void){


    xTemperatureHumidityQueue = xQueueCreate(
                        /* The number of items the queue can hold. */
                        TEMP_HUM_QUEUE_LENGTH,
                        /* Size of each item is big enough to hold the
                        whole structure. */
                        sizeof( SHT31_Data_t ) );
    xTaskCreate( prvSHT31SensorTask,
        "Temperature&Humidity",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL );

}

static void prvSHT31SensorTask( void *pvParameters ){
    // prvConfigureSensor();
    //  if(xSemaphoreTake(xI2C2Mutex,  pdMS_TO_TICKS(100)) == pdTRUE){
    
    
    // }
    if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorSHT31Break();
        xSemaphoreGive(xI2C2Mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(250));

    if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorSHT31Reset();
        xSemaphoreGive(xI2C2Mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(250));

    
    if(xSemaphoreTake(xI2C2Mutex, portMAX_DELAY) == pdTRUE){
        sensorSHT31SetFrequency();
        xSemaphoreGive(xI2C2Mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    UARTprintf("Initialised sht31\n");
    // prvConfigureSht31Timer();

    // Store humidity and temperature raw data
    uint8_t data[6] = {0};
    SHT31_Data_t sending_data;

    float sliding_window_temperature[5] = {0.0};
    float sliding_window_humidity[5] = {0.0};
    float temp;
    float humidity;
    float temp_filtered;
    float hum_filtered;
    float local_cooling = 18.0f;
    float local_heating = 18.0f;
    uint8_t index = 0;
    
    // Poll SHT31 sensor every second to get temperature and humidity data
    for(;;){
        if(xSemaphoreTake(xSHT31Semaphore, portMAX_DELAY) == pdPASS){            
            if(xSemaphoreTake(xI2C2Mutex, pdMS_TO_TICKS(100)) == pdPASS){
                readSensorSHT31DataTemp(data);
                xSemaphoreGive(xI2C2Mutex);
            }else{
                continue;
            }
            sensorSHT31Convert(data, &temp, &humidity);
            // Apply sliding window moving average for temp and hum
            sliding_window_temperature[index] = temp;
            sliding_window_humidity[index] = humidity;
            index = (index + 1) % SLIDING_WINDOW_SIZE;
    
            hum_filtered = 0;
            temp_filtered = 0;
            for(uint8_t i = 0; i < SLIDING_WINDOW_SIZE; i++){
                hum_filtered+=sliding_window_humidity[i];
                temp_filtered+=sliding_window_temperature[i];
            }
            hum_filtered = hum_filtered / SLIDING_WINDOW_SIZE;
            temp_filtered = temp_filtered / SLIDING_WINDOW_SIZE;
            sending_data.timestamp = xTaskGetTickCount();
            sending_data.hum_filtered = hum_filtered;
            sending_data.temp_filtered = temp_filtered;
            
            // if (xSemaphoreTake(xTempThresholdMutex, 0) == pdTRUE) {
            //     local_cooling = cooling_threshold;
            //     local_heating = heating_threshold;
            //     xSemaphoreGive(xTempThresholdMutex);
            // }

            // if (temp_filtered < local_cooling) {
            //     xEventGroupSetBits(xEventGroup, EVENT_TEMP_HEATING);
            // } else if (temp_filtered > local_heating) {
            //     xEventGroupSetBits(xEventGroup, EVENT_TEMP_COOLING);
            // }else{
            //     xEventGroupSetBits(xEventGroup, EVENT_NORMAL);
            // }
            xQueueSend( xTemperatureHumidityQueue,  ( void * )&sending_data, ( TickType_t ) 0);
        } 
    }
}