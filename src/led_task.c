// #include <stdio.h>
// #include <stdint.h>
// #include <stdbool.h>

// /* Kernel includes. */
// #include "FreeRTOS.h"
// #include "task.h"
// #include "semphr.h"
// #include <math.h>
// /* Hardware includes. */
// #include "inc/hw_ints.h"
// #include "inc/hw_memmap.h"
// #include "driverlib/gpio.h"
// #include "driverlib/interrupt.h"
// #include "driverlib/sysctl.h"
// #include "drivers/rtos_hw_drivers.h"
// #include "utils/uartstdio.h"
// #include "utils/ustdlib.h"
// #include "drivers/opt3001.h"
// #include "inc/hw_memmap.h"
// #include "driverlib/i2c.h"
// #include "utils/uartstdio.h"
// #include "driverlib/sysctl.h"
// #include "drivers/i2cOptDriver.h"
// // #include "drivers/i2cBmiDriver.h"

// #include "drivers/bmi160.h"
// /*-----------------------------------------------------------*/
// #define BMI160_BASE_ADDR 0x69
// /*
//  * Time stamp global variable.
//  */
// volatile uint32_t g_ui32TimeStamp = 0;
// extern uint32_t g_ui32SysClock;
// /*
//  * Global variable to log the last GPIO button pressed.
//  */
// volatile static uint32_t g_pui32ButtonPressed = NULL;

// /*
//  * The binary semaphore used by the switch ISR & task.
//  */
// extern SemaphoreHandle_t xButtonSemaphore;
// extern SemaphoreHandle_t xSensorSemaphore;

// /*
//  * The tasks as described in the comments at the top of this file.
//  */
// static void prvLEDTask( void *pvParameters );
// static void prvSensorTask( void *pvParameters );
// static void prvAccelormeterTask( void *pvParameters );

// /*
//  * Called by main() to do example specific hardware configurations and to
//  * create the Process Switch task.
//  */
// void vCreateLEDTask( void );
// void vCreateSensorTask( void );

// /* 
//  * Hardware configuration for the LEDs.
//  */
// static void prvConfigureLED( void );
// // static void GPIOF_Init(void);
// /*
//  * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
//  */
// static void prvConfigureButton( void );
// static void GPIOP_Init(void);
// /*-----------------------------------------------------------*/

// // uint16_t rawData = 0;
// // bool success = false;
// // float convertedLux = 0;
// // uint16_t config_reg = 0;
// // void GPIOPPHandler(void){
// //     BaseType_t xLEDTaskWoken = pdFALSE;
// //     uint32_t status = GPIOIntStatus(GPIO_PORTP_BASE, true);
// //     GPIOIntClear(GPIO_PORTP_BASE, status);
// //     if(status & GPIO_INT_PIN_2){
// //         xSemaphoreGiveFromISR(xSensorSemaphore, &xLEDTaskWoken);
// //         portYIELD_FROM_ISR(xLEDTaskWoken);
// //     }
// // }



// // void vCreateSensorTask(void){
// //     bool success = false;
    
// //     // Initialize the sensor

// //     // IntMasterDisable();
// //     sensorOpt3001Init();
// //     GPIOP_Init();
// //     // IntMasterEnable();
// //     SysCtlDelay(g_ui32SysClock);
// //     // Test sensor connectivity
// //     success = sensorOpt3001Test();
// //     while (!success) {
// //         SysCtlDelay(g_ui32SysClock / 100);
// //         UARTprintf("Sensor Test Failed, Trying again\n");
// //         success = sensorOpt3001Test();
// //     }
// //     readConfigReg();
// //     xTaskCreate(prvSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
// // }

// // static void prvSensorTask(void *pvParameters){
// //     // readConfigReg();
// //     for (;;) {
// //         if (xSemaphoreTake(xSensorSemaphore, portMAX_DELAY) == pdPASS) {
// //             config_reg = readConfigReg();
// //             rawData = sensorOpt3001ReadISR();
// //             sensorOpt3001Convert(rawData, &convertedLux);
// //             int int_part = (int)convertedLux;
// //             int decimal_part = (int)((convertedLux - int_part) * 100);
// //             if(config_reg & (0x01 << 6)){
// //                 UARTprintf("High Light Event: %d.%02d Lux\n", int_part, decimal_part);
               
// //             }else if(config_reg & (0x01 << 5)){
// //                 UARTprintf("Low Light Event: %d.%02d Lux\n", int_part, decimal_part);
// //             }else{
// //                 UARTprintf("%d.%02d Lux\n", int_part, decimal_part);
// //             }
// //         }
// //     }
// // }




// // static void GPIOP_Init(void) {
// //     // Enable GPIOC peripheral
// //     // IntMasterDisable();
// //     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
// //     while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)) {
// //         // Wait for GPIOP to be ready
// //     }
// //     GPIOPadConfigSet(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); 
// //     GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE); 
// //     GPIOIntEnable(GPIO_PORTP_BASE, GPIO_PIN_2);
// //     IntEnable(INT_GPIOP2);
// //     // IntMasterEnable();
// // }

// /*-----------------------------------------------------------*/

// void xButtonsHandler( void )
// {
// BaseType_t xLEDTaskWoken;
// uint32_t ui32Status;

//     /* Initialize the xLEDTaskWoken as pdFALSE.  This is required as the
//      * FreeRTOS interrupt safe API will change it if needed should a
//      * context switch be required. */
//     xLEDTaskWoken = pdFALSE;

//     /* Read the buttons interrupt status to find the cause of the interrupt. */
//     ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

//     /* Clear the interrupt. */
//     GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

//     /* Debounce the input with 200ms filter */
//     if ((xTaskGetTickCount() - g_ui32TimeStamp ) > 100)
//     {
//         /* Log which button was pressed to trigger the ISR. */
//         if ((ui32Status & USR_SW1) == USR_SW1)
//         {
//             g_pui32ButtonPressed = USR_SW1;
//         }
//         else if ((ui32Status & USR_SW2) == USR_SW2)
//         {
//             g_pui32ButtonPressed = USR_SW2;
//         }

//         /* Give the semaphore to unblock prvProcessSwitchInputTask.  */
//         xSemaphoreGiveFromISR( xButtonSemaphore, &xLEDTaskWoken );

//         /* This FreeRTOS API call will handle the context switch if it is
//          * required or have no effect if that is not needed. */
//         portYIELD_FROM_ISR( xLEDTaskWoken );
//     }

//     /* Update the time stamp. */
//     g_ui32TimeStamp = xTaskGetTickCount();
// }
// /*-----------------------------------------------------------*/





