/* Standard includes. */
#include "driverlib/pin_map.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

#include "motorlib.h"

void vCreateMotorTestTask( void );

extern void motorStart( void );
extern void motorSetRPM( int rpm );
extern int  motorGetRPM( void );
extern void motorEStop( void );

static void prvMotorTestTask( void *pvParameters );

void vCreateMotorTestTask( void )
{
    xTaskCreate( prvMotorTestTask,
                 "TestMotor",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
}

static void prvMotorTestTask(void *pvParameters)
{
    const int runRPM = 2000;

    UARTprintf("Motor starting...\n");

    // Start and enable motor
    motorStart();

    // Give time for initial commutation alignment if needed
    vTaskDelay(pdMS_TO_TICKS(2));


    UARTprintf("Motor set to 2000...\n");

    // Set desired RPM
    motorSetRPM(runRPM);

    // Run for 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));

    UARTprintf("Motor set to 3000...\n");

    motorSetRPM(3000);

    vTaskDelay(pdMS_TO_TICKS(5000));

    UARTprintf("Motor set to 4000...\n");

    motorSetRPM(4000);

    vTaskDelay(pdMS_TO_TICKS(5000));

    UARTprintf("Motor set to 2000...\n");

    motorSetRPM(2000);

    vTaskDelay(pdMS_TO_TICKS(5000));
    // UARTprintf("Motor set to 1000...\n");
    // motorSetRPM(1000);

    // vTaskDelay(pdMS_TO_TICKS(5000));

    UARTprintf("Motor Braking...\n");

    // Command motor to Estop   
    motorEStop();

    // Wait a moment before disabling (optional)
    vTaskDelay(pdMS_TO_TICKS(2000));
    // disableMotor();

    UARTprintf("Motor stopped.\n");

    // Clean up
    vTaskDelete(NULL);
}
