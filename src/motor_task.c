/* Standard includes. */
#include "driverlib/pin_map.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "drivers/sensor_data.h"

#include "motorlib.h"

//-------------------------------------------------------------
// Motor State Handling and PID controller
//-------------------------------------------------------------

// Current State Enum
// typedef enum
// {
//     MOTOR_STATE_STOPPED = 0,
//     MOTOR_STATE_IDLING,
//     MOTOR_STATE_RUNNING,
//     MOTOR_STATE_ESTOP
// } MotorState_t;

// Motor State Struct
typedef struct
{
    MotorState_t state;   
    int desiredRPM;     
    float rawRPM;          
    int filteredRPM; 
    float current; 
    float power;     
    int dutyCycle;       
    uint32_t lastEdgeCount; 
} MotorControl_t;

// PID controller Struct
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float i_error;
    float prev_p_error;
} PIDController_t;

//-------------------------------------------------------------
// Globals
//-------------------------------------------------------------
#define EDGES_PER_REV 24.0f // Total edge count per revolution
#define CYCLE_TIME 0.01f

#define HALL_FILTER_ALPHA 0.01f
#define CURRENT_FILTER_ALPHA 0.01f

#define MOTOR_MAX_RPM 6000              // RPM
#define MOTOR_MAX_DUTY 75               // us duty cycle
#define MOTOR_MAX_ACCELERATION 450      // RPM/s
#define MOTOR_MAX_DECELERATION 450      // RPM/s
#define MOTOR_ESTOP_DECELERATION 1000   // RPM/s
#define MOTOR_STALL_THRESHOLD 250       // below this speed - go to idle
#define MOTOR_QUEUE_LENGTH 1

// #define DUTY_GAIN ((float)MOTOR_MAX_DUTY / (float)MOTOR_MAX_RPM) 
#define DUTY_GAIN (250.0f * 24.0f) 
#define MAX_I_ERROR 500.0f

#define CURRENT_ZERO_OFFSET 2048
#define R_SHUNT 0.007f
#define GAIN 10.0f
#define VREF 3.3f
#define ADC_BUF_SIZE 16

extern SemaphoreHandle_t xMotorTimerSemaphore;
extern SemaphoreHandle_t xMotorAPIMutex;
extern SemaphoreHandle_t xCurrentSemaphore;

volatile uint32_t gHallEdgeCount = 0;
volatile float gTimerEdgeCount = 0;

volatile float gIB = 0.0;
volatile float gIC = 0.0;

volatile float gCurrentThreshold = 0.400f;

volatile uint32_t g_ui32TimeStampButton = 0;

volatile MotorControl_t motor = {
    .state = MOTOR_STATE_STOPPED,
    .desiredRPM = 0,
    .rawRPM = 0.0f,
    .filteredRPM = 0,
    .current = 0.0f,
    .power = 0.0f,
    .dutyCycle = 0,
    .lastEdgeCount = 0
};

PIDController_t speedPID = {
    .Kp = 15.0,
    .Ki = 1.2f,    
    .Kd = 0.5f, 
    .i_error = 0.0f,
    .prev_p_error = 0.0f
};

QueueHandle_t xMotorQueue = NULL;

//-------------------------------------------------------------
// Public API & Prototype Functions
//-------------------------------------------------------------

// Static private MotorTask prototype
static void prvMotorTask(void *pvParameters);
static void prvCurrentTask(void *pvParameters);
static void prvUARTPrintFloat(const char *label, float value);

// Extern public (implicit) prototypes
void vCreateMotorTask(void);
void motorStart(void);
void motorStop(void);
void motorSetCurrentThreshold(float current);
void motorSetRPM(int rpm);
void motorEStop(void);
void motorResetEStop(void);

//-------------------------------------------------------------
// Interrupt Handlers and Utils
//-------------------------------------------------------------
static void printHallEdges(bool hallA, bool hallB, bool hallC)
{
    UARTprintf("ISR fired: A=%d B=%d C=%d\n", hallA, hallB, hallC);
}

static void prvEnableHallInts(void)
{
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntEnable(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTN_BASE, GPIO_PIN_2);
}

static void prvDisableHallInts(void)
{
    GPIOIntDisable(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntDisable(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntDisable(GPIO_PORTN_BASE, GPIO_PIN_2);
}

static float prvConvertADCToCurrent(uint32_t adcVal)
{

    // float voltage = (((int32_t)adcVal - CURRENT_ZERO_OFFSET) / 4095.0f) * VREF;
    // float voltage = ((float)adcVal / 4095.0f) * VREF;
    float voltage = ((float)adcVal * VREF) / 4095.0f;
    float current = ((VREF / 2) - voltage) / (GAIN * R_SHUNT);
    // float current = (VREF_DIV2 - voltage) / (GAIN * R_SHUNT);
    // prvUARTPrintFloat("CURRENT: ", current);
    return current;
}

void xHallSensorHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check each interrupt status of each port
    uint32_t statusM = GPIOIntStatus(GPIO_PORTM_BASE, true);
    uint32_t statusH = GPIOIntStatus(GPIO_PORTH_BASE, true);
    uint32_t statusN = GPIOIntStatus(GPIO_PORTN_BASE, true);

    if ((statusM & GPIO_PIN_3) || (statusH & GPIO_PIN_2) || (statusN & GPIO_PIN_2))
    {
        bool hallA = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) ? true : false;
        bool hallB = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) ? true : false;
        bool hallC = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) ? true : false;

        updateMotor(hallA, hallB, hallC);
        // printHallEdges(hallA, hallB, hallC);
        gHallEdgeCount++;
    }

    // Only clear necessary interrupt
    if (statusM & GPIO_PIN_3)
    {
        GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_3);
    }
    if (statusH & GPIO_PIN_2)
    {
        GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
    }
    if (statusN & GPIO_PIN_2)
    {
        GPIOIntClear(GPIO_PORTN_BASE, GPIO_PIN_2);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void xCurrentADCHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t adcValues[2];

    ADCSequenceDataGet(ADC0_BASE, 2, adcValues);
    ADCIntClear(ADC0_BASE, 2);

    gIB = prvConvertADCToCurrent(adcValues[0]);
    gIC = prvConvertADCToCurrent(adcValues[1]);

    // Notify a task that new samples are ready (optional)
    xSemaphoreGiveFromISR(xCurrentSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void xMotorTimerHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear the timer interrupt
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // -- Critical Section -- //
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    gTimerEdgeCount = (float)(gHallEdgeCount);
    gHallEdgeCount = 0;

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    // ---------------------- //

    // ADCProcessorTrigger(ADC0_BASE, 2);

    // Triggger PID loop
    xSemaphoreGiveFromISR(xMotorTimerSemaphore, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void xCurrentTimerHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    // Optional: signal a task/semaphore here
    ADCProcessorTrigger(ADC0_BASE, 2);


    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void xButtonsHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32Status;

    // Read interrupt status
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);
    // Clear interrupt flags
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    uint32_t now = xTaskGetTickCountFromISR();  // Use ISR-safe version

    // Debounce: check if at least 50ms has passed since last press
    if ((now - g_ui32TimeStampButton) > pdMS_TO_TICKS(50))
    {
        if ((ui32Status & USR_SW1) == USR_SW1)
        {
            motorStart();
            xSemaphoreGiveFromISR(xMotorAPIMutex, &xHigherPriorityTaskWoken);
        }
        else if ((ui32Status & USR_SW2) == USR_SW2)
        {
            motorStop();
            xSemaphoreGiveFromISR(xMotorAPIMutex, &xHigherPriorityTaskWoken);
        }

        g_ui32TimeStampButton = now;  // Update timestamp after debounce check
    }

    // Request a context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//-------------------------------------------------------------
// Motor Task and Control API
//-------------------------------------------------------------

void vCreateMotorTask(void)
{
    initMotorLib(MOTOR_MAX_DUTY);

    xMotorQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        MOTOR_QUEUE_LENGTH,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof(Motor_Data_t));

    xTaskCreate(prvMotorTask,
                "Motor",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(prvCurrentTask,
                "Current",                
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
}

/*-----------------------------------------------------------*/

static void prvUARTPrintFloat(const char *label, float value)
{
    int whole = (int)value;
    int decimal = (int)((fabsf(value - whole)) * 100); // two decimal places

    UARTprintf("%s%d.%02d\n", label, whole, decimal);
}

static int prvFilterRPM(void)
{
    // Calculate raw RPM from Hall edges
    motor.rawRPM = (gTimerEdgeCount / EDGES_PER_REV) * (60.0f / CYCLE_TIME);
    // motor.rawRPM = gTimerEdgeCount * 250;

    // Apply EMA filter
    float filteredRPM = (int)((1.0f - HALL_FILTER_ALPHA) * (float)motor.filteredRPM +
                              HALL_FILTER_ALPHA * motor.rawRPM);

    motor.filteredRPM = (int)(filteredRPM + 0.5f);

    return filteredRPM;
}

static int prvCalcControllerError(int currentSetRPM)
{
    float p_error = (float)(currentSetRPM - motor.filteredRPM);

    speedPID.i_error += p_error * CYCLE_TIME;

    if (speedPID.i_error > MAX_I_ERROR)
    {
        speedPID.i_error = MAX_I_ERROR;  
    } 
    if (speedPID.i_error < -MAX_I_ERROR) 
    {
        speedPID.i_error = -MAX_I_ERROR;
    }

    float d_error = (p_error - speedPID.prev_p_error) / CYCLE_TIME;

    float pid = speedPID.Kp * p_error +
                speedPID.Ki * speedPID.i_error +
                speedPID.Kd * d_error;

    speedPID.prev_p_error = p_error;

    // Duty Error
    float ff = (float)MOTOR_MAX_DUTY * (float)currentSetRPM / DUTY_GAIN;
    // float ff = 0;
    float fb = (float)MOTOR_MAX_DUTY * pid / DUTY_GAIN;
    // uint16_t outputDuty = (uint16_t)(ff + fb);
    float outputDuty = ff + fb;

    return outputDuty;
}

static void prvCurrentTask(void *pvParameters)
{
    const float alpha = CURRENT_FILTER_ALPHA;
    // TimerEnable(TIMER5_BASE, TIMER_A);

    for (;;)
    {
        // Wait for new ADC values
        if (xSemaphoreTake(xCurrentSemaphore, portMAX_DELAY) == pdTRUE)
        {
            float IA, IB, IC, I_rms;

            IB = gIB;
            IC = gIC;
            IA = -(IB + IC);

            I_rms = sqrtf(((IA*IA) + (IB*IB) + (IC*IC))/3.0f);
            // I_rms = (fabsf(IA) + fabsf(IB) + fabsf(IC)) / 3.0f;

            if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdTRUE)
            {
                motor.current = (1.0f - alpha) * motor.current + alpha * I_rms;
                motor.power = motor.current * 24;

                if (motor.current > gCurrentThreshold && motor.state != MOTOR_STATE_ESTOP)
                {
                    xSemaphoreGive(xMotorAPIMutex);
                    motorEStop();
                    continue;
                }

                xSemaphoreGive(xMotorAPIMutex);
            }
        }
    }
}

static void prvMotorTask(void *pvParameters)
{
    // Start Timers
    TimerEnable(TIMER2_BASE, TIMER_A);
    // TimerEnable(TIMER0_BASE, TIMER_A);

    uint16_t duty = 0;
    uint16_t outputDuty;
    static int currentSetRPM = 0;
    Motor_Data_t sending_data;

    static int printCounter = 0;

    int filteredRPM = 0;
    int desiredRPM = 0;
    MotorState_t state;

    for (;;)
    {
        if (xSemaphoreTake(xMotorTimerSemaphore, portMAX_DELAY) == pdPASS)
        {
            
            if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdPASS)
            {
                // prvFilterCurrent();
                filteredRPM = prvFilterRPM();
                desiredRPM = motor.desiredRPM;
                state = motor.state;  
                
                xSemaphoreGive(xMotorAPIMutex);
            }

            int deltaRPM = desiredRPM - currentSetRPM;
            int maxDelta; 

            if (motor.state == MOTOR_STATE_ESTOP)
            {
                // E-stop must decelerate faster
                maxDelta = (int)((float)MOTOR_ESTOP_DECELERATION * CYCLE_TIME);
            }
            else if (deltaRPM > 0)
            {
                // accelerating
                maxDelta = (int)((float)MOTOR_MAX_ACCELERATION * CYCLE_TIME);
            }
            else
            {
                // decelerating
                maxDelta = (int)((float)MOTOR_MAX_DECELERATION * CYCLE_TIME);
            }

            // 2) Ramp currentSetRPM toward targetRPM by at most maxDelta
            if (deltaRPM > maxDelta)
            {
                currentSetRPM += maxDelta;
            }
            else if (deltaRPM < -maxDelta)
            {
                currentSetRPM -= maxDelta;
            }
            else
            {
                // within one step, so just snap to it
                currentSetRPM = desiredRPM;
            }

        {    
        switch(state)
        {
            case MOTOR_STATE_STOPPED:
                break;

            case MOTOR_STATE_IDLING:
                break;

            case MOTOR_STATE_ESTOP:
                if (filteredRPM == 0)
                {
                    TimerDisable(TIMER5_BASE, TIMER_A);
                    disableMotor();
                    break;
                }
                if (desiredRPM == 0 && currentSetRPM < MOTOR_STALL_THRESHOLD)
                {
                    stopMotor(false);        
                    prvDisableHallInts();  
                    break;  
                }

            case MOTOR_STATE_RUNNING:
            {
                // if (filteredRPM == 0 && currentSetRPM > 0)
                // {
                //     outputDuty = 10;  // gentle pre-start duty
                //     setDuty(duty);
                //     break;
                // }
                float outputDutyf = prvCalcControllerError(currentSetRPM);

                outputDuty = (uint16_t)(outputDutyf + 0.5f);

                if (outputDuty <= 0)
                {
                    outputDuty = 1;
                    duty = outputDuty;
                    setDuty(duty);
                    // stopMotor(false);
                }
                else if (outputDuty > MOTOR_MAX_DUTY)
                {
                    duty = MOTOR_MAX_DUTY;
                    setDuty(duty);
                }
                else
                {
                    duty = outputDuty;
                    setDuty(duty);
                }

                if (desiredRPM == 0 && filteredRPM < MOTOR_STALL_THRESHOLD)
                {
                    stopMotor(false);        
                    prvDisableHallInts();  
                    
                    if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdPASS)
                    {
                        motor.state = MOTOR_STATE_IDLING;
                        xSemaphoreGive(xMotorAPIMutex);
                    }  
                }

                break;
            }
        }

            if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdPASS)
            {
                motor.dutyCycle = duty;

                sending_data.timestamp = xTaskGetTickCount();
                sending_data.filtered_rpm = motor.filteredRPM;
                sending_data.filtered_power = motor.power;
                sending_data.state = motor.state;

                xQueueSend(xMotorQueue, (void *)&sending_data, (TickType_t)0);

                // static int printCounter = 0;
                if (++printCounter >= 10)
                {
                    // prvUARTPrintFloat("Current: ", (int)(motor.current*1000) );
                    // UARTprintf("outputDUTY: %d\n", outputDuty);
                    // UARTprintf("Filtered RPM = %d, Desired RPM = %d, Duty Cycle = %d\n, ", motor.filteredRPM, motor.desiredRPM, motor.dutyCycle);
                    UARTprintf("%d,%d,%d,%d\n",
                        motor.filteredRPM,   // channel 0
                        motor.desiredRPM,    // channel 1
                        (int)(motor.current*1000),        // channel 3
                        motor.dutyCycle*10
                    );
                    printCounter = 0;
                }

                xSemaphoreGive(xMotorAPIMutex);
                }
            }
        }
    }
}
/*-----------------------------------------------------------*/

static void prvInitMotor(void)
{
    setDuty(0);    // Start with 0 duty
    enableMotor(); // idle with driver enabled
}

static void prvKickstartMotor(void)
{
    if (motor.state != MOTOR_STATE_IDLING)
        return;

    uint16_t kickstartDuty = 20; // Microseconds — must be >0
    motor.dutyCycle = (int)kickstartDuty;

    bool hallA = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
    bool hallB = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2);
    bool hallC = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2);

    updateMotor(hallA, hallB, hallC); // Phase alignment

    prvEnableHallInts();

    setDuty(kickstartDuty); // Set duty before enabling
}

void motorStart(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xSemaphoreTakeFromISR(xMotorAPIMutex, &xHigherPriorityTaskWoken) == pdTRUE)
    {
        if (motor.state == MOTOR_STATE_STOPPED)
        {
            motor.state = MOTOR_STATE_IDLING;
            prvInitMotor();
        }
        TimerEnable(TIMER5_BASE, TIMER_A);
        LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0);
        LEDWrite(LED_D2, LED_D2);
        xSemaphoreGiveFromISR(xMotorAPIMutex, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void motorStop(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xSemaphoreTakeFromISR(xMotorAPIMutex, &xHigherPriorityTaskWoken) == pdTRUE)
    {
        if (motor.state == MOTOR_STATE_IDLING)
        {
            motor.state = MOTOR_STATE_STOPPED;
            TimerDisable(TIMER5_BASE, TIMER_A);
            prvDisableHallInts();
            disableMotor();
        }

        LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0);
        LEDWrite(LED_D3, LED_D3);

        xSemaphoreGiveFromISR(xMotorAPIMutex, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void motorSetCurrentThreshold(float current)
{
    prvUARTPrintFloat("motorSetCurrentThreshold called with: ", current);
    if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdTRUE)
    {
        gCurrentThreshold = current;
        xSemaphoreGive(xMotorAPIMutex);
    }
}

void motorSetRPM(int rpm)
{
    UARTprintf("motorSetRPM called with %d\n", rpm);

    if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdTRUE)
    {
        switch (motor.state)
        {
        case MOTOR_STATE_STOPPED:
            // cannot set rpm until started
            break;

        case MOTOR_STATE_IDLING:

            if (rpm < MOTOR_STALL_THRESHOLD || rpm > MOTOR_MAX_RPM)
            {
                break;
            }   

            motor.desiredRPM = rpm;
            prvKickstartMotor();
            motor.state = MOTOR_STATE_RUNNING;
            break;

        case MOTOR_STATE_RUNNING:

            if (rpm < MOTOR_STALL_THRESHOLD || rpm > MOTOR_MAX_RPM)
            {
                motor.desiredRPM = 0;
                break;
            }

            motor.desiredRPM = rpm;
            break;

        case MOTOR_STATE_ESTOP:
            break;
        }
        xSemaphoreGive(xMotorAPIMutex);
    }
}

void motorEStop(void)
{
    UARTprintf("EStop called\n");
    if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdTRUE)
    {
        // ESTOP can be triggered from ANY state at ANY time (unless stopped of course)
        if (motor.state == MOTOR_STATE_STOPPED)
        {
            // ignore when already stopped
            return;
        }

        if (motor.state == MOTOR_STATE_RUNNING || motor.state == MOTOR_STATE_IDLING)
        {
            motor.desiredRPM = 0;
        }

        LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0);
        LEDWrite(LED_D1, LED_D1);        

        motor.state = MOTOR_STATE_ESTOP;
        xSemaphoreGive(xMotorAPIMutex);
    }
}

void motorResetEStop(void)
{
    if (xSemaphoreTake(xMotorAPIMutex, portMAX_DELAY) == pdTRUE)
    {
        if (motor.state == MOTOR_STATE_ESTOP && motor.filteredRPM == 0)
        {
            motor.state = MOTOR_STATE_STOPPED;
            motor.current = 0;
        }

        LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0);
        LEDWrite(LED_D3, LED_D3);

        xSemaphoreGive(xMotorAPIMutex); 
    } 
}