/*============================================================

    Pin & Peripheral Map

    GPIO PORT A
    ------------------------------------------------------------
      PA0  - UART0 RX     // Console RX
      PA1  - UART0 TX     // Console TX

    GPIO PORT B
    ------------------------------------------------------------
      PB2  - I2C0 SCL     // I2C Clock
      PB3  - I2C0 SDA     // I2C Data
      
    GPIO PORT C
    ------------------------------------------------------------
      PC6  - GPIO input    // BMI160 Interrupt Handler

    GPIO PORT D
    ------------------------------------------------------------
      PD4
      PD7  - ADC (AIN4 - ISENB)

    GPIO PORT E
    ------------------------------------------------------------
      PE3  - ADC (AIN0 - ISENC)

    GPIO PORT M 
    ------------------------------------------------------------
      PM3  - GPIO Input    // Hall Sensor A 

    GPIO PORT H
    ------------------------------------------------------------
      PH2  - GPIO Input    // Hall Sensor B 

    GPIO PORT N
    ------------------------------------------------------------
      PN2  - GPIO Input    // Hall Sensor C 
      PN4  - I2C2 SDA      // I2C2SDA
      PN5  - I2C2 SCL      // I2C2SCL

    TIMERS
    ------------------------------------------------------------
      TIMER2A - 100 Hz     // Motor PID control loop timer
      TTIME3A - 1 Hz       // Temp hum sensor polling timer

============================================================*/

/* Standard includes. */
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"

/* Hardware Includes */
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"       
#include "inc/hw_adc.h"   
#include "inc/tm4c1294ncpdt.h"           

/* Utils */
#include "utils/uartstdio.h"

/* Header Declaration */
#include "rtos_hw_init.h"

/*-----------------------------------------------------------*/

extern uint32_t g_ui32SysClock;

static void prvConfigureUART(void);   
static void prvConfigureI2C(void);
static void prvConfigureHallSensor(void);
static void prvConfigureMotorTimer(void);
static void prvConfigureCurrentTimer(void);
static void prvGPIOC6_Init(void);
static void prvConfigureI2C2(void);
static void prvConfigureSht31Timer(void);
static void prvConfigureOpt3001Timer(void);
static void prvGPIOD4_Init(void);
static void prvConfigureButton( void );
static void prvConfigureCurrentADC(void);

//-------------------------------------------------------------
// Hardware Setup Entry Point 
//-------------------------------------------------------------

void prvSetupHardware( void )
{
    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);
       
    prvConfigureUART();
    // prvConfigur use this:eI2C();
    prvConfigureHallSensor();
    prvConfigureMotorTimer();
    prvConfigureCurrentTimer();
    prvConfigureSht31Timer();
    prvConfigureOpt3001Timer();
    prvConfigureI2C2();
    
    // prvGPIOC6_Init();
    prvGPIOD4_Init();
    prvConfigureCurrentADC();
  
    /* Configure device pins. */
    PinoutSet(false, false);
    LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0); // turn all off
    LEDWrite(LED_D3, LED_D3);

    prvConfigureButton();
     
    
}

/*-----------------------------------------------------------*/

static void prvConfigureUART(void)
{
    // Enable the GPIO Peripheral and UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Wait for ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}   

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    SysCtlDelay(g_ui32SysClock / 3);  // ~1 second delay
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

/*-----------------------------------------------------------*/

static void prvConfigureI2C(void)
{
    // The I2C0 peripheral must be enabled before use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait for ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.   
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, g_ui32SysClock, false);

    I2CMasterEnable(I2C0_BASE);
    I2CMasterIntEnable(I2C0_BASE);
    IntEnable(INT_I2C0); // Enable interrupt for I2C0
}

/*-----------------------------------------------------------*/

static void prvConfigureHallSensor( void )
{
    // Enable GPIO port Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    // Await peripheral ready state
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));

    // Configure hall sensor pins
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);  // A
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2);  // B
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2);  // C

    // Configure interrupts
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);

    // Clear any weird pending ISRs (Necessary??)
    GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntClear(GPIO_PORTN_BASE, GPIO_PIN_2);

    // Enable Interrupts
    // GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_3);
    // GPIOIntEnable(GPIO_PORTH_BASE, GPIO_PIN_2);
    // GPIOIntEnable(GPIO_PORTN_BASE, GPIO_PIN_2);

    // Enable interrupts in the NVIC 
    IntEnable(INT_GPIOM);
    IntEnable(INT_GPIOH);
    IntEnable(INT_GPION);   
}

/*-----------------------------------------------------------*/

static void prvConfigureMotorTimer(void)
{
    // Use Timer2A (since Timer1 is used by the touchscreen)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2));

    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

    // 100Hz = every 10ms
    uint32_t ui32PeriodA = g_ui32SysClock / 100;

    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32PeriodA - 1);   

    // TimerControlTrigger(TIMER2_BASE, TIMER_A, true);

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER2A); 
}

/*-----------------------------------------------------------*/

static void prvConfigureCurrentTimer(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5));

  TimerConfigure(TIMER5_BASE, TIMER_CFG_A_PERIODIC);  // FULL WIDTH

  uint32_t ticks = g_ui32SysClock / 150;
  TimerLoadSet(TIMER5_BASE, TIMER_A, ticks - 1);

  // Trigger ADC0 Sequencer 2
  // TimerControlTrigger(TIMER5_BASE, TIMER_A, true);
  TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

  IntEnable(INT_TIMER5A); 
}

/*-----------------------------------------------------------*/

static void prvConfigureI2C2(void){
    // The I2C2 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Configure the pin muxing for I2C2 functions on port N4 and N5.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);  // Mux for SCL
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);  // Mux for SDA
    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5); // SCL
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);     // SDA
    I2CMasterInitExpClk(I2C2_BASE, g_ui32SysClock, true);
    I2CMasterEnable(I2C2_BASE);
    I2CMasterIntEnable(I2C2_BASE);
    IntEnable(INT_I2C2); // Enable interrupt for I2C0  
}


/*-----------------------------------------------------------*/

static void prvConfigureSht31Timer(void)
{
    // Use Timer3A (since Timer1 and Timer2 is used by the touchscreen and motor)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3));

    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

    // 1hz = every 1 second
    uint32_t ui32Period = g_ui32SysClock;
    TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period - 1);


    // IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER3A);
    TimerEnable(TIMER3_BASE, TIMER_A);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}

static void prvConfigureOpt3001Timer(void)
{
  // Use Timer4A
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4))
    ;

  TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);

  // every 2 second
  uint32_t ui32Period = g_ui32SysClock / 2;
  TimerLoadSet(TIMER4_BASE, TIMER_A, ui32Period - 1);

  IntEnable(INT_TIMER4A);
  TimerEnable(TIMER4_BASE, TIMER_A);
  TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}


/*-----------------------------------------------------------*/

static void prvGPIOC6_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) {
        // Wait for the GPIOC to be ready
    }

    // Configure PC6 as GPIO input (for INT1 interrupt)
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);

    // Configure interrupt type (rising edge for INT1 active high)
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);  // Use rising edge for active high

    // Clear any pending interrupts
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_6);

    // Enable interrupt for PC6
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_6);

    // Enable the interrupt in the NVIC
    IntEnable(INT_GPIOC);
}

static void prvGPIOD4_Init(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
      // Wait for the GPIOC to be ready
  }

  // Configure PC6 as GPIO input (for INT1 interrupt)
  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4);

  // Configure interrupt type (rising edge for INT1 active high)
  GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);  // Use rising edge for active high

  // Clear any pending interrupts
  GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_4);

  // Enable interrupt for PC6
  GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_4);

  // Enable the interrupt in the NVIC
  IntEnable(INT_GPIOD);
}

static void prvConfigureCurrentADC(void)
{
    // Enable ADC1 and relevant GPIO ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    // Configure GPIOs for analog input
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);  // AIN4 - ISENB
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);  // AIN0 - ISENC

    // ADCSequenceDisable(ADC1_BASE, 2);

    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    // ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_TIMER, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH4);  // AIN4 (PD7 - ISENB)
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  // AIN0 (PE3 - ISENC)
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);
    ADCIntEnable(ADC0_BASE, 2);
    IntEnable(INT_ADC0SS2);

    // ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    // ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH4);  // ISENB
    // ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  // ISENC + interrupt

    // ADCSequenceEnable(ADC1_BASE, 2);
    // ADCIntClear(ADC1_BASE, 2);         
    // ADCIntEnable(ADC1_BASE, 2);        
    // IntEnable(INT_ADC1SS2);            

}

static void prvConfigureButton( void )
{
    /* Initialize the LaunchPad Buttons. */
    ButtonsInit();

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port F interrupt in the NVIC. */
    IntEnable(INT_GPIOJ);
}



