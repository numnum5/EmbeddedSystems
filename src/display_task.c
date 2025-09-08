/*
 * hello_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/******************************************************************************
 *
 * The Hello task creates a simple task to handle the UART output for the
 * 'Hello World!' message.  A loop is executed five times with a count down
 * before ending with the self-termination of the task that prints the UART
 * message by use of vTaskDelete.  The loop also includes a one second delay
 * that is achieved by using vTaskDelay.
 *
 * This example uses UARTprintf for output of UART messages.  UARTprintf is not
 * a thread-safe API and is only being used for simplicity of the demonstration
 * and in a controlled manner.
 *
 */

/* Standard includes. */
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
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "checkbox.h"
#include "container.h"
#include "pushbutton.h"
#include "radiobutton.h"
#include "slider.h"
#include "utils/ustdlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
// #include "images.h"

#include "grlib/grlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include "driverlib/gpio.h"
#include "drivers/sensor_data.h"

// Bit masks (defined in a header file or globally in a source file)
#define EVENT_TEMP_COOLING  (1 << 0)
#define EVENT_TEMP_HEATING   (1 << 1)
#define EVENT_BTN_TOGGLE (1 << 2)
// Hard coded limit values
#define OPT3001_SENSOR_MIN 0.01f
#define OPT3001_SENSOR_MAX 1000
#define PLOT_LENGTH 200
#define TENSECONDS_TO_MS 10000
#define GRAPH_HEIGHT 150
#define GRAPH_LEFT_MARGIN 30
#define NUM_CHECK_BOXES 6
// Change this 
#define BMI160_SENSOR_MIN 0.0f
#define BMI160_SENSOR_MAX 157.0f
#define MOTOR_SENSOR_MAX_RPM 4530   
#define MOTOR_SENSOR_MIN_RPM 0  
#define POWER_MIN_LIMIT 0
#define POWER_MAX_LIMIT 50
#define CURRENT_MIN_LIMIT 400
#define CURRENT_MAX_LIMIT 1000
#define SHT31_SENSOR_MIN -40.0f
#define SHT31_SENSOR_MAX 125.0f
#define SHT31_SENSOR_MIN_HUMIDITY 0.0f
#define SHT31_SENSOR_MAX_HUMIDITY 100.0f
#define SENSOR_PAGE_NUMBER 0
#define PLOTTING_PAGE_NUMBER 2
#define STATUS_PAGE_NUMBER 1
#define MS_PER_PIXEL 50
#define GRAPH_TOP_OFFSET 50
// extern float cooling_threshold;
// extern float heating_threshold;
// extern volatile EventGroupHandle_t xEventGroup;
// extern volatile SemaphoreHandle_t xTempThresholdMutex;

extern QueueHandle_t xAccelerationQueue;
extern QueueHandle_t xTemperatureHumidityQueue;
extern QueueHandle_t xLightSensorQueue;
extern volatile SemaphoreHandle_t xBMI160ThresholdMutex;
extern QueueHandle_t xMotorQueue;
// volatile uint32_t g_ui32TimeStampButton = 0;
volatile static uint32_t g_pui32ButtonPressed = 0;
static MotorState_t motor_state = MOTOR_STATE_STOPPED;
static int32_t cooling_threshold = 38;
static int32_t heating_threshold = 16;
static uint32_t acceleration_threshold_display = 30;
uint32_t acceleration_threshold = 30;
static int32_t day_night_threshold = 5; // Lux value for day/night threshold
static bool desiredRPM_set = false;
static int32_t desiredRPM = 0;
static bool currentLimit_set = false;
static int32_t currentLimit = 1; 
static bool estop_reset_hit = false; // Flag to indicate if E-Stop reset button was hit

// Private global variables
static uint32_t lastX = 40, lastY = 120;
static uint32_t newX = 40, newY = 0;
static uint32_t currentTime_LightSensor;
static uint32_t currentTime_TempHumSensor;
static uint32_t currentTime_AccelerationSensor;
static uint32_t currentTime_Motor;
static float filteredLux = 0.0;
static float filteredAcceleration = 0;
static float filteredTemperature = 0;
static float filteredHumidity = 0;
static float filteredRPM = 0;
static float filteredPower = 0;
static uint32_t start_time = 0;


// API to set the desired RPM
extern void motorSetRPM(int rpm);
extern void motorSetCurrentThreshold(float current);
extern void motorResetEStop();
/*-----------------------------------------------------------*/

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Gloal variable used to store the frequency of the system clock.
//
//*****************************************************************************
uint32_t g_ui32SysClock;
tContext sContext;

// Extern for the sensor data queue (used to receive data from sensor task)
extern QueueHandle_t xSensorDataQueue;

// Extern for the event group handle (used to detect threshold/BTN events)
extern EventGroupHandle_t xSensorEventGroup;

//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************
#ifdef ewarm
#pragma data_alignment = 1024
tDMAControlTable psDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(psDMAControlTable, 1024)
tDMAControlTable psDMAControlTable[64];
#else
tDMAControlTable psDMAControlTable[64] __attribute__((aligned(1024)));
#endif

//*****************************************************************************
void OnPrevious(tWidget *psWidget);
void OnNext(tWidget *psWidget);
void HandleNavgiation(tWidget * psWidget);
void OnIntroPaint(tWidget *psWidget, tContext *psContext);
void OnCanvasPaint(tWidget *psWidget, tContext *psContext);
void OnCheckChange(tWidget *psWidget, uint32_t bSelected);
void OnButtonPress(tWidget *psWidget);
void OnZoomOutClick(tWidget * psWidget);
void OnZoomInClick(tWidget * psWidget);
void OnTempHeatLimHandle(tWidget *psWidget, int32_t i32Value);
void OnTempCoolLimHandle(tWidget *psWidget, int32_t i32Value);
void OnCurrentLimitHandle(tWidget *psWidget, int32_t i32Value);
void OnRMPSpeedHandle(tWidget *psWidget, int32_t i32Value);
void OnSliderChangeAcceleration(tWidget *psWidget, int32_t i32Value);
void OnEstopHandler(tWidget * psWidget);
extern tCanvasWidget g_psPanels[];

typedef enum {
    COOLING,
    HEATING,
    NORMAL
} TempState_t;

typedef enum {
    Day,
    Night
} LightState_t;

typedef enum {
    TEMPERATURE = 0,
    SPEED,
    LIGHT,
    HUMIDITY,
    ACCELERATION,
    POWER,
    NONE
} PlotType_t;

typedef enum{
    HOME,
    SENSOR_CONTROLS,
    PLOT,
} Page;



// Temp, speed, light, humidity, accel, power, none
static char * top_labels[] = {"125", "4530", "1000", "100", "157", "1000", ""};
static char * bottom_labels[] = {"-40", "0", "0", "0", "0", "400", ""};

static char * top_labels_zoom2[] = {"62.5", "2265", "500", "50", "78.5", "500", ""};
static char * bottom_labels_zoom2[] = {"-20", "0", "0", "0", "0", "200", ""};

static char * top_labels_zoom3[] = {"41.7", "1510", "333", "33.3", "52.3", "333", ""};
static char * bottom_labels_zoom3[] = {"-13.33", "0", "0", "0", "0", "133", ""};

static char * top_labels_zoom4[] = {"31.25", "1132", "250", "25", "39.25", "250", ""};
static char * bottom_labels_zoom4[] = {"-10", "0", "0", "0", "0", "100", ""};

static char * top_labels_zoom5[] = {"25", "906", "200", "20", "31.4", "200", ""};
static char * bottom_labels_zoom5[] = {"-8", "0", "0", "0", "0", "80", ""};



static bool start_new_plot = false;
static PlotType_t selectedPlotType = NONE;
static uint8_t zoomFactor = 1;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvDisplayDemoTask( void *pvParameters );

/*
 * Called by main() to create the Hello print task.
 */
void vDisplayDemoTask( void );

//*****************************************************************************
//
// The first panel, status 
//
//*****************************************************************************


tCanvasWidget boxes[] = {
    CanvasStruct(g_psPanels, boxes + 1, 0,
                &g_sKentec320x240x16_SSD2119, 15, 70, 90, 40,
                CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_OUTLINE, ClrYellow, ClrWhite, 0, &g_sFontCm16, "Day", 0, 0),
  
    CanvasStruct(g_psPanels, 0, 0,
                &g_sKentec320x240x16_SSD2119, 215, 70, 90, 40,
                CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_OUTLINE, ClrLightGreen, ClrWhite, 0, &g_sFontCm16,"Normal", 0, 0),
};

tPushButtonWidget estop_button[] =
{
  RectangularButtonStruct(g_psPanels, boxes, 0,
        &g_sKentec320x240x16_SSD2119, 115, 70, 90, 40,
        PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_FILL, ClrGreen, ClrWhite, ClrWhite,
        ClrWhite, &g_sFontCm16, "Stopped", 0, 0, 0, 0, OnEstopHandler),
};

void OnEstopHandler(tWidget * psWidget){
    if(motor_state == MOTOR_STATE_ESTOP){
        if (filteredRPM == 0)
        {          
            estop_reset_hit = true; // Set the flag to indicate that the E-Stop reset button was hit
        }
    }
}

tCanvasWidget status_boxes_labels[] =
{
    CanvasStruct(g_psPanels, status_boxes_labels + 1, 0,
                 &g_sKentec320x240x16_SSD2119, 22, 110, // Position above the first slider
                 80, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT | CANVAS_STYLE_FILL),
                 ClrBlack, 0, ClrWhite,
                 &g_sFontCm16, "700.55 Lux", 0, 0),
    CanvasStruct(g_psPanels, status_boxes_labels  + 2, 0,
                 &g_sKentec320x240x16_SSD2119, 130, 110, // Position above the second slider
                 80, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT | CANVAS_STYLE_FILL),
                 ClrBlack, 0, ClrWhite,
                 &g_sFontCm16, "0 RMP", 0, 0),
    CanvasStruct(g_psPanels, status_boxes_labels + 3, 0,
                 &g_sKentec320x240x16_SSD2119, 90, 45, // Position above the second slider
                 160, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT | CANVAS_STYLE_FILL),
                 ClrBlack, 0, ClrWhite,
                 &g_sFontCm16, "Date: 14:15:00 29/5/2025", 0, 0),
        // Label for Slider 2
    CanvasStruct(g_psPanels, estop_button, 0,
                 &g_sKentec320x240x16_SSD2119, 220, 110, // Position above the second slider
                 90, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT | CANVAS_STYLE_FILL),
                 ClrBlack, 0, ClrWhite,
                 &g_sFontCm16, "23 Celcius", 0, 0),
};

// Define Slider Widgets
tSliderWidget status_Sliders[] =
{
    // First slider
    SliderStruct(g_psPanels, status_Sliders + 1, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 160, 300, 20, MOTOR_SENSOR_MIN_RPM, MOTOR_SENSOR_MAX_RPM, 0,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm18, "0 RPM", 0, 0, OnRMPSpeedHandle),

    // Second slider
    SliderStruct(g_psPanels, status_boxes_labels, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 200, 300, 20, CURRENT_MIN_LIMIT, CURRENT_MAX_LIMIT, 400,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm18, "400 mA", 0, 0, OnCurrentLimitHandle),
};

void OnCurrentLimitHandle(tWidget *psWidget, int32_t i32Value){
    static char Text[30];
    currentLimit = i32Value; // Update the global variable
    currentLimit_set = true; // Set the flag to indicate that the current limit is set
    usprintf(Text, "%d mA", i32Value);
    SliderTextSet(psWidget, Text);
    WidgetPaint((tWidget *)psWidget);
}
void OnRMPSpeedHandle(tWidget *psWidget, int32_t i32Value){
    static char Text[30];
    desiredRPM = i32Value; // Update the global variable
    desiredRPM_set = true; // Set the flag to indicate that the desired RPM is set
    usprintf(Text, "%d RPM", i32Value);
    SliderTextSet(psWidget, Text);
    WidgetPaint((tWidget *)psWidget);
}

// GrStringDraw();

// Define Labels Above Each Slider
tCanvasWidget g_sSliderLabel[] = {
    // Label for Slider 1
    CanvasStruct(status_Sliders, g_sSliderLabel + 1, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 140, // Position above the first slider
                 140, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT),
                 0, 0, ClrWhite,
                 &g_sFontCm18, "Speed", 0, 0),

    // Label for Slider 2
    CanvasStruct(status_Sliders, status_Sliders, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 180, // Position above the second slider
                 140, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT),
                 0, 0, ClrWhite,
                 &g_sFontCm18, "Current Limit", 0, 0),
};


// Rectangular shape to show status

// Root Canvas (panel) that holds the sliders and labels
Canvas(g_sIntroduction, g_psPanels, status_Sliders, g_sSliderLabel,
       &g_sKentec320x240x16_SSD2119, 0, 40,
       320, 240, CANVAS_STYLE_FILL, 
       0, 0, 0, 0, 0, 0, 0);

//*****************************************************************************
//
// The second panel, which demonstrates the graphics primitives.
//
//*****************************************************************************
tSliderWidget g_Sensors[] =
{
    
    SliderStruct(g_psPanels + 1, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 70, 300, 20, SHT31_SENSOR_MIN, SHT31_SENSOR_MAX, 16,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm18, "16 C", 0, 0, OnTempHeatLimHandle),
    SliderStruct(g_psPanels + 1,  0, 0,
                &g_sKentec320x240x16_SSD2119, 10, 110, 300, 20, SHT31_SENSOR_MIN, SHT31_SENSOR_MAX, 38,
                (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                &g_sFontCm18, "38 C", 0, 0, OnTempCoolLimHandle),
     SliderStruct(g_psPanels + 1, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 150, 300, 20, 0, 157, 30,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm18, "30 m/s^2", 0, 0, OnSliderChangeAcceleration),
};

void OnSliderChangeAcceleration(tWidget *psWidget, int32_t i32Value){
    static char Text[30];

    // UARTprintf("DAJ");
    acceleration_threshold_display = i32Value;
    usprintf(Text, "%d m/s^2", i32Value);
    SliderTextSet(psWidget, Text);
    WidgetPaint((tWidget *)psWidget);
}

tCanvasWidget sensors_Labels[] = {
    // Label for Slider 1
    CanvasStruct(g_psPanels + 1, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 50, // Position above the first slider
                 150, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT),
                 0, 0, ClrWhite,
                 &g_sFontCm18, "Temp Heating Limit", 0, 0),

    // Label for Slider 2
    CanvasStruct(g_psPanels + 1, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 90, // Position above the second slider
                 150, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT),
                 0, 0, ClrWhite,
                 &g_sFontCm18, "Temp Cooling Limit", 0, 0),
        // Label for Slider 2
    CanvasStruct(g_psPanels + 1, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 130, // Position above the second slider
                 150, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT),
                 0, 0, ClrWhite,
                 &g_sFontCm18, "Acceleration Limit", 0, 0),
};

void OnTempHeatLimHandle(tWidget *psWidget, int32_t i32Value){
    static char Text[30];
    heating_threshold = i32Value;
    usprintf(Text, "%d C", i32Value);
    SliderTextSet(psWidget, Text);
    WidgetPaint((tWidget *)psWidget);
}
void OnTempCoolLimHandle(tWidget *psWidget, int32_t i32Value){
    static char Text[30];
    cooling_threshold = i32Value;
    usprintf(Text, "%d C", i32Value);
    SliderTextSet(psWidget, Text);
    WidgetPaint((tWidget *)psWidget);
}

tCheckBoxWidget g_psCheckBoxes[] =
{
    CheckBoxStruct(g_psPanels + 2, g_psCheckBoxes + 1, 0,
                   &g_sKentec320x240x16_SSD2119, 240, 50, 80 , 20,
                   CB_STYLE_OUTLINE | CB_STYLE_TEXT, 16,
                   0, ClrWhite, ClrWhite, &g_sFontCm12, "Temp",
                   0, OnCheckChange),
    CheckBoxStruct(g_psPanels + 2, g_psCheckBoxes + 2, 0,
                &g_sKentec320x240x16_SSD2119, 240, 70, 80, 20,
                CB_STYLE_OUTLINE | CB_STYLE_TEXT, 16,
                0, ClrWhite, ClrWhite, &g_sFontCm12, "Speed",
                0, OnCheckChange),
    CheckBoxStruct(g_psPanels + 2, g_psCheckBoxes + 3, 0,
            &g_sKentec320x240x16_SSD2119, 240, 90, 80, 20,
            CB_STYLE_OUTLINE | CB_STYLE_TEXT, 16,
            0, ClrWhite, ClrWhite, &g_sFontCm12, "Light",
            0, OnCheckChange),
    CheckBoxStruct(g_psPanels + 2, g_psCheckBoxes + 4, 0,
            &g_sKentec320x240x16_SSD2119, 240, 110, 80, 20,
            CB_STYLE_OUTLINE | CB_STYLE_TEXT, 16,
            0, ClrWhite, ClrWhite, &g_sFontCm12, "Humidity",
            0, OnCheckChange),   
    CheckBoxStruct(g_psPanels + 2, g_psCheckBoxes + 5, 0,
            &g_sKentec320x240x16_SSD2119, 240, 130, 80, 20,
            CB_STYLE_OUTLINE | CB_STYLE_TEXT, 16,
            0, ClrWhite, ClrWhite, &g_sFontCm12, "Accel",
            0, OnCheckChange),          
    CheckBoxStruct(g_psPanels + 2, 0, 0,
            &g_sKentec320x240x16_SSD2119, 240, 150, 80, 20,
            CB_STYLE_OUTLINE | CB_STYLE_TEXT, 16,
            0, ClrWhite, ClrWhite, &g_sFontCm12, "Power",
            0, OnCheckChange),    
};


tCanvasWidget data_display[] = {
    CanvasStruct(g_psPanels + 2, data_display + 1, 0,
                 &g_sKentec320x240x16_SSD2119, 10, 220, // Position above the second slider
                 150, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT),
                 0, 0, ClrWhite,
                 &g_sFontCm16, "Data: ", 0, 0),
    CanvasStruct(g_psPanels + 2, data_display + 2, 0,
                 &g_sKentec320x240x16_SSD2119, 50, 220, // Position above the second slider
                 100, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT | CANVAS_STYLE_FILL),
                 0, 0, ClrWhite,
                 &g_sFontCm16, "", 0, 0),
    CanvasStruct(g_psPanels + 2, g_psCheckBoxes, 0,
                &g_sKentec320x240x16_SSD2119, 160, 220, // Position above the second slider
                80, 20, (CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT | CANVAS_STYLE_FILL),
                0, 0, ClrWhite,
                &g_sFontCm16, "Factor: 1x", 0, 0),
};


tPushButtonWidget g_psButtons[] =
{
    RectangularButtonStruct(g_psPanels + 2,  g_psButtons + 1, 0,
        &g_sKentec320x240x16_SSD2119, 240, 180, 80, 24,
        PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_FILL, ClrPurple, ClrWhite, ClrWhite,
        ClrWhite, &g_sFontCm12, "Zoom In", 0, 0, 0, 0, OnZoomInClick),

    RectangularButtonStruct(g_psPanels + 2, data_display, 0,
        &g_sKentec320x240x16_SSD2119, 240, 210, 80, 24,
        PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_FILL, ClrBlue, ClrWhite, ClrWhite,
        ClrWhite, &g_sFontCm12, "Zoom Out", 0, 0, 0, 0, OnZoomOutClick)
};






Canvas(g_sPlottingCanvas, g_psPanels + 2, g_psButtons, 0, &g_sKentec320x240x16_SSD2119, GRAPH_LEFT_MARGIN,
       GRAPH_TOP_OFFSET, 201, 151, CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN, 0, ClrWhite,
       0, 0, 0, 0, OnCanvasPaint);

void OnZoomOutClick(tWidget * psWidget){
    static char Text[30];
    if(zoomFactor > 1)
    {
        zoomFactor--;
        usprintf(Text, "Factor: %ux", zoomFactor);
        CanvasTextSet(&data_display[2], Text);
        WidgetPaint((tWidget *)&data_display[2]);
        start_new_plot = true;
    }
}

void OnZoomInClick(tWidget * psWidget){
    static char Text[30];
    if(zoomFactor < 5)
    {
        zoomFactor++;
        usprintf(Text, "Factor: %ux", zoomFactor);
        CanvasTextSet(&data_display[2], Text);
        WidgetPaint((tWidget *)&data_display[2]);
        start_new_plot = true;
    }
}

void CheckBoxSetState(tCheckBoxWidget *pCheckbox, bool bSelected) {

    if (bSelected) {
        pCheckbox->ui16Style |= CB_STYLE_SELECTED;
    } else {
        pCheckbox->ui16Style &= ~CB_STYLE_SELECTED;
    }
}
void OnCheckChange(tWidget *pWidget, uint32_t bSelected) {
    // Cast the widget back to a checkbox
    tCheckBoxWidget *pSelectedCheckBox = (tCheckBoxWidget *)pWidget;
    if (bSelected) {
        // Loop through all checkboxes
        for (int i = 0; i < NUM_CHECK_BOXES; i++) {
            // Uncheck others
            if ((tCheckBoxWidget *)&g_psCheckBoxes[i] != pSelectedCheckBox) {
                CheckBoxSetState(&g_psCheckBoxes[i], false);
                WidgetPaint((tWidget *)&g_psCheckBoxes[i]);
            }else{
                selectedPlotType = i;
                start_new_plot = true;
            }
        }
    }else{
        selectedPlotType = NONE;
        start_new_plot = true;
    }
}

//*****************************************************************************
//
// An array of canvas widgets, one per panel.  Each canvas is filled with
// black, overwriting the contents of the previous panel.
//
//*****************************************************************************
tCanvasWidget g_psPanels[] =
{
    CanvasStruct(0, 0, &g_sIntroduction, &g_sKentec320x240x16_SSD2119, 0, 40,
                 320, 200, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
    CanvasStruct(0, 0, &sensors_Labels[0], &g_sKentec320x240x16_SSD2119, 0, 40,
                 320, 200, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
    CanvasStruct(0, 0, &g_sPlottingCanvas, &g_sKentec320x240x16_SSD2119, 0, 40, 320,
                 200, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
};

//*****************************************************************************
//
// Navigation bar
//
//*****************************************************************************

RectangularButton(g_sSensors,  
0,    
0,               
0,               
&g_sKentec320x240x16_SSD2119,       
0, 0,            
107, 40,          
(PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT), 
ClrDarkBlue,       
ClrRed,      
ClrWhite,          
ClrWhite,       
g_psFontCm16,     
"Sensors",     
0, 0,            
0, 0,              
HandleNavgiation);    

RectangularButton(g_sStatus,   
0,                
0,                 
0,              
&g_sKentec320x240x16_SSD2119,     
107, 0,           
106, 40,          
(PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT), 
ClrDarkBlue,         
ClrRed,     
ClrWhite,  
ClrWhite,          
g_psFontCm16,    
"Status",      
0, 0,              
0, 0,              
HandleNavgiation);   

RectangularButton(g_sPlotting,       // sName
0,              
0,                
0,                 
&g_sKentec320x240x16_SSD2119,       
213, 0,           
107, 40,         
(PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT), 
ClrDarkBlue,           
ClrRed,       
ClrWhite,          
ClrWhite,          
g_psFontCm16,      
"Plotting",        
0, 0,             
0, 0,             
HandleNavgiation);    

//*****************************************************************************
//
// The panel that is currently being displayed.
//
//*****************************************************************************
uint32_t g_ui32Panel;

//*****************************************************************************
//
// Handles presses of the previous panel button.
//
//*****************************************************************************
void HandleNavgiation(tWidget * psWidget)
{
    // Reset all buttons to inactive color
    PushButtonFillColorSet(&g_sPlotting, ClrDarkBlue);
    PushButtonFillColorSet(&g_sSensors, ClrDarkBlue);
    PushButtonFillColorSet(&g_sStatus, ClrDarkBlue);

    // Repaint all nav buttons
    WidgetPaint((tWidget *)&g_sPlotting);
    WidgetPaint((tWidget *)&g_sSensors);
    WidgetPaint((tWidget *)&g_sStatus);

    // Remove all panels first
    WidgetRemove((tWidget *)(g_psPanels + 0)); // Status
    WidgetRemove((tWidget *)(g_psPanels + 1)); // Sensors
    WidgetRemove((tWidget *)(g_psPanels + 2)); // Plotting


    char buffer_temp_heating[30];
    char buffer_temp_cooling[30];
    if (psWidget == (tWidget *)&g_sPlotting)
    {
        start_new_plot = true;
        zoomFactor = 1;
        g_ui32Panel = 2;
        PushButtonFillColorSet(&g_sPlotting, ClrRed);
        WidgetPaint((tWidget *)&g_sPlotting);
        WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + 2));
        WidgetPaint((tWidget *)(g_psPanels + 2));
    }
    else if (psWidget == (tWidget *)&g_sSensors)
    {
        g_ui32Panel = 0;
        usprintf(buffer_temp_heating, "%d C", heating_threshold);
        SliderTextSet(&sensors_Labels[0], buffer_temp_heating);
        WidgetPaint((tWidget *)&sensors_Labels[0]);
        usprintf(buffer_temp_cooling, "%d C", cooling_threshold);
        SliderTextSet(&sensors_Labels[0], buffer_temp_cooling);


        WidgetPaint((tWidget *)&sensors_Labels[0]);
        PushButtonFillColorSet(&g_sSensors, ClrRed);
        WidgetPaint((tWidget *)&g_sSensors);
        WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + 1));
        WidgetPaint((tWidget *)(g_psPanels + 1));
    }
    else if (psWidget == (tWidget *)&g_sStatus)
    {
        g_ui32Panel = 1;
        PushButtonFillColorSet(&g_sStatus, ClrRed);
        WidgetPaint((tWidget *)&g_sStatus);
        WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + 0));
        WidgetPaint((tWidget *)(g_psPanels + 0));
    }
}


//*****************************************************************************
//
// Handles paint requests for the canvas demonstration widget.
//
//*****************************************************************************

// // Private global variables
// static uint32_t lastX = 40, lastY = 120;
// static uint32_t newX = 40, newY = 0;
// static uint32_t currentTime_LightSensor;
// static uint32_t currentTime_TempHumSensor;
// static uint32_t currentTime_AccelerationSensor;
// static uint32_t currentTime_Motor;
// static float filteredLux = 0.0;
// static float filteredAcceleration = 0;
// static float filteredTemperature = 0;
// static float filteredHumidity = 0;
// static float filteredRPM = 0;
// static uint32_t start_time = 0;

void OnCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    if(start_new_plot){
        // Draws outline of plot
        GrContextForegroundSet(&sContext, ClrBlack);
        tRectangle clearRect = {.i16XMin = 0, .i16YMin = GRAPH_TOP_OFFSET - 10, .i16XMax = GRAPH_LEFT_MARGIN + 209, .i16YMax = 220};
        GrRectFill(&sContext, &clearRect);

        uint16_t yPosition;
        float normalizedValue;
        
        switch(selectedPlotType){

            case TEMPERATURE:
                start_time = currentTime_TempHumSensor;
                normalizedValue = (filteredTemperature - SHT31_SENSOR_MIN) / (SHT31_SENSOR_MAX - SHT31_SENSOR_MIN);
                yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;

                

                break;
            case SPEED:
                start_time = currentTime_Motor;
                normalizedValue = (filteredRPM - MOTOR_SENSOR_MIN_RPM) / (MOTOR_SENSOR_MAX_RPM - MOTOR_SENSOR_MIN_RPM);
                yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                break;
            case LIGHT:
                // For opt3001 sensor
                start_time = currentTime_LightSensor;
                normalizedValue = (filteredLux - OPT3001_SENSOR_MIN) / (OPT3001_SENSOR_MAX - OPT3001_SENSOR_MIN);
                yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                break;
            case HUMIDITY:
                start_time = currentTime_TempHumSensor;
                normalizedValue = (filteredHumidity - SHT31_SENSOR_MIN_HUMIDITY) / (SHT31_SENSOR_MAX_HUMIDITY - SHT31_SENSOR_MIN_HUMIDITY);
                yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                break;
            case ACCELERATION:
                // For bmi160 sensor
                start_time = currentTime_AccelerationSensor;
                normalizedValue = (filteredAcceleration - BMI160_SENSOR_MIN) / (BMI160_SENSOR_MAX - BMI160_SENSOR_MIN);
                yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                break;
            case POWER:
                start_time = currentTime_Motor;
                normalizedValue = (filteredLux - POWER_MAX_LIMIT) / (POWER_MAX_LIMIT - POWER_MIN_LIMIT);
                yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                break;
            case NONE:
                yPosition = GRAPH_HEIGHT + GRAPH_TOP_OFFSET;
                break;
            default:
                yPosition = GRAPH_HEIGHT + GRAPH_TOP_OFFSET;
                break;
        }
        
        lastX = GRAPH_LEFT_MARGIN;
        lastY = yPosition;
        newX = lastX;
        newY = lastY;

        GrContextForegroundSet(&sContext, ClrWhite);
        GrContextFontSet(&sContext, &g_sFontCm14);


        // Draw vertical grids
        for (int i = 0; i <= 10; i++) {
        uint16_t x = GRAPH_LEFT_MARGIN + i * (PLOT_LENGTH / 10); // 20 * i
                GrContextForegroundSet(&sContext, ClrLightGrey);  // or any dim color for grid
                GrLineDraw(&sContext, x, 50, x, 200);  // from top to bottom of graph area
        }
        GrContextForegroundSet(&sContext, ClrWhite);
        // Labels for axies


        
        char ** selected_top_label = top_labels;
        char ** selected_bottom_label = bottom_labels;


        // Change y axis labels based on zoom factor
        if(zoomFactor == 1){
            selected_top_label = top_labels;
            selected_bottom_label = bottom_labels;
        }else if(zoomFactor == 2){
            selected_top_label = top_labels_zoom2;
            selected_bottom_label = bottom_labels_zoom2;
        }else if(zoomFactor == 3){
            selected_top_label = top_labels_zoom3;
             selected_bottom_label = bottom_labels_zoom3;
        }else if(zoomFactor == 4){
            selected_top_label = top_labels_zoom4;
             selected_bottom_label = bottom_labels_zoom4;
        }else if(zoomFactor == 5){
            selected_top_label = top_labels_zoom5;
            selected_bottom_label = bottom_labels_zoom5;
        }
        else{
            selected_top_label = top_labels;
            selected_bottom_label = bottom_labels;
        }


        GrStringDrawCentered(&sContext, selected_top_label[selectedPlotType], -1,
                            15, 50, 0);
        GrStringDrawCentered(&sContext, selected_bottom_label[selectedPlotType], -1,
                            15, 195, 0);
        GrStringDrawCentered(&sContext, "0s", -1,
                            30, 210, 0);
        GrStringDrawCentered(&sContext, "5", -1,
                            130, 210, 0);
        GrStringDrawCentered(&sContext, "10s", -1,
                            230, 210, 0);

        start_new_plot = false;
    }


    uint32_t elapsed_time = 0;
    switch(selectedPlotType){

        case TEMPERATURE:
            elapsed_time = currentTime_TempHumSensor - start_time;
            break;
        case SPEED:
            elapsed_time = currentTime_Motor - start_time;
            break;
        case LIGHT:
            elapsed_time = currentTime_LightSensor - start_time;
            break;
        case HUMIDITY:
            elapsed_time = currentTime_TempHumSensor - start_time;
            break;
        case ACCELERATION:
            elapsed_time = currentTime_AccelerationSensor - start_time;
            break;
        case POWER:
            elapsed_time = currentTime_Motor - start_time;
            break;
        case NONE:
            elapsed_time = 0;
            break;
        default:
            elapsed_time = 0;
            break;
    }
    // Measure how many miliseconds has passed
    if(elapsed_time < TENSECONDS_TO_MS + 1 && elapsed_time >= 0){


        // Normalize graph based on the plot height
            uint16_t yPosition = GRAPH_HEIGHT + GRAPH_TOP_OFFSET;
            uint16_t xPosition= GRAPH_LEFT_MARGIN;
            float normalizedValue = 0.0f;
            switch(selectedPlotType){

                case TEMPERATURE:
                    // For sht31 sensor
                    normalizedValue = (filteredTemperature - SHT31_SENSOR_MIN) / (SHT31_SENSOR_MAX - SHT31_SENSOR_MIN);
                    yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                    xPosition = ((uint16_t)(elapsed_time / MS_PER_PIXEL)) + GRAPH_LEFT_MARGIN;
                    break;

                case SPEED:
                    normalizedValue = (filteredRPM - MOTOR_SENSOR_MIN_RPM) / (MOTOR_SENSOR_MAX_RPM - MOTOR_SENSOR_MIN_RPM);
                    yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                    xPosition = ((uint16_t)(elapsed_time / MS_PER_PIXEL)) + GRAPH_LEFT_MARGIN;
                    break;

                case LIGHT:
                    // For opt3001 sensor
                    normalizedValue = (filteredLux - OPT3001_SENSOR_MIN) / (OPT3001_SENSOR_MAX - OPT3001_SENSOR_MIN);
                    yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                    xPosition = ((uint16_t)(elapsed_time / MS_PER_PIXEL)) + GRAPH_LEFT_MARGIN;
                    break;
                case HUMIDITY:
                    // For Sht31 sensor
                    normalizedValue = (filteredHumidity - SHT31_SENSOR_MIN_HUMIDITY) / (SHT31_SENSOR_MAX_HUMIDITY - SHT31_SENSOR_MIN_HUMIDITY);
                    yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                    xPosition = ((uint16_t)(elapsed_time / MS_PER_PIXEL)) + GRAPH_LEFT_MARGIN;
                    break;
                case ACCELERATION:
                    // For bmi160 sensor
                    normalizedValue = (filteredAcceleration - BMI160_SENSOR_MIN) / (BMI160_SENSOR_MAX - BMI160_SENSOR_MIN);
                    yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                    xPosition = ((uint16_t)(elapsed_time / MS_PER_PIXEL)) + GRAPH_LEFT_MARGIN;
                    break;
                case POWER:
                    normalizedValue = (filteredPower - POWER_MIN_LIMIT) / (POWER_MAX_LIMIT - POWER_MIN_LIMIT);
                    yPosition = (GRAPH_HEIGHT - (normalizedValue * GRAPH_HEIGHT * zoomFactor)) + GRAPH_TOP_OFFSET;
                    xPosition = ((uint16_t)(elapsed_time / MS_PER_PIXEL)) + GRAPH_LEFT_MARGIN;                
                    break;
                case NONE:
                    yPosition = GRAPH_HEIGHT + GRAPH_TOP_OFFSET;
                    xPosition = GRAPH_LEFT_MARGIN;
                    break;
                default:
                    yPosition = GRAPH_HEIGHT + GRAPH_TOP_OFFSET;
                    xPosition = GRAPH_LEFT_MARGIN;
                    break;
        }

        // // Prevent overflow top
        if(yPosition < GRAPH_TOP_OFFSET){
            yPosition = GRAPH_TOP_OFFSET;
        }

        newY = yPosition;
        newX = xPosition;
        GrContextForegroundSet(&sContext, ClrYellow);
        GrLineDraw(&sContext, lastX, lastY, newX, newY);
        lastX = newX;
        lastY = newY;

    }else if(selectedPlotType != NONE){
        start_new_plot = true;
    }
}


//*****************************************************************************
//
// Handles notifications from the slider controls.
//
//*****************************************************************************




bool second_passed = false;
bool temp_changed = false;

uint32_t previous_seconds = 0;

void vDisplayDemoTask(void)
{
    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name Hello task - for debug only as it is
     *    not used by the kernel.
     *  - The size of the stack to allocate to the task.
     *  - No parameter passed to the task
     *  - The priority assigned to the task.
     *  - The task handle is NULL */
    xTaskCreate(prvDisplayDemoTask,
                "Hello",
                configMINIMAL_STACK_SIZE * 8,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}
/*-----------------------------------------------------------*/

static void prvDisplayDemoTask(void *pvParameters)
{
    BMI160_Data_t acceleration_receivedData = {0};
    SHT31_Data_t temperature_hum_recievedData = {0};
    OPT3001_Data_t light_receivedData = {0};
    Motor_Data_t motor_data = {0};

    // States for status screen boxes
    TempState_t temp_state = NORMAL;
    LightState_t light_state = Night;
    // in interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(configCPU_CLOCK_HZ);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);


    // Set up widget initalisation for sensors page, idk why but macro intialisation didnt work
    sensors_Labels[0].sBase.psNext = (tWidget *)&sensors_Labels[1];
    sensors_Labels[1].sBase.psNext = (tWidget *)&sensors_Labels[2];
    sensors_Labels[2].sBase.psNext = (tWidget *)&g_Sensors[0];

    g_Sensors[0].sBase.psNext = (tWidget *)&g_Sensors[1];
    g_Sensors[1].sBase.psNext = (tWidget *)&g_Sensors[2];
    g_Sensors[2].sBase.psNext = 0;

    //
    // Initialize the touch screen driver and have it route its messages to the
    // widget tree.
    //
    TouchScreenInit(configCPU_CLOCK_HZ);
    TouchScreenCallbackSet(WidgetPointerMessage);

    //
    // Add the title block and the previous and next buttons to the widget
    // tree.
    //

    // Navigation bar
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sStatus);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sSensors);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sPlotting);

    // 1 is for status page
    // 0 is for sensor controls
    // 2 is for plotting
    g_ui32Panel = 1;
    WidgetAdd(WIDGET_ROOT, (tWidget *)g_psPanels);
    //
    // Issue the initial paint request to the widgets.
    //
    WidgetPaint(WIDGET_ROOT);

    char lux_buffer[30] = "500 Lux";
    char time_buffer[30] = "Date: 14:15:00 29/5/2025";
    char temp_buffer[30] = "24 Celcius";
    char hum_buffer[30] = "60 %";
    char rpm_buffer[30] = "0 RPM";
    char acc_buffer[30] = "0 m/s^2";
    char pow_buffer[30] = "0 w";

    uint32_t baseHour = 14, baseMin = 15, baseSec = 0;

    for (;;)
    {
        WidgetMessageQueueProcess();

        if (estop_reset_hit)
        {
            UARTprintf("RESETTING\n");
            motorResetEStop();
            estop_reset_hit = false; // Reset the flag after handling
        }
        // Acceleration slider has been changed
        if(xSemaphoreTake(xBMI160ThresholdMutex, 0) == pdTRUE){
            acceleration_threshold = acceleration_threshold_display;
            // UARTprintf("%u\n", acceleration_threshold);
            xSemaphoreGive(xBMI160ThresholdMutex);
        }
        // Desired RPM slider has been changed
        if (desiredRPM_set)
        {
            UARTprintf("Desired RPM: %d\n", desiredRPM);
            // Here you can add code to handle the desired RPM change
            // For example, send it to a motor controller or update a display
            motorSetRPM(desiredRPM); 
            desiredRPM_set = false; // Reset the flag after handling
        }
        // Current Limit slider has been changed
        if(currentLimit_set)
        {
            UARTprintf("Current Limit: %d\n", currentLimit);
            // Here you can add code to handle the current limit change
            // For example, send it to a motor controller or update a display
            float currentLimit_float = (float)currentLimit / 1000;
            motorSetCurrentThreshold(currentLimit_float);
            currentLimit_set = false; // Reset the flag after handling
        }



        // Check if there is new sensor data to plot
        if(g_ui32Panel == STATUS_PAGE_NUMBER || g_ui32Panel == PLOTTING_PAGE_NUMBER){
            if(xAccelerationQueue != NULL){
                if (xQueueReceive(xAccelerationQueue, &acceleration_receivedData, 0) == pdPASS)
                {
                    // Acceleartion parsing into float to string
                    filteredAcceleration = acceleration_receivedData.filtered_acceleration;
                    currentTime_AccelerationSensor = acceleration_receivedData.timestamp;
                    int int_part_acc_fil = (int)acceleration_receivedData.filtered_acceleration;
                    int decimal_part_acc_fil = (int)((acceleration_receivedData.filtered_acceleration - int_part_acc_fil) * 100);
                    usprintf(acc_buffer, "%d.%02d m/s^2", int_part_acc_fil, decimal_part_acc_fil);
                }
            }
            if(xTemperatureHumidityQueue != NULL){
                if (xQueueReceive(xTemperatureHumidityQueue, &temperature_hum_recievedData, 0) == pdPASS)
                {
                    int int_part_temp_fil = (int)temperature_hum_recievedData.temp_filtered;
                    int decimal_part_temp_fil = (int)((temperature_hum_recievedData.temp_filtered - int_part_temp_fil) * 100);
                    usprintf(temp_buffer, "%d.%02d Celcius", int_part_temp_fil, decimal_part_temp_fil);

                    int int_part_hum_fil = (int)temperature_hum_recievedData.hum_filtered;
                    int decimal_part_hum_fil = (int)((temperature_hum_recievedData.hum_filtered - int_part_hum_fil) * 100);
                    usprintf(hum_buffer, "%d.%02d %", int_part_hum_fil, decimal_part_hum_fil);

                    currentTime_TempHumSensor = temperature_hum_recievedData.timestamp;
                    filteredTemperature = temperature_hum_recievedData.temp_filtered;
                    filteredHumidity = temperature_hum_recievedData.hum_filtered;

                    
                    if(filteredTemperature > (float)cooling_threshold){
                        temp_state = COOLING;
                    }else if(filteredTemperature < (float) heating_threshold){
                        temp_state = HEATING;
                    }else{
                        temp_state = NORMAL;
                    }
                }
            }
            if(xLightSensorQueue != NULL){
                if (xQueueReceive(xLightSensorQueue, &light_receivedData, 0) == pdPASS)
                {
                    int int_part_filtered = (int)light_receivedData.filtered_lux;
                    int decimal_part_filtered = (int)((light_receivedData.filtered_lux - int_part_filtered) * 100);
                    usprintf(lux_buffer, "%d.%02d Lux", int_part_filtered, decimal_part_filtered);
                    currentTime_LightSensor = light_receivedData.timestamp;
                    filteredLux = light_receivedData.filtered_lux;

                    if (filteredLux > (float)day_night_threshold)
                    {
                        light_state = Day;
                    }
                    else if (filteredLux < (float)day_night_threshold)
                    {
                        light_state = Night;
                    }
                    else
                    {
                        light_state = Night; // Default to Day if not in thresholds
                    }
                }
            }
            if (xMotorQueue != NULL)
            {
                if (xQueueReceive(xMotorQueue, &motor_data, 0) == pdPASS)
                {
                    // Motor speed parsing into float to string
                    int int_part_speed = (int)motor_data.filtered_rpm;
                    usprintf(rpm_buffer, "%d RPM", int_part_speed);
                    int int_part_power = (int)motor_data.filtered_power;
                    filteredRPM = (float)(int_part_speed);
                    filteredPower = (float)(int_part_power);
                    usprintf(pow_buffer, "%d w", int_part_power);
                    currentTime_Motor = motor_data.timestamp;
                    motor_state = motor_data.state;
                }
            }
        }
        // Status page
        if(g_ui32Panel == STATUS_PAGE_NUMBER){
            // Calculate time since scheduler was called
            TickType_t tickCount = xTaskGetTickCount();
            uint32_t totalSeconds = tickCount / configTICK_RATE_HZ;
            // Add to base time
            uint32_t seconds = (baseSec + totalSeconds) % 60;
            uint32_t minutes = (baseMin + ((baseSec + totalSeconds) / 60)) % 60;
            uint32_t hours = (baseHour + ((baseMin + ((baseSec + totalSeconds) / 60)) / 60)) % 24;
            // Date time update
            usprintf(time_buffer, "Date: %u:%u:%u 29/5/2025", hours, minutes, seconds);
            CanvasTextSet(&status_boxes_labels[2], time_buffer);
            WidgetPaint((tWidget *)&status_boxes_labels[2]);

            // For Light reading on Status page
            if (xLightSensorQueue != NULL) // Avoid unnessary paint
            {
                switch (light_state)
                {
                    case Day:
                        CanvasFillColorSet(&boxes[0], ClrBlue);
                        CanvasTextColorSet(&boxes[0], ClrWhite);
                        CanvasTextSet(&boxes[0], "Day");
                        break;
                    case Night:
                        CanvasFillColorSet(&boxes[0], ClrBlack);
                        CanvasTextColorSet(&boxes[0], ClrWhite);
                        CanvasTextSet(&boxes[0], "Night");
                        break;

                    default:
                        break;
                }
                CanvasTextSet(&status_boxes_labels[0], lux_buffer);
                WidgetPaint((tWidget *)&status_boxes_labels[0]);
                WidgetPaint((tWidget *)&boxes[0]);
            }
        
            // For Motor speed on status page
            if (xMotorQueue != NULL) // Avoid unnessary paint
            {
                switch(motor_state){
                    case MOTOR_STATE_STOPPED:
                        PushButtonFillColorSet(&estop_button[0], ClrRed);
                        PushButtonTextSet(&estop_button[0], "Stopped");
                        break;
                    case MOTOR_STATE_RUNNING:
                        PushButtonFillColorSet(&estop_button[0], ClrGreen);
                        PushButtonTextSet(&estop_button[0], "Running");      
                        break;              
                    case MOTOR_STATE_IDLING:
                        PushButtonFillColorSet(&estop_button[0], ClrBlack);
                        PushButtonTextColorSet(&estop_button[0], ClrWhite);
                        PushButtonTextSet(&estop_button[0], "Idiling");
                        break;
                    case MOTOR_STATE_ESTOP:
                        PushButtonFillColorSet(&estop_button[0], ClrOrange);
                        PushButtonTextSet(&estop_button[0], "ESTOP");
                        break;
                    default:
                        break;
                }
                CanvasTextSet(&status_boxes_labels[1], rpm_buffer);
                WidgetPaint((tWidget *)&status_boxes_labels[1]);
                WidgetPaint((tWidget *)&estop_button[0]);
            }
            // For Temp readin on status page
            if (xTemperatureHumidityQueue != NULL) // Avoid unnessary paint
            {
                switch(temp_state){
                    case COOLING:
                        CanvasFillColorSet(&boxes[1], ClrLightBlue);
                        CanvasTextSet(&boxes[1], "Cooling On");
                        break;
                    case HEATING:
                        CanvasFillColorSet(&boxes[1], ClrRed);
                        CanvasTextSet(&boxes[1], "Heating On");      
                        break;              
                    case NORMAL:
                        CanvasFillColorSet(&boxes[1], ClrLightGreen);
                        CanvasTextSet(&boxes[1], "Normal");
                        break;
                    default:
                        break;
                }
                CanvasTextSet(&status_boxes_labels[3], temp_buffer);
                WidgetPaint((tWidget *)&status_boxes_labels[3]);
                WidgetPaint((tWidget *)&boxes[1]);
            }
        }else if(g_ui32Panel == SENSOR_PAGE_NUMBER){
            // Sensor control page
        }else{
            // Plotting page
            // Calling widget paint will call the OnCanvasPaint function

            switch(selectedPlotType){
                case TEMPERATURE:
                    CanvasTextSet(&data_display[1], temp_buffer);
                    break;
                case SPEED:
                    CanvasTextSet(&data_display[1], rpm_buffer);
                    break;
                case LIGHT:
                    // For opt3001 sensor
                    CanvasTextSet(&data_display[1], lux_buffer);
                    break;
                case HUMIDITY:
                    CanvasTextSet(&data_display[1], hum_buffer);
                    break;
                case ACCELERATION:
                    // For bmi160 sensor
                    CanvasTextSet(&data_display[1], acc_buffer);
                    break;
                case POWER:
                    CanvasTextSet(&data_display[1], pow_buffer);
                    break;
                case NONE:
                    CanvasTextSet(&data_display[1], "");
                    break;
                default:
                    break;
            }
            WidgetPaint((tWidget *)&data_display[1]);
            WidgetPaint((tWidget *)&g_sPlottingCanvas);
        }
        // Refresh every 40ms
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}
/*-----------------------------------------------------------*/




// void xButtonsHandler(void)
// {
//     UARTprintf("Switch called\n");
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     uint32_t ui32Status;

//     // Read interrupt status
//     ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);
//     // Clear interrupt flags
//     GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

//     uint32_t now = xTaskGetTickCountFromISR();  // Use ISR-safe version

//     // Debounce: check if at least 50ms has passed since last press
//     if ((now - g_ui32TimeStampButton) > pdMS_TO_TICKS(50))
//     {
//         if ((ui32Status & USR_SW1) == USR_SW1)
//         {
//             g_pui32ButtonPressed = USR_SW1;
//             UARTprintf("Switch1\n");
//             // xSemaphoreGiveFromISR(xSW1Semaphore, &xHigherPriorityTaskWoken);
//         }
//         else if ((ui32Status & USR_SW2) == USR_SW2)
//         {
//             g_pui32ButtonPressed = USR_SW2;
//             UARTprintf("Switch2\n");
//             // xSemaphoreGiveFromISR(xSW2Semaphore, &xHigherPriorityTaskWoken);
//         }

//         g_ui32TimeStampButton = now;  // Update timestamp after debounce check
//     }

//     // Request a context switch if needed
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }

/*-----------------------------------------------------------*/
