#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "FreeRTOS.h"

typedef struct {
    float filtered_acceleration;
    TickType_t timestamp;
} BMI160_Data_t;


typedef struct {
    float hum_filtered;
    float temp_filtered;
    TickType_t timestamp;
} SHT31_Data_t;

typedef struct {
    float filtered_lux;
    TickType_t timestamp;
} OPT3001_Data_t;


typedef enum
{
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_IDLING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_ESTOP
} MotorState_t;

typedef struct {
    TickType_t timestamp;
    MotorState_t state;
    int filtered_rpm;
    float filtered_power;
} Motor_Data_t;

#endif