#include <stdint.h>
#include <stdbool.h>

bool readStatus(uint8_t * data);
bool readSensorSHT31DataTemp(uint8_t * data);
void sensorSHT31Break(void);
void sensorSHT31Reset(void);
void sensorSHT31SetFrequency(void);
void sensorSHT31Convert(uint8_t * rawData, float * convertedTemp, float * convertedHumidity);