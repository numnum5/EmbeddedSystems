
#include <stdbool.h>
#include <stdint.h>
void sensorBmi160Reset(void);
bool sensorBmi160Test(void);
void sensorBmi160SetMode(void);
bool sensorBmi160Init(void);
bool sensorBmi160ReadAcceleration(uint8_t * data);
bool sensorBmi160ReadAccelerationX(uint8_t * data);
bool sensorBmi160ReadAccelerationY(uint8_t * data);
bool sensorBmi160ReadAccelerationZ(uint8_t * data);
bool sensorBmi160SetHighGThreshold(float threshold);
float convertRawData(float rawData);
bool sensorBmi160ReadTest(uint8_t * data);
void sensorBmi160SetFOC(void);