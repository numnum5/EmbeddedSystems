#include <stdbool.h>
#include <stdint.h>
bool readI2COneByte(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data);
bool bmi160ReadTwoBytes(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data);
bool bmi160WriteOneByte(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t data);
bool bmi160ReadSixBytes(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data);
bool bmiReadTwoBytes(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data);
