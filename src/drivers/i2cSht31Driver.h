#include <stdbool.h>
#include <stdint.h>
#define SHT31_I2C_ADDRRESS 0x44
#define SHT31_FETCH_DATA 0xE000
#define SHT31_RESET 0x30A2
#define SHT31_PERIODIC_BREAK 0x3093
#define SHT31_READ_STATUS_REG 0xF32D
#define REP_LOW_FREQUENCY_0_5 0x202F
#define REP_MED_FREQUENCY_0_5 0x2024
#define REP_HIGH_FREQUENCY_0_5 0x2032
#define REP_LOW_FREQUENCY_1 0x212D
#define REP_MED_FREQUENCY_1 0x2126
#define REP_HIGH_FREQUENCY_1 0x2130
#define REP_LOW_FREQUENCY_2 0x222B
#define REP_MED_FREQUENCY_2 0x2220
#define REP_HIGH_FREQUENCY_2 0x2236
#define REP_LOW_FREQUENCY_4 0x2329
#define REP_MED_FREQUENCY_4 0x2322
#define REP_HIGH_FREQUENCY_4 0x2334
#define REP_LOW_FREQUENCY_10 0x272A
#define REP_MED_FREQUENCY_10 0x2721
#define REP_HIGH_FREQUENCY_10 0x2737


bool sendSHT31Command(uint8_t address, uint8_t * command);
bool readStatusRegSHT31(uint8_t ui8Addr, uint8_t * status);
bool readSHT31Data(uint8_t ui8Addr, uint8_t *data);