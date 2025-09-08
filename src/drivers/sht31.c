#include "i2cSht31Driver.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Converts raw temperature and humidity data from the SHT31 sensor
 *        into human-readable float values in Celsius and %RH.
 * 
 * @param rawData Pointer to raw sensor data array (6 bytes expected).
 * @param convertedTemp Pointer to float where temperature result will be stored.
 * @param convertedHumidity Pointer to float where humidity result will be stored.
 */
void sensorSHT31Convert(uint8_t * rawData, float * convertedTemp, float * convertedHumidity)
{
    // Combine high and low bytes of temperature and humidity readings
    uint16_t temp = (rawData[0] << 8) | rawData[1];
    uint16_t humidity = (rawData[3] << 8) | rawData[4];

    // Convert raw data to Celsius and relative humidity using datasheet formula
    float temperatureCelsius = -45.0 + (175.0 * (float)temp / 65535);
    float humidityPercent = 100.0 * (float)humidity / 65535;
    
    // Store results in output pointers
    *convertedTemp = temperatureCelsius;
    *convertedHumidity = humidityPercent;
}




/**
 * @brief Reads the status register of the SHT31 sensor.
 * 
 * @param data Pointer to a 3-byte array to store status register (MSB, LSB, CRC).
 * @return true on success, false on failure.
 */
bool readStatusSHT31(uint8_t * data){
    if(!readStatusRegSHT31(SHT31_I2C_ADDRRESS, data)){
        return false;
    }
    return true;
}


/**
 * @brief Reads the temperature and humidity data from the sensor into a data buffer.
 * 
 * @param data Pointer to a 6-byte array where raw data will be stored.
 * @return true on success.
 */
bool readSensorSHT31DataTemp(uint8_t * data){
    readSHT31Data(SHT31_I2C_ADDRRESS, data);
    return true;
}

/**
 * @brief Sends a command to break (stop) periodic acquisition mode.
 */
void sensorSHT31Break(void){
    uint16_t periodic_break = SHT31_PERIODIC_BREAK;
    sendSHT31Command(SHT31_I2C_ADDRRESS, (uint8_t *)&periodic_break);
}

/**
 * @brief Sends a soft reset command to the SHT31 sensor.
 */
void sensorSHT31Reset(void){
    uint16_t reset = SHT31_RESET;
    sendSHT31Command(SHT31_I2C_ADDRRESS, (uint8_t *)&reset);
}

/**
 * @brief Sets the measurement frequency to medium repeatability at 1Hz sampling rate.
 */
void sensorSHT31SetFrequency(void){
    uint16_t frequency = REP_MED_FREQUENCY_1;
    sendSHT31Command(SHT31_I2C_ADDRRESS, (uint8_t *)&frequency);
}





uint8_t calculate_sht31_crc(uint8_t * data, int length) {
    uint8_t crc = 0xFF; // Initialization value
    uint8_t polynomial = 0x31; // CRC polynomial

    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) { // Check if MSB is set
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc; // The calculated CRC value
}