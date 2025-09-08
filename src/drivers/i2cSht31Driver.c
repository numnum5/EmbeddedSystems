#include "i2cSht31Driver.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern volatile SemaphoreHandle_t xI2C2Semaphore;
/**
 * Sends a 16-bit command to the SHT31 sensor over I2C.
 *
 * @param address: I2C address of the sensor.
 * @param command: Pointer to 2-byte command to send (must be little-endian).
 * @return true on success.
 */
bool sendSHT31Command(uint8_t address, uint8_t * command) {
    I2CMasterSlaveAddrSet(I2C2_BASE, address, false); // Set slave address for write
    I2CMasterDataPut(I2C2_BASE, command[1]);          // Send MSB first
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }         // Wait for transmission

    I2CMasterDataPut(I2C2_BASE, command[0]);          // Send LSB
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }          // Wait until bus is free

    return true;
}

/**
 * Reads the SHT31 status register (2 bytes + CRC).
 *
 * @param ui8Addr: I2C address of the sensor.
 * @param status: Buffer to store 3 bytes (status_MSB, status_LSB, CRC).
 * @return true on success, false if command fails.
 */
bool readStatusRegSHT31(uint8_t ui8Addr, uint8_t * status) {
    uint16_t status_read = SHT31_READ_STATUS_REG;

    // Send command to read status register
    if (!sendSHT31Command(SHT31_I2C_ADDRRESS, (uint8_t *)&status_read)) {
        return false;
    }

    // Set up I2C to read 3 bytes from sensor
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    status[0] = I2CMasterDataGet(I2C2_BASE); // MSB

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    status[1] = I2CMasterDataGet(I2C2_BASE); // LSB

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    status[2] = I2CMasterDataGet(I2C2_BASE); // CRC

    return true;
}

/**
 * Reads temperature and humidity data from SHT31 (6 bytes: temp[2]+CRC, humidity[2]+CRC).
 *
 * @param ui8Addr: I2C address of the sensor.
 * @param data: Buffer to store 6 bytes of output (T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC).
 * @return true on success, false on failure.
 */
bool readSHT31Data(uint8_t ui8Addr, uint8_t *data) {
    uint16_t status_read = SHT31_FETCH_DATA;

    // Send command to fetch latest data
    if (!sendSHT31Command(SHT31_I2C_ADDRRESS, (uint8_t *)&status_read)) {
        return false;
    }

    // Switch to read mode
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

    // Begin burst read sequence (6 bytes total)
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[0] = I2CMasterDataGet(I2C2_BASE); // Temp MSB

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[1] = I2CMasterDataGet(I2C2_BASE); // Temp LSB

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[2] = I2CMasterDataGet(I2C2_BASE); // Temp CRC

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[3] = I2CMasterDataGet(I2C2_BASE); // Humidity MSB

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[4] = I2CMasterDataGet(I2C2_BASE); // Humidity LSB

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[5] = I2CMasterDataGet(I2C2_BASE); // Humidity CRC

    return true;
}
