#include "i2cOptDriver.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "FreeRTOS.h"
#include "semphr.h"

// #define BASE 0x40020000
#define BASE 0x40022000
extern volatile SemaphoreHandle_t xI2CSemaphore;
extern volatile SemaphoreHandle_t xI2C2Semaphore;
/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over i2c to specify register being read from
 * Reads three bytes from i2c slave. The third is redundant but
 * helps to flush the i2c register
 * Stores first two received bytes into *data
 */
bool bmi160ReadTwoBytes(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address and change I2C to write
    I2CMasterSlaveAddrSet(BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(BASE, ui8Reg);
    I2CMasterControl(BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }

    // Load device slave address and change I2C to read
    I2CMasterSlaveAddrSet(BASE, ui8Addr, true);

    // Read two bytes from I2C
    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[0] = I2CMasterDataGet(BASE);

    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[1] = I2CMasterDataGet(BASE);

    return true;
}


bool bmi160ReadSixBytes(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Step 1: Send register address to read from
    I2CMasterSlaveAddrSet(BASE, ui8Addr, false); // Write mode
    I2CMasterDataPut(BASE, ui8Reg);
    I2CMasterControl(BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }

    // Step 2: Set to read mode
    I2CMasterSlaveAddrSet(BASE, ui8Addr, true); // Read mode

    // Step 3: Read 6 bytes
    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[0] = I2CMasterDataGet(BASE);

    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[1] = I2CMasterDataGet(BASE);

    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[2] = I2CMasterDataGet(BASE);

    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[3] = I2CMasterDataGet(BASE);

    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[4] = I2CMasterDataGet(BASE);

    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    data[5] = I2CMasterDataGet(BASE);

    return true;
}


bool bmi160WriteOneByte(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t data)
{
    // UARTprintf("wprled\n");
    // Load device slave address
    I2CMasterSlaveAddrSet(BASE, ui8Addr, false);
    // Place the character to be sent in the data register
    I2CMasterDataPut(BASE, ui8Reg);


    //  UARTprintf("wprled2\n")
    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_SEND_START);
    // uint32_t err = I2CMasterErr(BASE);
    // if (err != I2C_MASTER_ERR_NONE) {
    //     UARTprintf("I2C Error: 0x%08x\n", err);
    //     return false;
    // }

    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    // Send Data
    //  UARTprintf("wprled3\n");
    I2CMasterDataPut(BASE, data);
    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // err = I2CMasterErr(BASE);
    // if (err != I2C_MASTER_ERR_NONE) {
    //     UARTprintf("I2C Error: 0x%08x\n", err);
    //     return false;
    // }
    //  UARTprintf("wprled51\n");
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
            //  UARTprintf("wprled5\n");
        return false; // Timeout occurred
    }
    return true;
    //   UARTprintf("wprled4\n");
    // UARTprintf("wprled\n");
}


bool bmiReadTwoBytes(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data) {
    // Set slave address for writing the register address
    I2CMasterSlaveAddrSet(BASE, ui8Addr, false); // Write mode
    I2CMasterDataPut(BASE, ui8Reg);             // Put register address
    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_SEND_START); // Start, send reg addr

    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE) {
        return false; // Timeout occurred during write
    }

    // Set slave address for reading the two bytes
    I2CMasterSlaveAddrSet(BASE, ui8Addr, true);  // Read mode

    // Initiate a burst read of two bytes
    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE){
        return false;
    }
    data[0] = I2CMasterDataGet(BASE);

    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
     if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE){
        return false;
    }
    data[1] = I2CMasterDataGet(BASE);
    
    I2CMasterControl(BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // Finish condition.
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE) {
        return false; // Timeout occurred during read
    }

    return true;
}


bool readI2COneByte(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Set slave address and write the register address
    I2CMasterSlaveAddrSet(BASE, ui8Addr, false); // Write mode
    I2CMasterDataPut(BASE, ui8Reg);
    I2CMasterControl(BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }

    // Set slave address again for reading
    I2CMasterSlaveAddrSet(BASE, ui8Addr, true); // Read mode
    I2CMasterControl(BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    if (xSemaphoreTake(xI2C2Semaphore, portMAX_DELAY) == pdFALSE)
    {
        return false; // Timeout occurred
    }
    // Read the data
    *data = I2CMasterDataGet(BASE);
    return true;
}
