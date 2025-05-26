/*
 * acc_ADXL345.c
 *
 *  Created on: May 26, 2025
 *      Author: Mati
 */

#include "acc_ADXL345.h"

void ADXL345_Init(void){
    uint8_t id;
    ADXL345_Read(ADXL345_DEVID_REG, &id, 1);
    if (id != 0x8)
    {
        // Handle error
        while (1);
    }

    ADXL345_Write(ADXL345_POWER_CTL, 0x08);      // Set measure bit
    ADXL345_Write(ADXL345_DATA_FORMAT, 0x00);    // Set range to ±2g
}

void ADXL345_ReadAccel(float* x, float* y, float* z){
    uint8_t buffer[6];
    int16_t rawX, rawY, rawZ;
    float sensitivity = 0.004f; // 4 mg/LSB

    ADXL345_Read(ADXL345_DATAX0, buffer, 6);

    rawX = (int16_t)(buffer[1] << 8 | buffer[0]);
    rawY = (int16_t)(buffer[3] << 8 | buffer[2]);
    rawZ = (int16_t)(buffer[5] << 8 | buffer[4]);

    *x = rawX * sensitivity;
    *y = rawY * sensitivity;
    *z = rawZ * sensitivity;
}

void ADXL345_Write(uint8_t reg, uint8_t data){
    HAL_I2C_Mem_Write(&hi2c3, ADXL345_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
}

void ADXL345_Read(uint8_t reg, uint8_t* buffer, uint8_t length){
    HAL_I2C_Mem_Read(&hi2c3, ADXL345_ADDR, reg, 1, buffer, length, HAL_MAX_DELAY);
}
