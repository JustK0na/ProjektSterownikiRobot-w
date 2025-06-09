/*
 * gyro.c
 *
 *  Created on: May 26, 2025
 *      Author: Mati
 */

#include <stdio.h>
#include "gyro.h"
#include "spi.h"

// Gyroscope initialization
void Gyro_Init(void){
    // CTRL_REG1: 0x0F = Normal mode, all axes enabled, 95 Hz ODR
    Gyro_WriteReg(0x20, 0x0F);
    // CTRL_REG4: 0x20 = 2000 dps full scale
    Gyro_WriteReg(0x23, 0x20);
}

// Write to gyroscope register
void Gyro_WriteReg(uint8_t reg, uint8_t data){
    uint8_t tx[2] = {reg, data};
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi5, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

// Read from gyroscope register
uint8_t Gyro_ReadReg(uint8_t reg){
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi5, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi5, &rx, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return rx;
}

// Read multiple bytes from gyroscope
void Gyro_ReadRegs(uint8_t reg, uint8_t* buffer, uint8_t length){
    uint8_t tx = reg | 0xC0; // Set auto-increment and read bits
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi5, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi5, buffer, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

// Read and convert gyroscope data
void Gyro_ReadData(float* x, float* y, float* z){
    uint8_t buffer[6];
    int16_t rawX, rawY, rawZ;
    float sensitivity = 70.0f; // Sensitivity for 2000 dps full scale

    Gyro_ReadRegs(0x28, buffer, 6);

    rawX = (int16_t)(buffer[1] << 8 | buffer[0]);
    rawY = (int16_t)(buffer[3] << 8 | buffer[2]);
    rawZ = (int16_t)(buffer[5] << 8 | buffer[4]);

    // LSB * mg/LSB / 1000 = g
    *x = rawX * sensitivity / 1000.0f;
    *y = rawY * sensitivity / 1000.0f;
    *z = rawZ * sensitivity / 1000.0f;
}
