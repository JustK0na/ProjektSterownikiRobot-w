/*
 * acc_ADXL345.h
 *
 *  Created on: May 26, 2025
 *      Author: Mati
 */

#ifndef SRC_ACC_ADXL345_H_
#define SRC_ACC_ADXL345_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "i2c.h"

#define ADXL345_ADDR        (0x53 << 1) // 7-bit address shifted for HAL
#define ADXL345_DEVID_REG   0x00
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32

/**
 * @brief Accelerometer initialization
 */
void ADXL345_Init(void);
/**
 * @brief Read and convert accelerometer data
 * @param x accelerometer's reading on X axis [g]
 * @param y accelerometer's reading on Y axis [g]
 * @param z accelerometer's reading on Z axis [g]
 */
void ADXL345_ReadAccel(float* x, float* y, float* z);
/**
 * @brief Write to accelerometer register
 * @param reg register's index
 * @param data data to be written into the register
 */
void ADXL345_Write(uint8_t reg, uint8_t data);
/**
 * @brief Read from accelerometer's given register
 * @param reg register's index
 * @param buffer where the multiple-bytes data is stored
 * @param length lenght of the buffer
 */
void ADXL345_Read(uint8_t reg, uint8_t* buffer, uint8_t length);

#endif /* SRC_ACC_ADXL345_H_ */
