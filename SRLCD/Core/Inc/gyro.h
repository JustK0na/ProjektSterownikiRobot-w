/*
 * gyro.h
 *
 *  Created on: May 26, 2025
 *      Author: Matylda Odrowąż-Wilkońska
 */

#ifndef INC_GYRO_H_
#define INC_GYRO_H_

#include <stdint.h>
#include "main.h"
#include "stm32f4xx_hal.h"

#define GYRO_CS_PIN       GPIO_PIN_1
#define GYRO_CS_PORT      GPIOC
#define NOISE_THRESHOLD   2.4

/**
 * @brief Gyroscope initialization
 */
void Gyro_Init(void);
/**
 * @brief Read and convert gyroscope data
 * @param x gyroscope's reading on X axis [g]
 * @param y gyroscope's reading on Y axis [g]
 * @param z gyroscope's reading on Z axis [g]
 */
void Gyro_ReadData(float* x, float* y, float* z);
/**
 * @brief Write to gyroscope register
 * @param reg register's index
 * @param data data to be written into the register
 */
void Gyro_WriteReg(uint8_t reg, uint8_t data);
/**
 * @brief Read from gyroscope register
 * @param reg register's index
 */
uint8_t Gyro_ReadReg(uint8_t reg);
/**
 * @brief Read multiple bytes from gyroscope
 * @param reg register's index
 * @param buffer where the multiple-bytes data is stored
 * @param length lenght of the buffer
 */
void Gyro_ReadRegs(uint8_t reg, uint8_t* buffer, uint8_t length);

#endif /* INC_GYRO_H_ */
