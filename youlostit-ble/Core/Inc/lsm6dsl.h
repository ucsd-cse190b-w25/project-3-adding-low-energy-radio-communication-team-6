
/*
 * lsm6dsl.h
 *
 *  Created on: Feb 5, 2025
 *      Author: Alexis Vega
 */

#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <stdint.h>
#include <stm32l475xx.h>

// LSM6DSL I2C Address
#define LSM6DSL_I2C_ADDR   0x6A
#define LSM6DSL_WHO_AM_I   0x0F

// Register Addresses
#define LSM6DSL_CTRL1_XL   0x10  // Accelerometer control register
#define LSM6DSL_OUTX_L_XL  0x28  // X-axis acceleration output register (LSB)
#define LSM6DSL_INT1_CTRL  0x0D  // Interrupt control register

// Bit Masks
#define LSM6DSL_AUTO_INC   0x80  // Enable auto-increment for register addresses

// Sensitivity for 2G mode (mg/LSB)
#define SENS2G 0.061

// Configuration values
#define CTRL1_XL_CONFIG  0x50  // Example: 104 Hz, 2g, anti-aliasing filter
#define INT1_CTRL_CONFIG  0x01  // Example: Enable accelerometer data ready interrupt on INT1

// Function Prototypes
void lsm6dsl_init();
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);

#endif // LSM6DSL_H
