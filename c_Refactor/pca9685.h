/*
 * pca9685.h
 *
 *  Created on: Jun. 8, 2024
 *      Author: Hassan
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"



// Public Functions
void servoControl(double angle, uint32_t baseAddress, uint32_t driverAddress);

void PCA9685Init(uint32_t addr);

// Private Functions
void pca9685sendDataFrame(uint8_t slaveAddr, uint8_t controlReg, uint8_t data);


int systemTransferEquation(double angle);


