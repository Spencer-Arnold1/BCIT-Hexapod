/*
 * obstructionDetection.h
 *
 *  Created on: Aug. 27, 2024
 *      Author: Hassa
 */


#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"




typedef enum {

  OBSTRUCTION_DETECTED,
  IDLE,

} OBSTRUCTION;

extern volatile OBSTRUCTION OBSTRUCTION_STATE;


#define ARDUNIO_ADDRESS 0xFE


static int SYSTEM_STATE;

void obstructionDetectionInit();

void obstructionHandler();

void obstructionAvoidance(void);





