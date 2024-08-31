/*
 * timers.h
 *
 *  Created on: Aug. 27, 2024
 *      Author: Slaptop
 */

#ifndef TIMERS_H_
#define TIMERS_H_


#include "inc/tm4c123gh6pm.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

#define DELAY_1MS 16000// CPU Cycles required for 1ms
//#define WAIT_TIME 300  //time to wait in ms

extern unsigned long long int timerVal;

void timerSetup();
//void SysTick_Handler(void);


#endif /* TIMERS_H_ */
