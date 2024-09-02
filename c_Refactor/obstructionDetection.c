/*
 * obstructionDetection.c
 *
 *  Created on: Aug. 27, 2024
 *      Author: Hassa
 */




#include "obstructionDetection.h"


void obstructionHandler(void)
{

    // Clear the interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_2);

    OBSTRUCTION_STATE = OBSTRUCTION_DETECTED;

    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_STOP);

}

void obstructionAvoidance(void){

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false); // i2c speed reduced to 100kb/s

    uint32_t obstructionDATA;

    SysCtlDelay(1 * (SysCtlClockGet() / 3 / 1000)); // 1ms debounce

    // Set slave address and specify master receive mode
    I2CMasterSlaveAddrSet(I2C0_BASE, ARDUNIO_ADDRESS, true);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait for the transaction to complete
    while (I2CMasterBusy(I2C0_BASE)) {
        // Optionally, implement a timeout here
    }

    // Get received data
    obstructionDATA = I2CMasterDataGet(I2C0_BASE);

    OBSTRUCTION_STATE = IDLE;

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); // i2c speed increased to 400 kb/s

    if(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)){

        SysCtlDelay(10000000);

        int i = 0;

        for( i = 0; i < 5; i++) {

            updateTargets(0, -50, 0);

            updateLegNoClock();

        }
    }


    updateTargets(0, 50, 0);
}


void obstructionDetectionInit() {

    // Set PF2 as input with a pull-up resistor
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Pull-up resistor strength
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Configure PF2 to trigger an interrupt on a falling edge
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);

    // Enable the GPIOF interrupt for PF2
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Interrupt enable
    IntEnable(INT_GPIOF);
    IntRegister(INT_GPIOF, obstructionHandler);

}


