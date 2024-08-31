/*/////////////////////////
debugging includes go here
*//////////////////////////
//#define __DEBUGMODE
///////////////////////////

#define LEG1 0x06
#define LEG2 0x12
#define LEG3 0x1E

#define LEG4 0x06
#define LEG5 0x12
#define LEG6 0x1E

#define SLAVE_ADDRESS 0x40


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "leg.h"
#include "obstructionDetection.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"


volatile OBSTRUCTION OBSTRUCTION_STATE = IDLE;

void delayMs(uint32_t ui32Ms) {

    SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3 / 1000));
}


//*****************************************************************************
//
// Configure the I2C0 master
//
//*****************************************************************************
void i2cInit(){


    // enable I2C0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);


    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);  ////// Changed from false to true on 2024-08-12 //replaced SysCtlClockGet with 80 Mhz

}



int main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_8|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);


    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //init_arctan_lut();
    //init_trig_lookup();

    i2cInit();

    obstructionDetectionInit();

    // Initialize PCA9685 Servo control module
    PCA9685Init(SLAVE_ADDRESS);

    IntMasterEnable();

    // initializing leg modules


    struct leg leg1;    legInit(&leg1, 0, 1, 1, LEG1, SLAVE_ADDRESS);
    struct leg leg2;    legInit(&leg2, 0, 1, 1, LEG2, SLAVE_ADDRESS);
    struct leg leg3;    legInit(&leg3, 0, 1, 1, LEG3, SLAVE_ADDRESS);
    struct leg leg4;    legInit(&leg4, 0, 1, 1, LEG4, SLAVE_ADDRESS + 1);
    struct leg leg5;    legInit(&leg5, 0, 1, 1, LEG5, SLAVE_ADDRESS + 1);
    struct leg leg6;    legInit(&leg6, 0, 1, 1, LEG6, SLAVE_ADDRESS + 1);


    double position3[3] =  {1.39, 0, -0.4922};
    double position[3]  =   {2, 0, 0}; // extends al legs out

    while(1)
    {

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        setAllPosition(position);

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

        delayMs(300);

        updatePositon();

        setAllPosition(position3);

        delayMs(300);

        updatePositon();



        if(OBSTRUCTION_STATE == OBSTRUCTION_DETECTED){

            obstructionAvoidance();
        }

    }
}
