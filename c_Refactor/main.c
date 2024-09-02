/*/////////////////////////
debugging includes go here
*//////////////////////////
//#define __DEBUGMODE
///////////////////////////



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
#include "walk.h"
//#include "leg.h"

#include "obstructionDetection.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"


volatile OBSTRUCTION OBSTRUCTION_STATE = IDLE;


//*****************************************************************************
//
// Millisecond delay
//
//*****************************************************************************
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // i2c protocol initialization
    i2cInit();

    // Initialize obstruction detection interrupt and procedure for automated operation
    obstructionDetectionInit();

    // Initialize PCA9685 Servo control module
    PCA9685Init(SLAVE_ADDRESS);

    // hexapod walk initialization
    walkInit();

    // master interrupt enable, needed for timer and i2c interrupts
    IntMasterEnable();

    updateTargets(0, 50, 0);

    while(1)
    {
       //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

       updateLegNoClock();

       //testCycle();

       //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

       if(OBSTRUCTION_STATE == OBSTRUCTION_DETECTED){

           obstructionAvoidance();
      }

    }
}





