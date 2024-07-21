/*/////////////////////////
debugging includes go here
*//////////////////////////
//#define __DEBUGMODE
///////////////////////////

#ifdef __DEBUGMODE

    #include "driverlib/uart.h"
    #include "utils/uartstdio.h"

    void InitConsole(void)
    {

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

        GPIOPinConfigure(GPIO_PA0_U0RX);

        GPIOPinConfigure(GPIO_PA1_U0TX);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

        UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        UARTStdioConfig(0, 115200, 16000000);
    }

#endif

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


double* jointAngles;
double _result[3];


void delayMs(uint32_t ui32Ms) {

    SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3 / 1000));
}

void ledBlink(){

    SysCtlDelay(SysCtlClockGet() / 10 );

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    SysCtlDelay(SysCtlClockGet() / 10 );

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}


int main(void)
{


    /*//////////////////////////////////////////////////////////////////
     * Initialization code
     *///////////////////////////////////////////////////////////////////

     // Enable lazy stacking for interrupt handlers. This allows floating-point
     // instructions to be used within interrupt handlers, but at the expense of
     // extra stack usage.
     MAP_FPULazyStackingEnable();

     // Set the clocking to run directly from the crystal.
     // Configure the system clock to 16 MHz.
     MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

     // Enable the GPIO port that is used for the on-board LED.
     MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

     // Wait for the GPIOF module to be ready.
     while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
     {
     }

     // Enable the GPIO pins for the LED (PF2).
     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Initialize PCA9685 Servo control module
    PCA9685Init(SLAVE_ADDRESS);

    // initializing leg modules
    struct leg leg1;    legInit(&leg1, 0, 1, 1, LEG1, SLAVE_ADDRESS);
    struct leg leg2;    legInit(&leg2, 0, 1, 1, LEG2, SLAVE_ADDRESS);
    struct leg leg3;    legInit(&leg3, 0, 1, 1, LEG3, SLAVE_ADDRESS);
    struct leg leg4;    legInit(&leg4, 0, 1, 1, LEG4, SLAVE_ADDRESS + 1);
    struct leg leg5;    legInit(&leg5, 0, 1, 1, LEG5, SLAVE_ADDRESS + 1);
    struct leg leg6;    legInit(&leg6, 0, 1, 1, LEG6, SLAVE_ADDRESS + 1);

    double position[3]  =   {2, 0, 0}; // extends al legs out
    double position2[3] =  {0.1736, 0, 0.0152};
    double position3[3] =  {1.4142, 0, -1.4142};

    while(1)
    {

        setAllPosition(position); // writes position to all legs

        updatePositon(); // moves servos to position

        ledBlink(); // blinks to indicate operation

        delayMs(1000);

        setAllPosition(position2); // writes position to all legs

        updatePositon(); // moves servos to position

        ledBlink(); // blinks to indicate operation

        setAllPosition(position3); // writes position to all legs

        updatePositon(); // moves servos to position

        ledBlink(); // blinks to indicate operation



    }
}
