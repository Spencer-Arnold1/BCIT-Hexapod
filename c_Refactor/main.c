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

    //
    // Delay for a bit
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

    //
    // Delay for a bit
    //
    SysCtlDelay(2000000/5);


    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
}


int main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_8|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);

    //SysCtlClockSet(SYSCTL_SYSDIV_6 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_OSC_MAIN);

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
    init_trig_lookup();


    // Initialize PCA9685 Servo control module
    PCA9685Init(SLAVE_ADDRESS);


    // initializing leg modules

    struct leg leg1;    legInit(&leg1, 0, 1, 1, LEG1, SLAVE_ADDRESS);
    struct leg leg2;    legInit(&leg2, 0, 1, 1, LEG2, SLAVE_ADDRESS);
    struct leg leg3;    legInit(&leg3, 0, 1, 1, LEG3, SLAVE_ADDRESS);
    struct leg leg4;    legInit(&leg4, 0, 1, 1, LEG4, SLAVE_ADDRESS + 1);
    struct leg leg5;    legInit(&leg5, 0, 1, 1, LEG5, SLAVE_ADDRESS + 1);
    struct leg leg6;    legInit(&leg6, 0, 1, 1, LEG6, SLAVE_ADDRESS + 1);


    double position3[3] =  {0, 0, 2};
    double position[3]  =   {2, 0, 0}; // extends al legs out



    while(1)
    {

                //ledBlink();

                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

                setAllPosition(position);

                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

                updatePositon(); // moves servos to position

                delayMs(1000);


                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

                //ledBlink(); // blinks to indicate operation

                setAllPosition(position3);


                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

                updatePositon();

                delayMs(1000);


                 // moves servos to position

    }
}
