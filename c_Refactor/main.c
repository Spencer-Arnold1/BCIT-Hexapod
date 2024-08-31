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

#define DELAY_1MS 16000// CPU Cycles required for 1ms
#define WAIT_TIME 300  //time to wait in ms

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//#include "TM4C123.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "leg.h"
#include "walk.h"


//double  angles[3];
double _result[3];

int ledState = 0;
uint32_t startTime = 0;

//uint32_t mySystemClock;

//mySystemClock = SysCtlClockGet();



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

/*void timerSetup()
{
    //setup timer
    NVIC_ST_CTRL_R = 0;                //disable SysTick clock
    NVIC_ST_RELOAD_R = DELAY_1MS - 1;    //set interrupt time
    NVIC_ST_CURRENT_R = 0;             // reset register
    NVIC_ST_CTRL_R = 0x07;             // re-enable clocks

}*/

volatile uint32_t g_ui32Counter = 0;
int clockDelay = 1000;


void SysTickIntHandler(void)
{
    //
    // Update the Systick interrupt counter.
    //
    g_ui32Counter++;
}

void Timer1A_Handler(void) {
    TIMER1_ICR_R = TIMER_ICR_TATOCINT; // Clear the timer interrupt flag
    g_ui32Counter++;


}



int main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_8|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);

    IntMasterEnable();
    //timer test 2
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;  //enable clock Timer1 subtimer A in run mode

    while((SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R1) == 0) {} // Wait until Timer 0 is ready

    TIMER1_CTL_R = 0; // disable timer1 output
    TIMER1_CFG_R = 0x4; //select 16-bit configuration option
    TIMER1_TAMR_R = 0x02; //select periodic down counter mode of timer1
    TIMER1_TAILR_R = 16000 - 1; //sets timer 1 for 1ms interval

    TIMER1_ICR_R = TIMER_ICR_TATOCINT; // Clear timeout flag

    //TIMER1_CTL_R |= (1<<0);  //Enable TimerA module

    TIMER1_IMR_R |= TIMER_IMR_TATOIM;  // Enable timeout interrupt
    NVIC_EN1_R |= 1 << 21;             // Enable Timer 1A interrupt in NVIC

    TIMER1_CTL_R |= TIMER_CTL_TAEN; // Enable Timer 1A

    //__asm("CPSIE I"); // Enable interrupts



/*
    //timerSetup();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
    SysTickPeriodSet(25000000);
    IntMasterEnable();
    SysTickIntEnable();
    SysTickIntEnable();
*/
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
        struct leg leg1;    legInit(&leg1, 32, 124, 221, LEG1, SLAVE_ADDRESS);
        struct leg leg2;    legInit(&leg2, 32, 124, 221, LEG2, SLAVE_ADDRESS);
        struct leg leg3;    legInit(&leg3, 32, 124, 221, LEG3, SLAVE_ADDRESS);
        struct leg leg4;    legInit(&leg4, 32, 124, 221, LEG4, SLAVE_ADDRESS + 1);
        struct leg leg5;    legInit(&leg5, 32, 124, 221, LEG5, SLAVE_ADDRESS + 1);
        struct leg leg6;    legInit(&leg6, 32, 124, 221, LEG6, SLAVE_ADDRESS + 1);

        //double position1[3]  =   {376, 0, 0}; // extends all legs out
        //double position2[3] =  {236, 236, 0};
        double position1[3] =  {156, 0, -221};
        //double position3[3]  =   {150, 0, -100}; // extends all legs out
        //double position4[3] =  {236, -236, 0};
        //double position4[3] =  {156, 0, -221};



        //unsigned long previousMils = 0;
        //const long interval = 30;
        //int xSpeed = 0;
        //int ySpeed = 50;
        //int rotationSpeed = 0;


    int systemClockVal = 0;
    uint32_t currentTime = g_ui32Counter;
    setAllPosition(position1);
    updatePositon();
    while(1)
    {

        //unsigned long currentMillis = timerVal;
        //if (currentMillis - previousMils >= interval) {
            //previousMils = currentMillis;

            //updateTargets(xSpeed,ySpeed,rotationSpeed);
            //updateLegNoClock();

        //}
        //delayMs();
        testCycle();
        /*
        uint32_t currentTime = g_ui32Counter;
        systemClockVal = SysCtlClockGet();

        if ((currentTime - startTime) > clockDelay)
        {
            if (ledState)
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            else
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            ledState = !ledState;
        }
        */










        /*
        testMove(1,position1);
        updatePositon();
        delayMs(500);

        testMove(1,position2);
        updatePositon();
        delayMs(500);
        */



       /* GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        setAllPosition(position1);

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

        updatePositon(); // moves servos to position

        delayMs(300);


        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        setAllPosition(position2);

        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

        updatePositon();

        delayMs(300);

        setAllPosition(position1);
        updatePositon();
        delayMs(300);

        setAllPosition(position4);
        updatePositon();
        delayMs(1000);*/

    }
}
