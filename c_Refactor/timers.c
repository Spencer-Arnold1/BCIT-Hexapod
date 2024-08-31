/*
 * timers.c
 *
 *  Created on: Aug. 27, 2024
 *      Author: Slaptop
 */

const int freq = 25000000;
unsigned long long int timerVal = 0;

void timerSetup()
{
    //setup timer
    /*NVIC_ST_CTRL_R = 0;                //disable SysTick clock
    NVIC_ST_RELOAD_R = DELAY_1MS - 1;    //set interrupt time
    NVIC_ST_CURRENT_R = 0;             // reset register
    NVIC_ST_CTRL_R = 0x07;             // re-enable clocks
    */

    //enable timer and set interrupt

    //SysTick->CTRL = 0x03;

    //set systick time clockfreq*DELAY
    // 25Mhz * 1s = 25,000,000 cycles
    int clockSpeed = 25000000;
    int delay = 1;
    int ticks = clockSpeed * delay;
    //SysTick->LOAD = ticks - 1; //set timer to count down from ticks


}

/*void SysTick_Handler(void)
{
    timerVal++;
}*/

