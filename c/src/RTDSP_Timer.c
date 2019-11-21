/*
 * RTDSP_Timer.h
 *
 *  Created on: Oct 8, 2019
 *      Author: dhami
 */

#include "RTDSP_Timer.h"

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: timer1Init
 * This function initializes timer1 to interrupt at 10Hz
 *
 * NOTE:
 * Assumption is that an ISR has been configured using the
 * following function call:
 *
 * Interrupt_register(INT_TIMER1, &cpuTimer1ISR);
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void timer1Init(float period)
{
    // points the timer interrupt to the ISR.
    // Interrupt_register(INT_TIMER1, &cpuTimer1ISR);

    // initializes the CPU timer's device peripheral.
    initCPUTimers();

    // configure CPU timer to interrupt at 10 Hz (0.1 s)
    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

    // enable interrupt for the timer
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_enable(INT_TIMER1);

    // start CPU timer 1
    CPUTimer_startTimer(CPUTIMER1_BASE);

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;
}

//
// initCPUTimers - This function initializes all three CPU timers
// to a known state.
//
void initCPUTimers(void)
{
    //
    // Initialize timer period to maximum
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    CPUTimer_setPeriod(CPUTIMER2_BASE, 0xFFFFFFFF);

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);

    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_stopTimer(CPUTIMER2_BASE);

    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);
}

//
// configCPUTimer - This function initializes the selected timer to the
// period specified by the "freq" and "period" parameters. The "freq" is
// entered as Hz and the period in uSeconds. The timer is held in the stopped
// state after configuration.
//
void configCPUTimer(uint32_t cpuTimer, float freq, float period)
{
    uint32_t temp;

    //
    // Initialize timer period:
    //
    temp = (uint32_t)(freq / 1000000 * period);
    CPUTimer_setPeriod(cpuTimer, temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(cpuTimer, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(cpuTimer);
}
