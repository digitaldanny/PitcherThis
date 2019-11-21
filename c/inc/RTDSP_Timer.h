/*
 * RTDSP_Timer.h
 *
 *  Created on: Oct 8, 2019
 *      Author: dhami
 */

#ifndef SRC_RTDSP_TIMER_H_
#define SRC_RTDSP_TIMER_H_

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

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
void timer1Init(float period);

//
// initCPUTimers - This function initializes all three CPU timers
// to a known state.
//
void initCPUTimers(void);

//
// configCPUTimer - This function initializes the selected timer to the
// period specified by the "freq" and "period" parameters. The "freq" is
// entered as Hz and the period in uSeconds. The timer is held in the stopped
// state after configuration.
//
void configCPUTimer(uint32_t cpuTimer, float freq, float period);

#endif /* SRC_RTDSP_TIMER_H_ */
