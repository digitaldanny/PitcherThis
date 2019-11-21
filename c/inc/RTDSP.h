/*
 * AUTHOR: Daniel Hamilton
 *
 * SUMMARY: RTDSP.h
 * This file includes all drivers/functions written for the Real-Time
 * DSP Applications course.
 */

#ifndef _RTDSP
#define _RTDSP

#include "RTDSP_SramSpi.h"
#include "RTDSP_CodecGpio.h"
#include "RTDSP_Timer.h"
#include "RTDSP_ADC.h"
#include "RTDSP_Sampling.h"
#include "OneToOneI2CDriver.h"
#include "InitAIC23.h"
#include "AIC23.h"

// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
// TIMER
// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
#define SAMPLING_FREQUENCY_48       46875.0f
#define SAMPLING_FREQUENCY_32       32000.0f
#define SAMPLING_FREQUENCY_8        8000.0f

#define TIMER_PERIOD_48             (float)(1000000.0f/(2.0f*SAMPLING_FREQUENCY_48)) // in uSeconds
#define TIMER_PERIOD_32             (float)(1000000.0f/(2.0f*SAMPLING_FREQUENCY_32)) // in uSeconds
#define TIMER_PERIOD_8              (float)(1000000.0f/(2.0f*SAMPLING_FREQUENCY_8))  // in uSeconds

// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
// ADC
// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
#define VREF                        3.0f

// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
// CODEC BUTTONS / SWITCHES / LEDS
// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
#define LEFT_BUTTON                 0x4
#define MIDDLE_BUTTON               0x2
#define RIGHT_BUTTON                0x1

#define SW0                         0x1
#define SW1                         0x2
#define SW2                         0x4

#define LED0                        0x01
#define LED1                        0x02
#define LED2                        0x04
#define LED3                        0x08
#define LED4                        0x10
#define LED5                        0x20
#define LED6                        0x40
#define LED7                        0x80

// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
// MISC
// +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
#define INT_TO_ASCII(VAL)           VAL + 0x30

#endif
