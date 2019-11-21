/*
 * RTDSP_Gpio.h
 *
 *  Created on: Sep 20, 2019
 *      Author: Daniel Hamilton
 */

#ifndef SRC_RTDSP_CODEC_GPIO_H_
#define SRC_RTDSP_CODEC_GPIO_H_

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

#define clearCodecLeds()   setCodecLeds(0x0000) // turn all leds off

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initLeds
 * This function initializes GPIO 7:0 as outputs for LED
 * control on the codec board.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodecLeds();

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: setCodecLeds
 * This function sets the Codec LEDs to equal the lower
 * byte of the 32-bit input. The function handles low-true
 * LEDs.
 *
 * Ex. Input = 0xFF, turns all LEDs on.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void setCodecLeds(uint32_t ledData);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: setCodecLeds
 * This function reads in data from the Codec LEDs and
 * outputs a 32-bit value containing the LED information
 * in bits 7:0. The function also handles low-true LEDs.
 *
 * Ex. Output = 0xFF, all LEDs are currently on.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
uint32_t getCodecLeds();

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initSwitches
 * This function initializes GPIO 11:8 as inputs for switch
 * control on the codec board.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodecSwitches();

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: setCodecSwitches
 * This function reads in data from the Codec switches and
 * outputs a 32-bit value containing the switch information
 * in bits 3:0.
 *
 * Ex. Output = 0xF, all switches are currently on.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
uint32_t getCodecSwitches();

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initSwitches
 * This function initializes GPIO 16:14 as inputs for button
 * control on the codec board.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodecButtons();

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: setCodecSwitches
 * This function reads in data from the Codec switches and
 * outputs a 32-bit value containing the switch information
 * in bits 2:0.
 *
 * Ex. Output = 0x7, all switches are currently on.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
uint32_t getCodecButtons();

#endif /* SRC_RTDSP_GPIO_H_ */
