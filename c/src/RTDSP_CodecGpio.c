/*
 * RTDSP_Gpio.c
 *
 *  Created on: Sep 20, 2019
 *      Author: Daniel Hamilton
 */

#include "RTDSP_CodecGpio.h"

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initLeds
 * This function initializes GPIO 7:0 as outputs for LED
 * control on the codec board.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodecLeds()
{
    for (unsigned int i = 0; i <= 7; i++)
    {
        GPIO_setDirectionMode(i, GPIO_DIR_MODE_OUT); // leds set as outputs
        GPIO_setPadConfig(i, GPIO_PIN_TYPE_PULLUP); // internal pullups enabled
        GPIO_setPinConfig(GPIO_0_GPIO0 + i*0x200); // Generic GPIO config separated by 0x200
    }
}

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
void setCodecLeds(uint32_t ledData)
{
    // read, modify, write so other port functions are not ruined.

    // READ -------------------------------------------
    uint32_t portData = GPIO_readPortData(GPIO_PORT_A);

    // MODIFY -----------------------------------------
    portData &= 0xFFFFFF00; // preserves data from bits 31:8
    ledData ^= 0x000000FF;  // inverts bits 7:0
    portData |= ledData;    // adds the byte of new data to the current data

    // WRITE ------------------------------------------
    GPIO_writePortData(GPIO_PORT_A, ledData); // writes low-true data to LEDs
}

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
uint32_t getCodecLeds()
{
    uint32_t ledData = GPIO_readPortData(GPIO_PORT_A);
    ledData &= 0x000000FF; // masks bits 7:0
    ledData ^= 0x000000FF; // inverts bits 7:0
    return ledData;
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initSwitches
 * This function initializes GPIO 11:8 as inputs for switch
 * control on the codec board.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodecSwitches()
{
    for (unsigned int i = 8; i <= 11; i++)
    {
        GPIO_setDirectionMode(i, GPIO_DIR_MODE_IN); // leds set as outputs
        GPIO_setPadConfig(i, GPIO_PIN_TYPE_PULLUP); // internal pullups enabled
        GPIO_setPinConfig(GPIO_0_GPIO0 + i*0x200); // Generic GPIO config separated by 0x200
    }
}

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
uint32_t getCodecSwitches()
{
    uint32_t swData = GPIO_readPortData(GPIO_PORT_A);
    swData >>= 8; // bits 11:8 become bits 3:0
    swData &= 0x0000000F; // masks bits 3:0
    return swData;
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initSwitches
 * This function initializes GPIO 16:14 as inputs for button
 * control on the codec board.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodecButtons()
{
    for (unsigned int i = 14; i <= 16; i++)
    {
        GPIO_setDirectionMode(i, GPIO_DIR_MODE_IN); // leds set as outputs
        GPIO_setPadConfig(i, GPIO_PIN_TYPE_PULLUP); // internal pullups enabled
        GPIO_setPinConfig(GPIO_0_GPIO0 + i*0x200); // Generic GPIO config separated by 0x200
    }
}

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
uint32_t getCodecButtons()
{
    uint32_t buttonData = GPIO_readPortData(GPIO_PORT_A);
    buttonData >>= 14; // bits 16:14 become bits 3:0
    buttonData &= 0x00000007; // masks bits 2:0
    buttonData ^= 0x00000007; // inverts bits 2:0
    return buttonData;
}

