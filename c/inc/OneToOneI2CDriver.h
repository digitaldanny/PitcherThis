/*
 * OneToOneI2CDriver.h
 *
 *  Created on: Sep 24, 2016
 *      Author: Raz Aloni
 */

#ifndef ONETOONEI2CDRIVER_H_
#define ONETOONEI2CDRIVER_H_

#include "F2837xD_Examples.h"

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+
 *                   WRAPPER MACROS
 * +=====+=====+=====+=====+=====+=====+=====+=====+
 */

#define lcdRow1()             lcdCommand(0x80 | 0x00) // moves cursor to the first line of the LCD
#define lcdRow2()             lcdCommand(0x80 | 0x40) // moves cursor to the second line of the LCD
#define lcdClear()            lcdCommand(0x01)        // clears the LCD screen
#define lcdCursor(OFFSET)     lcdCommand(0x80 | OFFSET) // moves cursor to CURSOR value
#define lcdCursorRow1(OFFSET) lcdCommand(0x80 | OFFSET) // moves cursor to CURSOR value
#define lcdCursorRow2(OFFSET) lcdCommand(0x80 | (0x40 + OFFSET));
#define lcdDisableCursorBlinking()   lcdCommand(0x0C)    // stops lcd cursor from blinking

#define lcdClearBottomRow()         char clearString[] = "                "; \
                                    lcdRow2(); \
                                    lcdString((Uint16 *)&clearString) \

#define lcdClearTopRow()            char clearString[] = "                "; \
                                    lcdRow1(); \
                                    lcdString((Uint16 *)&clearString) \

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+
 *                    FUNCTIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+
 */
/*
 * <summary>
 * 	Initializes the I2C to run in Master Mode for a One-To-One connection
 * </summary>
 * <param="slaveAddress">Address of the slave device to write to</param>
 * <param="sysClkMhz">System Clock Frequency in Mhz</param>
 * <param="I2CClkKHz">Desired I2C Clock Frequency in KHz</param>
 */
void I2C_O2O_Master_Init(Uint16 slaveAddress, float32 sysClkMhz, float32 I2CClkKHz);

/*
 * <summary>
 * 	Sends bytes via I2C
 * </summary>
 * <param="values">Pointer to array of bytes to send</param>
 * <param-"length">Length of array</param>
 */
void I2C_O2O_SendBytes(Uint16 * const values, Uint16 length);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: lcdByte____
 * This function translates LCD messages to work with the I2C
 * backpack. The function transmits 1 nibble of the byte at a
 * time while controlling the LCD's RS, E, etc bits.
 *
 * lcdByteCmd => transmits byte to command register.
 * lcdByteData => transmit byte to data register.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void lcdByteCmd(Uint16 byte);
void lcdByteData(Uint16 byte);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: lcdInit
 * This function initializes the LCD to work in 4-bit 2-line
 * mode, turn the display on, turn the cursor on, and clear
 * the screen.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void lcdInit();

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: lcdCommand
 * This function transfers a command value to
 * the LCD over through an I2C backpack connected
 * on pins 104 (SDA) and 105 (SCL).
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 */
void lcdCommand(Uint16 cmd);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: lcdString
 * This function transfers a string value to
 * the LCD over through an I2C backpack connected
 * on pins 104 (SDA) and 105 (SCL).
 *
 * INPUTS:
 * <str> - pointer to the first character of the string transfer
 * to the data register.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void lcdString(Uint16 * str);

#endif /* ONETOONEI2CDRIVER_H_ */
