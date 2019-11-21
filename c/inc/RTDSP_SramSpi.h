/*
 * RTDSP_SramSpi.h
 *
 *  Created on: Sep 23, 2019
 *      Author: dhami
 */

#ifndef SRC_RTDSP_SRAMSPI_H_
#define SRC_RTDSP_SRAMSPI_H_

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

#define SRAM0_MIN_ADDR  0x000000
#define SRAM0_MAX_ADDR  0x01FFFF
#define SRAM1_MIN_ADDR  0x020000
#define SRAM1_MAX_ADDR  0x03FFFF
#define SRAM_MIN_ADDR   SRAM0_MIN_ADDR
#define SRAM_MAX_ADDR   SRAM1_MAX_ADDR
#define SRAM_LENGTH     SRAM1_MAX_ADDR + 1
#define BUF_MASK        SRAM1_MAX_ADDR

void sramSpiInit(void);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: spiTransmit
 * This function transmits a byte of SPI data without
 * consideration of the CS lines. The MISO value is returned
 * as a Uint16.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 spiTransmit(Uint16 data);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: sramWrite16
 * This function burst writes a variable number of bytes
 * to the selected SRAM device through SPI.
 *
 * INPUTS:
 * addr - starting address to write data to.
 * data - pointer to the array of Uint16's to write to
 * len -
 * cs -
 *
 * BYTE ORDER: MSB to LSB
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void sramWrite(Uint32 addr, Uint16 * data, Uint32 len, Uint16 cs);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: sramRead
 * This function burst reads a variable number of Uint16s from
 * the SRAM.
 *
 * INPUTS:
 * addr - starting address to write data to.
 * data - pointer to the array of Uint16's to STORE data to
 * len - number of Uint16's to read
 * cs - SRAM device selected
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 sramRead(Uint32 addr, Uint16 * data, Uint32 len, Uint16 cs);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: sramVirtualWrite
 * This function burst writes a variable number of 16-bit
 * values to the selected SRAM device through SPI. The
 * virtual wrapper handles mapping the CS for SRAM0 and
 * SRAM1 so the user can think of them as a single block.
 *
 * INPUTS:
 * addr - starting address to write data to.
 * data - pointer to the array of Uint16's to write to
 * len - number of bytes to write out
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 sramVirtualWrite(Uint32 addr, Uint16 * data, Uint32 len);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: sramVirtualRead
 * This function burst reads a variable number of 16-bit
 * values from the SRAM through SPI. The virtual wrapper
 * handles mapping the CS for SRAM0 and SRAM1 so the user
 * can think of them as a single block.
 *
 * INPUTS:
 * addr - starting address to write data to.
 * data - pointer to the array of Uint16's to store to
 * len - number of bytes to write out
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 sramVirtualRead(Uint32 addr, Uint16 * data, Uint32 len);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: sramCircularWrite
 * This function will automatically write to the beginning
 * of the SRAM as a circular buffer if the input address
 * is past the SRAM's address range.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 sramCircularWrite(Uint32 addr, Uint16 * data, Uint32 len);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: sramCircularRead
 * This function will automatically read from the beginning
 * of the SRAM as a circular buffer if the input address
 * is past the SRAM's address range.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 sramCircularRead(Uint32 addr, Uint16 * data, Uint32 len);

#endif /* SRC_RTDSP_SRAMSPI_H_ */
