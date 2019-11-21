/*
 * InitAIC23.h
 *
 *  Created on: Oct 11, 2019
 *      Author: dhami
 */

#ifndef INC_INITAIC23_H_
#define INC_INITAIC23_H_

#include <stdint.h>
#include <F28x_Project.h>
#include "AIC23.h"

/***************** Defines ***************/
//#define SmallDelay() for(volatile long  i = 0; i < 500000; i++) -> MOVED TO AIC23.h
/***************** Defines ***************/
#define CodecSPI_CLK_PULS {EALLOW; GpioDataRegs.GPASET.bit.GPIO18 = 1; GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;}
#define CodecSPI_CS_LOW {EALLOW; GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;}
#define CodecSPI_CS_HIGH {EALLOW; GpioDataRegs.GPASET.bit.GPIO19 = 1;}

/***************** User Functions *****************/
void InitMcBSPb(Uint16 mcbspIntEn);
void InitSPIA();
void InitAIC23();
void SpiTransmit(uint16_t data);
void SmallDelay();

void InitBigBangedCodecSPI();
void BitBangedCodecSpiTransmit(Uint16 data);

/*
 * SUMMARY: AIC23MaxIOGain
 * This function sets the line_in and hp_out gains to the max.
 */
void AIC23MaxIOGain(void);

#endif /* INC_INITAIC23_H_ */
