/*
 * RTDSP_ADC.h
 *
 *  Created on: Oct 8, 2019
 *      Author: dhami
 */

#ifndef SRC_RTDSP_ADC_H_
#define SRC_RTDSP_ADC_H_

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

//
// Function to configure and power up ADCs A and B.
//
void initADCs(void);

//
// Function to configure SOCs 0 and 1 of ADCs A and B.
//
void initADCSOCs(void);


#endif /* SRC_RTDSP_ADC_H_ */
