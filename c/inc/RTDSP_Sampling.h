#ifndef SRC_RTDSP_SAMPLING_H__
#define SRC_RTDSP_SAMPLING_H_

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "InitAIC23.h"
#include "AIC23.h"
#include "OneToOneI2CDriver.h"
#include "RTDSP_SramSpi.h"
#include "RTDSP_Timer.h"
#include "RTDSP_CodecGpio.h"

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        TYPEDEFS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

typedef enum i2sSide
{
    LEFT,
    RIGHT
} i2sSide_t;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

#define startDmaCh1()   EALLOW; \
                        DmaRegs.CH1.CONTROL.bit.RUN = 1; \
                        EDIS \

#define startDmaCh2()   EALLOW; \
                        DmaRegs.CH2.CONTROL.bit.RUN = 1; \
                        EDIS \

#define CODEC_MCBSPB_INT_DIS    0
#define CODEC_MCBSPB_INT_EN     1

#define BURST 1
#define TRANSFER 0

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 *           Must be defined in main.c source code
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

extern volatile i2sSide_t ch_sel;
extern volatile float DataInLeft;
extern volatile float DataInRight;
extern volatile int16 DataInMono;
extern volatile Uint16 LR_received;
extern __interrupt void Mcbsp_RxINTB_ISR(void);
extern __interrupt void RTDSP_DMA_CH1_ISR(void);
extern __interrupt void RTDSP_DMA_CH2_ISR(void);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initCodec
 * This function initializes the following parameters..
 *
 * (extern) ch_sel
 * (extern) DataInLeft
 * (extern) DataInRight
 * (extern) LR_received
 *
 * ~ Sets Codec input / output gains to the max
 * ~ Initializes McBSP interrupt for receiving codec samples (assumes Mcbsp_RxINTB_ISR is available)
 * ~ Clear the codec LEDs
 *
 *  FUTURE IMPLEMENTATION:
 * __interrupt void cpuTimer1ISR(void);
 * interrupt void ISR_rightButton(void);
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodec(Uint16 mcbspIntEn);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: init_dma
 * Initialize DMA for ping-pong sampling for word sizes == 16.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void init_dma(int16 * ping, int16 * pong, Uint32 transferSize);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initDmaPingPong
 * Initialize DMA for ping-pong sampling for word sizes == 16.
 *
 * INPUTS:
 * ping - address of the first sample buffer
 * pong - address of the second sample buffer
 * transferSize - size of the frame to transfer before interrupting
 *
 * GLOBALS (must be declared in external file):
 * __interrupt void local_D_INTCH6_ISR(void)
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initDmaPingPong(int16 * ping, int16 * pong, Uint32 transferSize, void(*ISR)(void));

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: startDmaChannels
 * This function starts both DMA channels at approximately the same time
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void startDmaChannels(void);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: pingPong
 * Assuming that the "initDmaPingPong" configuration function was run with the same "ping" and "pong"
 * parameters, this function will swap the buffers in the DMA channels.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pingPong(void);

#endif // SRC_RTDSP_SAMPLING_H_ //
