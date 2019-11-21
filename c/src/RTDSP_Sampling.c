/*
 * Using this file requires definining the following variables and functions to be defined in the
 * global section of your source file.
 *
 * volatile i2sSide_t ch_sel;
 * volatile float DataInLeft;
 * volatile float DataInRight;
 * volatile int16 DataInMono;
 * volatile Uint16 LR_received;
 * __interrupt void Mcbsp_RxINTB_ISR(void);
 *
 */

#include "RTDSP_Sampling.h"

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
 * INPUTS:
 * --------------------
 * mcbspIntEn
 * --------------------
 * CODEC_MCBSPB_INT_DIS - disable McBSPb interrupts (used for DMA ping-pong)
 * CODEC_MCBSPB_INT_EN - enable McBSPb interrupts
 *
 * FUTURE IMPLEMENTATION:
 * __interrupt void cpuTimer1ISR(void);
 * interrupt void ISR_rightButton(void);
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodec(Uint16 mcbspIntEn)
{
    // global variable initialization
    ch_sel      = LEFT;
    DataInLeft  = 0;
    DataInRight = 0;
    LR_received = 0;

    //Interrupt_register(INT_TIMER1, &cpuTimer1ISR); // set the timer interrupt to point at the cpuTimerISR
    //Interrupt_disable(INT_TIMER1);
    EALLOW;

    sramSpiInit();              // SPI module for SRAM reads and writes
    lcdInit();                  // initialize GPIO for I2C communication to LCD
    lcdDisableCursorBlinking();
    //gpioTimerCheckInit();       // GPIO 123 used to probe timer interrupt
    //timer1Init(TIMER_PERIOD);   // initialize timer1 interrupt on Int13 at 10 Hz
    initCodecLeds();            // turned off by default
    initCodecButtons();         // set as inputs
    initCodecSwitches();        // set as inputs

    InitSPIA();
    InitBigBangedCodecSPI();

    if (mcbspIntEn == CODEC_MCBSPB_INT_EN)
    {
        Interrupt_enable(INT_MCBSPB_RX);
        Interrupt_register(INT_MCBSPB_RX, &Mcbsp_RxINTB_ISR); // set I2S RX interrupt to ISR address
        InitMcBSPb(1); // initialize I2S for sound input/output with receive interrupt enabled
    }
    else
    {
        Interrupt_disable(INT_MCBSPB_RX);
        InitMcBSPb(0); // initialize I2S for sound input/output with receive interrupt disabled
    }

    InitAIC23();                // initialize Codec's command registers

    clearCodecLeds();

    AIC23MaxIOGain(); // amplify the input lines to the max volume

    // Set codec interrupt to 48 KHz
    Uint16 command = CLKsampleratecontrol (SR48);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    // 48 KHz output
    //float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
    //configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);
}

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
void initDmaPingPong(int16 * ping, int16 * pong, Uint32 transferSize, void(*ISR)(void))
{
    // ----------------------------------------------------------------------------
    // HEADER
    // ----------------------------------------------------------------------------
    DINT;

    //pie vector -> stolen from TI code
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.DMA_CH6_INT= (PINT)ISR; // only 1 interrupt needed for both DMA channels because they run concurrently
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //Initialize DMA -> stolen from TI code
    DMAInitialize();

    // ----------------------------------------------------------------------------
    // DMA_CH6: Configure this channel for collecting new samples from codec
    //                                  AUDIO IN
    // ----------------------------------------------------------------------------

    // source and destination pointers
    volatile Uint16 * DMA_CH6_Source = &McbspbRegs.DRR2.all;
    volatile Uint16 * DMA_CH6_Dest   = (volatile Uint16 *)ping;

    // configure DMA CH6 -> modified from TI code
    DMACH6AddrConfig(DMA_CH6_Dest, DMA_CH6_Source);
    DMACH6BurstConfig(BURST,1,1);
    DMACH6TransferConfig(transferSize-1,1,1); // go to next address - will be ignored when wrap occurs
    DMACH6ModeConfig(74,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,
                     SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                     CHINT_END,CHINT_ENABLE);

    // srcWrapSize, srcWrapStep, dstWrapSize, dstWrapStep
    DMACH6WrapConfig(0x0000, 0x0000, transferSize-1, 0x0000);

    // ----------------------------------------------------------------------------
    // DMA_CH5: Configure this channel for sending processed data to codec audio out
    //                                 AUDIO OUT
    // ----------------------------------------------------------------------------

    // source and destination pointers
    volatile Uint16 * DMA_CH5_Source = (volatile Uint16 *)pong;
    volatile Uint16 * DMA_CH5_Dest   = &McbspbRegs.DXR2.all;

    // configure DMA CH6 -> modified from TI code
    DMACH5AddrConfig(DMA_CH5_Dest, DMA_CH5_Source);
    DMACH5BurstConfig(BURST,1,1);
    DMACH5TransferConfig(transferSize-1,1,1); // src, dst - SRC BUFFER should increment by transfer size
    DMACH5ModeConfig(74,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,
                     SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                     CHINT_END,CHINT_DISABLE);

    // srcWrapSize, srcWrapStep, dstWrapSize, dstWrapStep
    DMACH5WrapConfig(transferSize-1, 0x0000, 0x0000, 0x0000);

    // ----------------------------------------------------------------------------
    // TRAILER
    // ----------------------------------------------------------------------------

    //something about a bandgap voltage -> stolen from TI code
    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;
    EDIS;

    //
    // Enable interrupts required for this example -> stolen from TI code
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER7.bit.INTx6 = 1;   // Enable PIE Group 7, INT 2 (DMA CH2)
    IER |= M_INT7;                         // Enable CPU INT6

    // Enable Global Interrupts
    EINT;
    EnableInterrupts();

    // Start both DMA channels at the same time
    startDmaChannels();
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: startDmaChannels
 * This function starts both DMA channels at approximately the same time
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void startDmaChannels(void)
{
    EALLOW;
    DmaRegs.CH6.CONTROL.bit.RUN = 1;
    DmaRegs.CH5.CONTROL.bit.RUN = 1;
    EDIS;
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: pingPong
 * Swap DMA channels 5/6 source and destinations
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pingPong(void)
{
    EALLOW;

    // grab the current addresses to switch
    Uint32 rxAddr = (Uint32)DmaRegs.CH6.DST_ADDR_SHADOW;
    Uint32 switcherAddr;
    Uint32 txAddr = (Uint32)DmaRegs.CH5.SRC_ADDR_SHADOW;

    // switch the address values
    switcherAddr    = rxAddr;       // 0 = 1
    rxAddr          = txAddr;       // 1 = 2
    txAddr          = switcherAddr; // 2 = 0 = 1(original)

    // Set up the RECEIVE buffer address:
    DmaRegs.CH6.DST_ADDR_SHADOW     = rxAddr;

    // Set up TRANSFER buffer address:
    DmaRegs.CH5.SRC_ADDR_SHADOW     = txAddr;
    EDIS;
}
