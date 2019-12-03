/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * | SUMMARY: main.c                                                 |
 * | This program can be configured to perform 64, 256, or 512 point |
 * | DFT/FFT calculations of an input signal and output the max      |
 * | frequency and gain on the LCD.                                  |
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "RTDSP.h"
#include <math.h>

#include "fpu.h"
#include "dsp.h"
#include "fpu32/fpu_cfft.h"

#include "kiss_fft.h"
#include "kiss_fftr.h"

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                      CONFIGURATIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

// PITCHSHIFTER - ADC controls pitch shifting between -24 to 24 steps.
// AUTOTUNE - Automatically pitch shift to selected scale.
#define PITCHSHIFTER
#define V2

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

// FFT related
// 9 => 512 pt, 10 => 1024 pt FFT
// (hamming LUT will have to be regenerated if this value changes)
#define CFFT_STAGES         10

#define CFFT_SIZE           (1 << CFFT_STAGES)
#define CFFT_FREQ_PER_BIN   (48000.0f / (float)CFFT_SIZE)
#define CFFT_SIZE_MIN_1     (CFFT_SIZE - 1)
#define CFFT_SIZE_X2_MASK   ((2*CFFT_SIZE) - 1)
#define STFT_HOP            (CFFT_SIZE>>2); // 75% overlap

#define VREF                        3.0f
#define INT_TO_ASCII(VAL)           VAL + 0x30

#define FORCE_ADC_CONVERSION        AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1
#define WAIT_FOR_ADC_CONVERSION     while (AdcaRegs.ADCCTL1.bit.ADCBSY == 1)
#define CLEAR_ADC_FLAG              AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 0x0001

#define FORCE_ADC_CONVERSION_B      AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1
#define WAIT_FOR_ADC_CONVERSION_B   while (AdcbRegs.ADCCTL1.bit.ADCBSY == 1)
#define CLEAR_ADC_FLAG_B            AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 0x0001

#define MAX_SHIFT                   6.0f
#define MIN_SHIFT                   -12.0f

#define LEFT_BUTTON                 0x4
#define MIDDLE_BUTTON               0x2
#define RIGHT_BUTTON                0x1

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       TYPEDEFS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

typedef struct frame
{
    int16 buffer[2*CFFT_SIZE];      // buffer is large enough LR or mono processing
    struct frame * prevFrame;       // points to the previous frame
    struct frame * nextFrame;       // points to the next frame
} frame_t;

typedef struct polar
{
    float magnitude;
    float freq;
} polar_t;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       PROTOTYPES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

void adcA0Init();
void adcB2Init();

kiss_fft_cpx* pitchShift(kiss_fft_cpx * freq_bins,
                         Uint16 cfft_size,
                         float shift);

__interrupt void Mcbsp_RxINTB_ISR(void);
__interrupt void DMA_FRAME_COMPLETE_ISR(void);

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

volatile i2sSide_t ch_sel;   // unused in this project but required to compile RTDSP_Sampling
volatile Uint16 boolTimer1;  // allows main to determine if the timer has run or not

// RTDSP_Sampling Requirements
volatile i2sSide_t ch_sel;
volatile float DataInLeft;
volatile float DataInRight;
volatile int16 DataInMono;
volatile Uint16 LR_received;
volatile float monof;

// ping pong buffers
#pragma DATA_SECTION(frames, "DMAACCESSABLE")    // DMA-accessible RAM
volatile Uint16 dma_flag;   // notifies program to begin dft computation
frame_t frames[3];          // 3 buffers for ping-pong-panging sampling, processing, and outputting
frame_t * inFrame;          // points at the frame to store new values
frame_t * fftFrame;         // points at the frame to be processed
frame_t * outFrame;         // points at the frame to output
polar_t testPointMax;

// +-----+-----+-----+-----+-----+-----+-----+
// KISS FFT LIBRARY SETUP
// +-----+-----+-----+-----+-----+-----+-----+
#pragma DATA_SECTION(rin1,"CFFTdata1");
kiss_fft_scalar rin1[CFFT_SIZE]; // real input

#pragma DATA_SECTION(rin2,"CFFTdata2");
kiss_fft_scalar rin2[CFFT_SIZE]; // real input

#pragma DATA_SECTION(cout,"CFFTdata3");
kiss_fft_cpx cout[CFFT_SIZE]; // complex output

#pragma DATA_SECTION(fftOutBuff,"CFFTdata4");
kiss_fft_cpx fftOutBuff[CFFT_SIZE]; // complex output

#pragma DATA_SECTION(overlayBuff,"CFFTdata5");
float overlayBuff[CFFT_SIZE];

kiss_fftr_cfg  kiss_fftr_state;
kiss_fftr_cfg  kiss_fftri_state;
// +-----+-----+-----+-----+-----+-----+-----+

float * binFft;
float * prevInPtr;
float * currInPtr;

// PITCH SHIFTING
volatile Uint16 robotEffectEn;
volatile Uint16 adcBResult;
volatile Uint16 adcAResult;
volatile float shift;
volatile float kFilter;
char wr[6] = "#XX.X"; // store ASCII versions of DFT magnitude in here..

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                         MAINS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
#ifdef PITCHSHIFTER
/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * Pitch shifting between -24 to 24 steps using ADC input.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main(void)
{
    // *************************************************************************
    // INITIALIZE GLOBALS
    // ------------------------------------------------------------------------
    frames[0].nextFrame     = &frames[1];
    frames[1].nextFrame     = &frames[2];
    frames[2].nextFrame     = &frames[0];

    frames[0].prevFrame     = &frames[2];
    frames[1].prevFrame     = &frames[0];
    frames[2].prevFrame     = &frames[1];

    inFrame                 = &frames[0];       // first buffer to be filled with samples
    fftFrame                = &frames[1];       // first buffer to be processed
    outFrame                = &frames[2];       // first buffer to output processed samples

    dma_flag                = 0;

    currInPtr               = (float*)&rin1[0];
    prevInPtr               = (float*)&rin2[0];     // prev buff must be initialized to 0 for stft

    robotEffectEn           = 0;
    // ------------------------------------------------------------------------

    // prev buff must be initialized to 0 for stft
    for (Uint16 i = 0; i < CFFT_SIZE_X2_MASK; i++) prevInPtr[i] = 0.0f;

    // *************************************************************************
    // HARDWARE INITIALIZATIONS
    // ------------------------------------------------------------------------
    DINT;  // Enable Global interrupt INTM
    DRTM;  // Enable Global realtime interrupt DBGM

    InitSysCtrl();      // disable watchdog
    InitPieCtrl();      // set PIE ctrl registers to default state
    InitPieVectTable(); // set PIE vectors to default shell ISRs

    // sets up codec and processor for sampling at 48 KHz
    initDmaPingPong(&inFrame->buffer[0], &outFrame->buffer[0], CFFT_SIZE, &DMA_FRAME_COMPLETE_ISR);
    initCodec(CODEC_MCBSPB_INT_DIS);
    gpioTimerCheckInit();
    adcA0Init();
    adcB2Init();

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // update frequencies on LCD
    char s1[] = "filter k = ";
    lcdRow1();
    lcdString((Uint16 *)&s1);

    // update shift amount on
    char s2[] = "shift = ";
    lcdRow2();
    lcdString((Uint16 *)&s2);

    /*
     * fftptr - assigns where the fft malloc will start at
     * lenmem - assign to max value so fftptr is used for malloc
     */
    kiss_fft_cpx * bins;
    int fftSize = CFFT_SIZE;
    kiss_fftr_state = kiss_fftr_alloc(fftSize, 0, NULL, NULL);
    kiss_fftri_state = kiss_fftr_alloc(fftSize, 1, NULL, NULL);
    EALLOW;

    Uint16 switches;
    Uint16 prevSwitches = 1234; // switches != prevSwitches initially

    while(1)
    {
        // +--------------------------------------------------------------------------------------+
        // CREATE ADC VALUES WHILE WAITING FOR NEW SAMPLES
        // +--------------------------------------------------------------------------------------+

        // line in volume control
        switches = getCodecSwitches();
        if (switches != prevSwitches)
        {
            // set input gain to 0dB by default
            Uint16 command = linput_volctl (0x17 - 8 - switches); // 12dB - 8*1.5dB (-8 => 0dB, -12 => -6dB
            BitBangedCodecSpiTransmit (command);
            SmallDelay();

            // set output gain to 0dB by default
            command = lhp_volctl (0x69 - 8 - switches); // 12dB - 8*1.5dB (-8 => 0dB, -12 => -6dB
            BitBangedCodecSpiTransmit (command);
            SmallDelay();
        }
        prevSwitches = switches;

        // Select the audio source..
        Uint16 buttons = getCodecButtons();
        if (buttons == LEFT_BUTTON || buttons == MIDDLE_BUTTON)
        {
            // turn mic on
            Uint16 command = fullpowerup();
            BitBangedCodecSpiTransmit (command);
            SmallDelay();

            command = aaudpath();
            BitBangedCodecSpiTransmit (command);
            SmallDelay();

            lcdClearTopRow();
            lcdCursorRow1(0);
            char str[] = "MIC ON";
            lcdString((Uint16 *)&str);

            // only enable the robot vocal effect if the middle button
            // is pushed and the mic is enabled.
            if (buttons == MIDDLE_BUTTON)
            {
                lcdCursorRow1(7);
                char str[] = "-R";
                lcdString((Uint16 *)&str);

                robotEffectEn = 1;
            }
            else
                robotEffectEn = 0;
        }
        else if (buttons == RIGHT_BUTTON)
        {
            // turn mic off
            Uint16 command = nomicpowerup();
            BitBangedCodecSpiTransmit (command);
            SmallDelay();

            command = nomicaaudpath();
            BitBangedCodecSpiTransmit (command);
            SmallDelay();

            lcdClearTopRow();
            lcdCursorRow1(0);
            char str[] = "MIC OFF";
            lcdString((Uint16 *)&str);

            robotEffectEn = 0;
        }

        FORCE_ADC_CONVERSION;     // force ADC to convert on A0 (shift)
        FORCE_ADC_CONVERSION_B;   // force ADC to convert on B2 (kFilter)

        // find ADC value to pitch shift by
        WAIT_FOR_ADC_CONVERSION;  // wait for first conversion to complete
        CLEAR_ADC_FLAG;           // clear the interrupt flag generated by the conversion
        adcAResult = AdcaResultRegs.ADCRESULT0; // save results of the conversion
        shift = (MAX_SHIFT - MIN_SHIFT)*(float)adcAResult/(float)4095 + MIN_SHIFT;

        // find ADC value to change filter bin by
        WAIT_FOR_ADC_CONVERSION_B;  // wait for first conversion to complete
        CLEAR_ADC_FLAG_B;           // clear the interrupt flag generated by the conversion
        adcBResult = AdcbResultRegs.ADCRESULT0; // save results of the conversion
        kFilter = (CFFT_SIZE/2-1)*(float)adcBResult/(float)4095;

        float tens   = abs(shift) / 10.0;
        float ones   = (tens - (Uint16)tens) * 10;
        float tenths = (ones - (Uint16)ones) * 10;

        float k_hundreds = abs(kFilter) / 100.0;
        float k_tens   = (k_hundreds - (Uint16)k_hundreds) * 10;
        float k_ones   = (k_tens - (Uint16)k_tens) * 10;

        // +--------------------------------------------------------------------------------------+
        // NEW SAMPLES ARE READY FOR USER APPLICATION
        // +--------------------------------------------------------------------------------------+

        if (dma_flag)
        {
            timerOn();

             // create mono samples by averaging left and right samples and store to the fft buffer
             for (int i = 0; i < CFFT_SIZE_X2_MASK; i+=2)
                 currInPtr[i>>1] = ((float)fftFrame->buffer[i] + (float)fftFrame->buffer[i+1])/2.0f;

             kiss_fftr(kiss_fftr_state, currInPtr, cout); // FFT

             // +----------------------------------------------------------------------------+
             //                               USER APP END
             // +----------------------------------------------------------------------------+

             // Create the index for filtering in the buffer
             // Uint16 k = 442; // tested value to remove white noise (32KHz sampling) -> 13.8 KHz
             // Uint16 k = 295; // (48KHz sampling) -> 13.8 KHz
             Uint16 k = (Uint16)kFilter; // controllable using B2 ADC

             // robotic voice effect zeros out the phase without changing the magnitude of the
             // frequency bins.. (optional effect)
             if (robotEffectEn)
             {
                 for (Uint16 i = 0; i < k; i++)
                 {
                     cout[i].r = sqrtf(cout[i].r*cout[i].r + cout[i].i*cout[i].i);
                     cout[i].i = 0.0f;
                 }
             }

             // get rid of the white noise in upper bins
             for (Uint16 i = k; i < CFFT_SIZE/2; i++)
             {
                 cout[i].r = 0.0f;
                 cout[i].i = 0.0f;
             }

             bins = (kiss_fft_cpx*)&cout;
             bins = pitchShift(bins, CFFT_SIZE, shift);

             // +----------------------------------------------------------------------------+
             //                               USER APP END
             // +----------------------------------------------------------------------------+

             kiss_fftri(kiss_fftri_state, bins, currInPtr); // IFFT

             // output the fft results
             for (int i = 0; i < CFFT_SIZE_MIN_1; i++)
             {
                 fftFrame->buffer[2*i]      = (int16)(currInPtr[i] * 0.02);    // left channel
                 fftFrame->buffer[2*i+1]    = fftFrame->buffer[2*i];           // right channel
             }

             // switch the inBuff pointers so the currentBuff becomes the previous input buffer
             Uint32 tempSwitchingPtr = (Uint32)prevInPtr;
             prevInPtr = currInPtr;
             currInPtr = (float*)tempSwitchingPtr;

             // Write to the LCD AFTER processing has completed
             // handle adc input to work with pitch function
             if (shift > 0.0f)
                 wr[0] = '+';
             else
                 wr[0] = '-';

             // update LCD with shift/kFilter data
             lcdCursorRow2(8);
             wr[1] = INT_TO_ASCII((Uint16)tens + 0.5f);
             wr[2] = INT_TO_ASCII((Uint16)ones); // 1's place (Ex. [1].23)
             wr[3] = '.';
             wr[4] = INT_TO_ASCII((Uint16)tenths); // 10th's place (Ex. 1.[2]3)
             wr[5] = '\0';
             lcdString((Uint16 *)&wr);

             lcdCursorRow1(11);
             wr[0] = INT_TO_ASCII((Uint16)k_hundreds);
             wr[1] = INT_TO_ASCII((Uint16)k_tens);
             wr[2] = INT_TO_ASCII((Uint16)k_ones);
             wr[3] = '\0';
             wr[4] = '\0';
             wr[5] = '\0';
             lcdString((Uint16 *)&wr);

             timerOff();
             dma_flag = 0;
        }
    }
}
#endif

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 * FUNCTIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: pitchShift
 * V1 -> in version 1, this function can only pitch shift the bins by
 *       shifting bins to the left or right by the shift size.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
kiss_fft_cpx* pitchShift(kiss_fft_cpx * freq_bins, Uint16 cfft_size, float shift)
{
    /*
     * V1: Bin shifting proof of concept
     * ~ algorithm takes a long time to move all values to the left or right
     * ~ reassigns values to new address
     */
#ifdef V1
    // shift all bins either to the left or right
    if (shift > 0.001)
    {
        int shiftSize = (int)shift;

        // shift bins to the right (higher pitch)
        for (int i = cfft_size-1; i >= 0; i--)
        {
            int ishift = i - shiftSize;
            if (ishift < 0)
            {
                // pad with 0's if index is out of bounds
                freq_bins[i].r = 0.0f;
                freq_bins[i].i = 0.0f;
            }
            else
            {
                // shift bins over if within range
                freq_bins[i].r = freq_bins[ishift].r;
                freq_bins[i].i = freq_bins[ishift].i;
            }
        }
    }
    else if (shift < -0.001)
    {
        int shiftSize = (int)abs(shift);

        // shift bins to the left (deeper pitch)
        for (int i = 0; i < cfft_size; i++)
        {
            int ishift = i + shiftSize;
            if (ishift > cfft_size-1)
            {
                // pad with 0's if out of bounds index
                freq_bins[i].r = 0.0f;
                freq_bins[i].i = 0.0f;
            }
            else
            {
                // shift bins over if within range
                freq_bins[i].r = freq_bins[ishift].r;
                freq_bins[i].i = freq_bins[ishift].i;
            }
        }
    }
#endif
    /*
     * V2: Bin array sliding
     * ~ this algorithm should improve the time it takes to pitch shift by a full bin amount
     * ~ instead of reassigning bin values, it simply slides the pointer up or down
     */
#ifdef V2
    // shift all bins either to the left or right
    int32 indexShift = (int32)-1*shift;
    freq_bins = (kiss_fft_cpx*)((int32)freq_bins + (int32)sizeof(kiss_fft_cpx)*indexShift);
#endif
    return freq_bins;
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: adcA0Init
 * This function sets single-ended 12-bit ADC conversions on
 * ADCIN A0 for 200MHz system clock.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void adcA0Init()
{
    EALLOW;
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0; // 12 bit resolution
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0; // single ended
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;   // divide by 5

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // power up the circuitry + enable ADC
    DELAY_US(1000);

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0; // software trigger only
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0; // single ended channel 0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 15; // #clock cycles for sample time
    DELAY_US(1000);
}

void adcB2Init()
{
    EALLOW;
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0; // 12 bit resolution
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0; // single ended
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;   // divide by 5

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // power up the circuitry + enable ADC
    DELAY_US(1000);

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0; // software trigger only
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2; // single ended channel 2
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 15; // #clock cycles for sample time
    DELAY_US(1000);
}

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 * INTERRUPT SERVICE ROUTINES (ISR)
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * DMA_FRAME_COMPLETE_ISR - DMA ISR for channel 6
 * This interrupt ping-pongs the sampling buffer and the processing
 * buffer.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
__interrupt void DMA_FRAME_COMPLETE_ISR(void)
{
    EALLOW;
    GpioDataRegs.GPDDAT.bit.GPIO123 = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP7; // ACK to receive more interrupts from this PIE groups
    EDIS;

    // rotate input buffers
    inFrame = inFrame->prevFrame;    // previous OUT
    fftFrame = fftFrame->prevFrame;  // previous IN
    outFrame = outFrame->prevFrame;  // previous FFT

    // switch buffer end-points on DMA channels
    pingPongPang(&inFrame->buffer[0], &outFrame->buffer[0]);

    // trigger application processing
    dma_flag = 1;

    startDmaChannels();
    GpioDataRegs.GPDDAT.bit.GPIO123 = 0;
}

/* Unused interrupt handles error thrown by RTDSP_Sampling.c */
__interrupt void Mcbsp_RxINTB_ISR(void){}
