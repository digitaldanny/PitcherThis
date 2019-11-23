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

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                      CONFIGURATIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

// PITCHSHIFTER - ADC controls pitch shifting between -24 to 24 steps.
// AUTOTUNE - Automatically pitch shift to selected scale.
#define PITCHSHIFTER

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

// FFT related
#define CFFT_STAGES         9
#define CFFT_SIZE           (1 << CFFT_STAGES)
#define CFFT_FREQ_PER_BIN   (48000.0f / (float)CFFT_SIZE)
#define CFFT_SIZE_MIN_1     (CFFT_SIZE - 1)

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
polar_t searchMaxBin (float * bin, Uint16 len, float freqPerBin);

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

const float hanningLUT[512] = {
            0.000000f, 0.000038f, 0.000151f, 0.000340f, 0.000605f, 0.000945f, 0.001360f, 0.001851f, 0.002417f, 0.003058f, 0.003775f, 0.004566f, 0.005433f, 0.006374f, 0.007390f, 0.008480f,
            0.009645f, 0.010884f, 0.012196f, 0.013583f, 0.015043f, 0.016576f, 0.018182f, 0.019862f, 0.021614f, 0.023438f, 0.025334f, 0.027302f, 0.029341f, 0.031452f, 0.033633f, 0.035885f,
            0.038207f, 0.040599f, 0.043061f, 0.045591f, 0.048190f, 0.050858f, 0.053593f, 0.056396f, 0.059266f, 0.062203f, 0.065205f, 0.068274f, 0.071408f, 0.074606f, 0.077869f, 0.081196f,
            0.084586f, 0.088038f, 0.091554f, 0.095130f, 0.098769f, 0.102467f, 0.106226f, 0.110044f, 0.113922f, 0.117857f, 0.121851f, 0.125901f, 0.130009f, 0.134172f, 0.138390f, 0.142663f,
            0.146990f, 0.151371f, 0.155804f, 0.160289f, 0.164826f, 0.169413f, 0.174051f, 0.178737f, 0.183472f, 0.188255f, 0.193085f, 0.197962f, 0.202884f, 0.207851f, 0.212862f, 0.217917f,
            0.223014f, 0.228153f, 0.233334f, 0.238554f, 0.243814f, 0.249113f, 0.254450f, 0.259824f, 0.265234f, 0.270680f, 0.276160f, 0.281674f, 0.287222f, 0.292801f, 0.298412f, 0.304053f,
            0.309724f, 0.315423f, 0.321151f, 0.326905f, 0.332686f, 0.338492f, 0.344323f, 0.350177f, 0.356053f, 0.361952f, 0.367871f, 0.373810f, 0.379768f, 0.385745f, 0.391739f, 0.397749f,
            0.403774f, 0.409814f, 0.415868f, 0.421935f, 0.428013f, 0.434102f, 0.440201f, 0.446309f, 0.452426f, 0.458549f, 0.464679f, 0.470814f, 0.476953f, 0.483096f, 0.489242f, 0.495389f,
            0.501537f, 0.507685f, 0.513831f, 0.519975f, 0.526117f, 0.532254f, 0.538387f, 0.544513f, 0.550633f, 0.556746f, 0.562850f, 0.568944f, 0.575028f, 0.581100f, 0.587160f, 0.593207f,
            0.599240f, 0.605258f, 0.611260f, 0.617246f, 0.623213f, 0.629162f, 0.635091f, 0.641000f, 0.646888f, 0.652753f, 0.658596f, 0.664414f, 0.670207f, 0.675975f, 0.681716f, 0.687430f,
            0.693115f, 0.698771f, 0.704397f, 0.709993f, 0.715556f, 0.721087f, 0.726584f, 0.732047f, 0.737476f, 0.742868f, 0.748223f, 0.753541f, 0.758821f, 0.764061f, 0.769262f, 0.774421f,
            0.779540f, 0.784616f, 0.789649f, 0.794638f, 0.799583f, 0.804482f, 0.809336f, 0.814142f, 0.818901f, 0.823612f, 0.828274f, 0.832887f, 0.837449f, 0.841960f, 0.846419f, 0.850826f,
            0.855180f, 0.859480f, 0.863726f, 0.867917f, 0.872052f, 0.876131f, 0.880153f, 0.884118f, 0.888024f, 0.891872f, 0.895661f, 0.899390f, 0.903058f, 0.906666f, 0.910212f, 0.913696f,
            0.917117f, 0.920476f, 0.923770f, 0.927001f, 0.930167f, 0.933269f, 0.936304f, 0.939274f, 0.942177f, 0.945014f, 0.947783f, 0.950484f, 0.953118f, 0.955683f, 0.958179f, 0.960605f,
            0.962962f, 0.965249f, 0.967466f, 0.969612f, 0.971687f, 0.973691f, 0.975623f, 0.977483f, 0.979271f, 0.980987f, 0.982630f, 0.984200f, 0.985696f, 0.987120f, 0.988469f, 0.989745f,
            0.990947f, 0.992074f, 0.993127f, 0.994106f, 0.995010f, 0.995839f, 0.996593f, 0.997272f, 0.997875f, 0.998404f, 0.998857f, 0.999235f, 0.999537f, 0.999764f, 0.999915f, 0.999991f,
            0.999991f, 0.999915f, 0.999764f, 0.999537f, 0.999235f, 0.998857f, 0.998404f, 0.997875f, 0.997272f, 0.996593f, 0.995839f, 0.995010f, 0.994106f, 0.993127f, 0.992074f, 0.990947f,
            0.989745f, 0.988469f, 0.987120f, 0.985696f, 0.984200f, 0.982630f, 0.980987f, 0.979271f, 0.977483f, 0.975623f, 0.973691f, 0.971687f, 0.969612f, 0.967466f, 0.965249f, 0.962962f,
            0.960605f, 0.958179f, 0.955683f, 0.953118f, 0.950484f, 0.947783f, 0.945014f, 0.942177f, 0.939274f, 0.936304f, 0.933269f, 0.930167f, 0.927001f, 0.923770f, 0.920476f, 0.917117f,
            0.913696f, 0.910212f, 0.906666f, 0.903058f, 0.899390f, 0.895661f, 0.891872f, 0.888024f, 0.884118f, 0.880153f, 0.876131f, 0.872052f, 0.867917f, 0.863726f, 0.859480f, 0.855180f,
            0.850826f, 0.846419f, 0.841960f, 0.837449f, 0.832887f, 0.828274f, 0.823612f, 0.818901f, 0.814142f, 0.809336f, 0.804482f, 0.799583f, 0.794638f, 0.789649f, 0.784616f, 0.779540f,
            0.774421f, 0.769262f, 0.764061f, 0.758821f, 0.753541f, 0.748223f, 0.742868f, 0.737476f, 0.732047f, 0.726584f, 0.721087f, 0.715556f, 0.709993f, 0.704397f, 0.698771f, 0.693115f,
            0.687430f, 0.681716f, 0.675975f, 0.670207f, 0.664414f, 0.658596f, 0.652753f, 0.646888f, 0.641000f, 0.635091f, 0.629162f, 0.623213f, 0.617246f, 0.611260f, 0.605258f, 0.599240f,
            0.593207f, 0.587160f, 0.581100f, 0.575028f, 0.568944f, 0.562850f, 0.556746f, 0.550633f, 0.544513f, 0.538387f, 0.532254f, 0.526117f, 0.519975f, 0.513831f, 0.507685f, 0.501537f,
            0.495389f, 0.489242f, 0.483096f, 0.476953f, 0.470814f, 0.464679f, 0.458549f, 0.452426f, 0.446309f, 0.440201f, 0.434102f, 0.428013f, 0.421935f, 0.415868f, 0.409814f, 0.403774f,
            0.397749f, 0.391739f, 0.385745f, 0.379768f, 0.373810f, 0.367871f, 0.361952f, 0.356053f, 0.350177f, 0.344323f, 0.338492f, 0.332686f, 0.326905f, 0.321151f, 0.315423f, 0.309724f,
            0.304053f, 0.298412f, 0.292801f, 0.287222f, 0.281674f, 0.276160f, 0.270680f, 0.265234f, 0.259824f, 0.254450f, 0.249113f, 0.243814f, 0.238554f, 0.233334f, 0.228153f, 0.223014f,
            0.217917f, 0.212862f, 0.207851f, 0.202884f, 0.197962f, 0.193085f, 0.188255f, 0.183472f, 0.178737f, 0.174051f, 0.169413f, 0.164826f, 0.160289f, 0.155804f, 0.151371f, 0.146990f,
            0.142663f, 0.138390f, 0.134172f, 0.130009f, 0.125901f, 0.121851f, 0.117857f, 0.113922f, 0.110044f, 0.106226f, 0.102467f, 0.098769f, 0.095130f, 0.091554f, 0.088038f, 0.084586f,
            0.081196f, 0.077869f, 0.074606f, 0.071408f, 0.068274f, 0.065205f, 0.062203f, 0.059266f, 0.056396f, 0.053593f, 0.050858f, 0.048190f, 0.045591f, 0.043061f, 0.040599f, 0.038207f,
            0.035885f, 0.033633f, 0.031452f, 0.029341f, 0.027302f, 0.025334f, 0.023438f, 0.021614f, 0.019862f, 0.018182f, 0.016576f, 0.015043f, 0.013583f, 0.012196f, 0.010884f, 0.009645f,
            0.008480f, 0.007390f, 0.006374f, 0.005433f, 0.004566f, 0.003775f, 0.003058f, 0.002417f, 0.001851f, 0.001360f, 0.000945f, 0.000605f, 0.000340f, 0.000151f, 0.000038f, 0.000000f
     };

// ping pong buffers
#pragma DATA_SECTION(frames, "DMAACCESSABLE")    // DMA-accessible RAM
volatile Uint16 dma_flag;   // notifies program to begin dft computation
frame_t frames[3];          // 3 buffers for ping-pong-panging sampling, processing, and outputting
frame_t * inFrame;          // points at the frame to store new values
frame_t * fftFrame;         // points at the frame to be processed
frame_t * outFrame;         // points at the frame to output
polar_t testPointMax;
#if defined(PT1) || defined(PT2)
float bin[NUM_DFT_BINS];    // stores result of dft256 function
#endif

#pragma DATA_SECTION(CFFTin1Buff,"CFFTdata1");  //Buffer alignment,optional for CFFT_f32u - required by CFFT_f32
float   CFFTin1Buff[CFFT_SIZE*2];

#pragma DATA_SECTION(CFFTin2Buff,"CFFTdata2");  //Buffer alignment,optional for CFFT_f32u - required by CFFT_f32
float   CFFTin2Buff[CFFT_SIZE*2];

#pragma DATA_SECTION(CFFToutBuff,"CFFTdata3");  //Buffer alignment,optional for CFFT_f32u - required by CFFT_f32
float   CFFToutBuff[CFFT_SIZE*2];

#pragma DATA_SECTION(CFFTF32Coef,"CFFTdata4");  //Buffer alignment,optional for CFFT_f32u - required by CFFT_f32
float   CFFTF32Coef[CFFT_SIZE];                 // Twiddle buffer

float * binFft;

CFFT_F32_STRUCT cfft;

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
void main()
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

    inFrame                 = &frames[0];   // first buffer to be filled with samples
    fftFrame                = &frames[1];   // first buffer to be processed
    outFrame                = &frames[2];   // first buffer to output processed samples

    cfft.CoefPtr            = CFFTF32Coef;   //Twiddle factor table
    cfft.Stages             = CFFT_STAGES;
    cfft.FFTSize            = CFFT_SIZE;

    dma_flag                = 0;
    // ------------------------------------------------------------------------


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

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // update frequencies on LCD
    char s1[] = "PITCH SHIFTER";
    lcdRow1();
    lcdString((Uint16 *)&s1);

    CFFT_f32_sincostable(&cfft); // Calculate twiddle factor
    // ------------------------------------------------------------------------

    while(1)
    {
        if (dma_flag)
        {
            timerOn();

            // *************************************************************************
            // CFFT STRUCT SETUP & HANNING WINDOW
            // ------------------------------------------------------------------------

            // Store input samples into CFFTin1Buff:
            //     CFFTin1Buff[0] = real[0]
            //     CFFTin1Buff[1] = imag[0]

            // convert int16 LR samples to float mono samples
            for (Uint32 i = 0; i < 2*CFFT_SIZE_MIN_1; i+=2)
            {
                monof = ((float)fftFrame->buffer[i] + (float)fftFrame->buffer[i+1])/2.0f;
                //CFFTin1Buff[i] = hanningLUT[i] * monof; // real (average of L and R channels)
                CFFTin1Buff[i] = monof;
                CFFTin1Buff[i+1] = 0.0f; // imaginary
            }

            cfft.InPtr = &CFFTin1Buff[0];  //Input/output or middle stage of ping-pong buffer
            cfft.OutPtr = &CFFToutBuff[0]; //Output or middle stage of ping-pong buffer
            // *************************************************************************


            // *************************************************************************
            // PROCESSING LAYER
            // ------------------------------------------------------------------------
            CFFT_f32u(&cfft);
            cfft.InPtr  = cfft.CurrentOutPtr;  // ICFFT input pointer
            cfft.OutPtr = cfft.CurrentInPtr;   // ICFFT output pointer
            ICFFT_f32(&cfft);                  // Calculate the ICFFT

            // save the valid real parts of the icfft for left and right channels
            for (Uint16 i = 0; i < 2*CFFT_SIZE_MIN_1; i+=2)
            {
                fftFrame->buffer[i] = (int16)cfft.CurrentOutPtr[2*i]; // Left
                fftFrame->buffer[i+1] = fftFrame->buffer[i]; // Right
            }
            // *************************************************************************

            dma_flag = 0;

            timerOff();
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
 * SUMMARY: searchMaxBin
 * This function searches a bin array for the max magnitude and outputs
 * the max magnitude and frequency to the LCD
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
polar_t searchMaxBin (float * bin, Uint16 len, float freqPerBin)
{
    char wr[5] = "X.XX"; // store ASCII versions of DFT magnitude in here..
    char wr2[8] = "XXXXX.X"; // store ASCII versions of the max frequency here..
    float max = 0.0f;
    Uint16 maxK = 0;
    polar_t conversion;

    // 1.) search for the max bin value
    for (Uint16 k = 0; k < len; k++)
    {
        if (bin[k] > max)
        {
            //bruh what even is this
            max = bin[k];
            maxK = k;
        }

        bin[k] = 0.0f; // reset to be used later
    }

    max = 10.0f*log10f(max);

    // 2.) Output the max bin magnitude to the LCD
    float tens         = max / 10.0f;
    float ones         = (tens - (Uint16)tens) * 10;
    float tenths       = (ones - (Uint16)ones) * 10;
    float hundredths   = (tenths - (Uint16)tenths) * 10;

    // Convert voltage to characters and store to the LCD
    wr[0] = INT_TO_ASCII((Uint16)tens + 0.5f);
    wr[1] = INT_TO_ASCII((Uint16)ones); // 1's place (Ex. [1].23)
    wr[2] = '.';
    wr[3] = INT_TO_ASCII((Uint16)tenths); // 10th's place (Ex. 1.[2]3)
    wr[4] = '\0';

    lcdCursorRow2(6); // offset t.o the X.XX decimal voltage value
    lcdString((Uint16*)&wr);

    // 3.) Output the max frequency to the LCD
    float maxFreq       = (float)maxK * freqPerBin;
    float tenThousands  = maxFreq / 10000.0f;
    float thousands     = (tenThousands - (Uint16)tenThousands) * 10;
    float hundreds      = (thousands - (Uint16)thousands) * 10;
    tens                = (hundreds - (Uint16)hundreds) * 10;
    ones                = (tens - (Uint16)tens) * 10;
    tenths              = (ones - (Uint16)ones) * 10;
    //hey hey hey
    // Convert voltage to characters and store to the LCD
    wr2[0] = INT_TO_ASCII((Uint16)tenThousands);
    wr2[1] = INT_TO_ASCII((Uint16)thousands);
    wr2[2] = INT_TO_ASCII((Uint16)hundreds);
    wr2[3] = INT_TO_ASCII((Uint16)tens);
    wr2[4] = INT_TO_ASCII((Uint16)ones);
    wr2[5] = '.';
    wr2[6] = INT_TO_ASCII((Uint16)tenths);
    wr2[7] = '\0';

    lcdCursorRow1(7);
    lcdString((Uint16*)&wr2);

    // create the return structure
    conversion.magnitude = max;
    conversion.freq = maxFreq;
    return conversion;
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
}

/* Unused interrupt handles error thrown by RTDSP_Sampling.c */
__interrupt void Mcbsp_RxINTB_ISR(void){}
