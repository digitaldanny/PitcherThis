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

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

// FFT related
// 9 => 512 pt, 10 => 1024 pt FFT
// (hamming LUT will have to be regenerated if this value changes)
#define CFFT_STAGES         9

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
#define MAX_SHIFT                   12.0f
#define MIN_SHIFT                   -12.0f

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

#if CFFT_STAGES == 7
const float hanningLUT[128] = {
            0.000000f, 0.000612f, 0.002446f, 0.005497f, 0.009759f, 0.015220f, 0.021868f, 0.029685f, 0.038654f, 0.048752f, 0.059954f, 0.072232f, 0.085558f, 0.099898f, 0.115217f, 0.131477f,
            0.148640f, 0.166662f, 0.185500f, 0.205108f, 0.225437f, 0.246438f, 0.268060f, 0.290249f, 0.312952f, 0.336112f, 0.359673f, 0.383578f, 0.407768f, 0.432183f, 0.456764f, 0.481452f,
            0.506184f, 0.530901f, 0.555543f, 0.580049f, 0.604359f, 0.628414f, 0.652154f, 0.675522f, 0.698460f, 0.720913f, 0.742825f, 0.764143f, 0.784815f, 0.804789f, 0.824018f, 0.842454f,
            0.860052f, 0.876768f, 0.892563f, 0.907397f, 0.921234f, 0.934040f, 0.945784f, 0.956437f, 0.965973f, 0.974369f, 0.981603f, 0.987660f, 0.992523f, 0.996180f, 0.998624f, 0.999847f,
            0.999847f, 0.998624f, 0.996180f, 0.992523f, 0.987660f, 0.981603f, 0.974369f, 0.965973f, 0.956437f, 0.945784f, 0.934040f, 0.921234f, 0.907397f, 0.892563f, 0.876768f, 0.860052f,
            0.842454f, 0.824018f, 0.804789f, 0.784815f, 0.764143f, 0.742825f, 0.720913f, 0.698460f, 0.675522f, 0.652154f, 0.628414f, 0.604359f, 0.580049f, 0.555543f, 0.530901f, 0.506184f,
            0.481452f, 0.456764f, 0.432183f, 0.407768f, 0.383578f, 0.359673f, 0.336112f, 0.312952f, 0.290249f, 0.268060f, 0.246438f, 0.225437f, 0.205108f, 0.185500f, 0.166662f, 0.148640f,
            0.131477f, 0.115217f, 0.099898f, 0.085558f, 0.072232f, 0.059954f, 0.048752f, 0.038654f, 0.029685f, 0.021868f, 0.015220f, 0.009759f, 0.005497f, 0.002446f, 0.000612f, 0.000000f
     };
#elif CFFT_STAGES == 8
const float hanningLUT[256] = {
            0.000000f, 0.000152f, 0.000607f, 0.001365f, 0.002427f, 0.003790f, 0.005454f, 0.007419f, 0.009683f, 0.012244f, 0.015102f, 0.018253f, 0.021698f, 0.025433f, 0.029455f, 0.033764f,
            0.038355f, 0.043227f, 0.048376f, 0.053800f, 0.059494f, 0.065456f, 0.071681f, 0.078166f, 0.084908f, 0.091902f, 0.099143f, 0.106628f, 0.114351f, 0.122309f, 0.130496f, 0.138907f,
            0.147537f, 0.156382f, 0.165435f, 0.174691f, 0.184144f, 0.193790f, 0.203621f, 0.213632f, 0.223818f, 0.234170f, 0.244684f, 0.255354f, 0.266171f, 0.277131f, 0.288226f, 0.299449f,
            0.310794f, 0.322255f, 0.333823f, 0.345492f, 0.357254f, 0.369104f, 0.381032f, 0.393033f, 0.405099f, 0.417223f, 0.429397f, 0.441614f, 0.453866f, 0.466146f, 0.478447f, 0.490761f,
            0.503080f, 0.515398f, 0.527706f, 0.539997f, 0.552264f, 0.564500f, 0.576696f, 0.588845f, 0.600941f, 0.612976f, 0.624941f, 0.636831f, 0.648638f, 0.660355f, 0.671974f, 0.683489f,
            0.694893f, 0.706178f, 0.717338f, 0.728366f, 0.739256f, 0.750000f, 0.760592f, 0.771027f, 0.781296f, 0.791395f, 0.801317f, 0.811056f, 0.820607f, 0.829962f, 0.839118f, 0.848067f,
            0.856805f, 0.865327f, 0.873626f, 0.881699f, 0.889540f, 0.897145f, 0.904508f, 0.911626f, 0.918495f, 0.925109f, 0.931464f, 0.937558f, 0.943387f, 0.948946f, 0.954233f, 0.959243f,
            0.963976f, 0.968426f, 0.972592f, 0.976471f, 0.980061f, 0.983359f, 0.986364f, 0.989074f, 0.991487f, 0.993601f, 0.995416f, 0.996930f, 0.998142f, 0.999052f, 0.999659f, 0.999962f,
            0.999962f, 0.999659f, 0.999052f, 0.998142f, 0.996930f, 0.995416f, 0.993601f, 0.991487f, 0.989074f, 0.986364f, 0.983359f, 0.980061f, 0.976471f, 0.972592f, 0.968426f, 0.963976f,
            0.959243f, 0.954233f, 0.948946f, 0.943387f, 0.937558f, 0.931464f, 0.925109f, 0.918495f, 0.911626f, 0.904508f, 0.897145f, 0.889540f, 0.881699f, 0.873626f, 0.865327f, 0.856805f,
            0.848067f, 0.839118f, 0.829962f, 0.820607f, 0.811056f, 0.801317f, 0.791395f, 0.781296f, 0.771027f, 0.760592f, 0.750000f, 0.739256f, 0.728366f, 0.717338f, 0.706178f, 0.694893f,
            0.683489f, 0.671974f, 0.660355f, 0.648638f, 0.636831f, 0.624941f, 0.612976f, 0.600941f, 0.588845f, 0.576696f, 0.564500f, 0.552264f, 0.539997f, 0.527706f, 0.515398f, 0.503080f,
            0.490761f, 0.478447f, 0.466146f, 0.453866f, 0.441614f, 0.429397f, 0.417223f, 0.405099f, 0.393033f, 0.381032f, 0.369104f, 0.357254f, 0.345492f, 0.333823f, 0.322255f, 0.310794f,
            0.299449f, 0.288226f, 0.277131f, 0.266171f, 0.255354f, 0.244684f, 0.234170f, 0.223818f, 0.213632f, 0.203621f, 0.193790f, 0.184144f, 0.174691f, 0.165435f, 0.156382f, 0.147537f,
            0.138907f, 0.130496f, 0.122309f, 0.114351f, 0.106628f, 0.099143f, 0.091902f, 0.084908f, 0.078166f, 0.071681f, 0.065456f, 0.059494f, 0.053800f, 0.048376f, 0.043227f, 0.038355f,
            0.033764f, 0.029455f, 0.025433f, 0.021698f, 0.018253f, 0.015102f, 0.012244f, 0.009683f, 0.007419f, 0.005454f, 0.003790f, 0.002427f, 0.001365f, 0.000607f, 0.000152f, 0.000000f
     };

#elif CFFT_STAGES == 9
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
#endif

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
#pragma DATA_SECTION(rin1,          "CFFTdata1_0x0800");
kiss_fft_scalar rin1[CFFT_SIZE];    // real input

#pragma DATA_SECTION(rin2,          "CFFTdata2_0x0800");
kiss_fft_scalar rin2[CFFT_SIZE];    // real input

#pragma DATA_SECTION(cout,          "CFFTdata3_0x1000");
kiss_fft_cpx cout[CFFT_SIZE];       // complex output

#pragma DATA_SECTION(fftOutBuff,    "CFFTdata4_0x1000");
kiss_fft_cpx fftOutBuff[CFFT_SIZE]; // complex output

#pragma DATA_SECTION(fftMagBuff,    "CFFTdata5_0x0400");
float fftMagBuff[CFFT_SIZE/2];

#pragma DATA_SECTION(fftPhaseArray,  "CFFTdata6_0x0C00");
float fftPhaseArray[3][CFFT_SIZE/2];

#pragma DATA_SECTION(overlayBuff,   "CFFTdata7_0x0800");
float overlayBuff[CFFT_SIZE];

#pragma DATA_SECTION(stretched,     "CFFTdata8_0x1400");
float stretched[5*(CFFT_SIZE>>1)];  // used for overlap add
                                    // pitch up 12 steps results in a buffer 2.5x larger
                                    // than a single frame.

kiss_fftr_cfg  kiss_fftr_state;
kiss_fftr_cfg  kiss_fftri_state;
// +-----+-----+-----+-----+-----+-----+-----+

float * binFft;
float * prevInPtr;
float * currInPtr;

float * prevPhasePtr;
float * currPhasePtr;
float * deltaPhasePtr;
float * phaseCumulativePtr;

// PITCH SHIFTING
volatile Uint16 adcAResult;
volatile float shift;
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
    // ------------------------------------------------------------------------

    // prev buffs must be initialized to 0 for stft
    for (Uint16 i = 0; i <= CFFT_SIZE_X2_MASK; i++)  prevInPtr[i] = 0.0f;
    for (Uint16 i = 0; i < CFFT_SIZE/2; i++)         prevPhasePtr[i] = 0.0f;

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

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // update frequencies on LCD
    char s1[] = "PITCH SHIFTER";
    lcdRow1();
    lcdString((Uint16 *)&s1);

    // update shift amount on
    char s2[] = "shift = ";
    lcdRow2();
    lcdString((Uint16 *)&s2);

    // KISS initializations
    int fftSize = CFFT_SIZE;
    kiss_fftr_state = kiss_fftr_alloc(fftSize, 0, NULL, NULL);
    kiss_fftri_state = kiss_fftr_alloc(fftSize, 1, NULL, NULL);

    // Phase vocoder initializations
    prevPhasePtr        = &fftPhaseArray[0][0];
    currPhasePtr        = &fftPhaseArray[1][0];
    phaseCumulativePtr  = &fftPhaseArray[2][0];
    deltaPhasePtr       = prevPhasePtr;

    EALLOW;

    while(1)
    {
        // +--------------------------------------------------------------------------------------+
        // CREATE ADC VALUES WHILE WAITING FOR NEW SAMPLES
        // +--------------------------------------------------------------------------------------+

        // find ADC value to pitch shift by
        FORCE_ADC_CONVERSION;     // force ADC to convert on A0
        WAIT_FOR_ADC_CONVERSION;  // wait for first conversion to complete
        CLEAR_ADC_FLAG;           // clear the interrupt flag generated by the conversion
        adcAResult = AdcaResultRegs.ADCRESULT0; // save results of the conversion
        shift = (MAX_SHIFT - MIN_SHIFT)*(float)adcAResult/(float)4095 - MAX_SHIFT;

        float tens   = abs(shift) / 10.0;
        float ones   = (tens - (Uint16)tens) * 10;
        float tenths = (ones - (Uint16)ones) * 10;

        // +--------------------------------------------------------------------------------------+
        // NEW SAMPLES ARE READY FOR USER APPLICATION
        // +--------------------------------------------------------------------------------------+

        if (dma_flag)
        {
            // DEBUGGING.......
            shift = 12.0f;

            timerOn();

             // create mono samples by averaging left and right samples and store to the fft buffer
             for (int i = 0; i < CFFT_SIZE_X2_MASK; i+=2)
                 currInPtr[i>>1] = ((float)fftFrame->buffer[i] + (float)fftFrame->buffer[i+1])/2.0f;

             // USER APP BEGIN ---------------------------------------------------------------

             // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
             // INIT - creating constants to lighten DSP load during following loop
             // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
             float      hop             = CFFT_SIZE>>2;             // 75% overlay = 1/4 sample size
             float      alpha           = powf(2, shift/12.0f);     // pitch scaling factor
             float      hopOut          = roundf(alpha*hop);        // intermediate constants
             //float      normalizer      = M_SQRT1_2;                // 1/sqrt(winSize/(2*hop))

             float      C_1_DIV_CFFT_SIZE    = 1/(float)CFFT_SIZE;
             float      C_2_MULT_PI          = 2*(float)M_PI;
             float      C_1_DIV_HOP          = 1/hop;

             Uint16     ifusion         = 0;    // index for combining 4 frames back together

             for (int i = 0; i < 4; i++)
             {
                 // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
                 // ANALYSIS
                 // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

                 // create 4 frames from the sample inputs and the previous inputs (or zeros)
                 // with hanning window applied.
                 Uint16 intermediate = (3-i)*CFFT_SIZE>>2;
                 Uint16 complement = CFFT_SIZE - intermediate;
                 for (Uint16 j = 0; j < CFFT_SIZE; j++)
                 {
                     overlayBuff[j] = (j < intermediate) ? prevInPtr[j+complement] : currInPtr[j-intermediate];
                     overlayBuff[j] = overlayBuff[j] * hanningLUT[j]; // * normalizer; // *normalizer = 1/sqrt(2)
                 }

                 // get the FFT of the current buffer
                 kiss_fftr(kiss_fftr_state, overlayBuff, cout); // FFT

                 // find the magnitude and phase angle (in radians) of the FFT vectors
                 // (only iterates N/2 because FFT is only valid up to fs/2)
                 for (Uint16 j = 0; j < CFFT_SIZE/2; j++)
                 {
                     // mag
                     fftMagBuff[j] = sqrtf((cout[j].r * cout[j].r) + (cout[j].i * cout[j].i)); // Sqrt(Re^2 + Im^2)

                     // phase
                     currPhasePtr[j] = atanf(cout[j].i / cout[j].r); // atan(Im/Re) in radians
                 }

                 // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
                 // PROCESSING
                 // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

                 // The delta phase array takes the place of the previous phase buffer since the
                 // previous values are not important after this iteration.
                 deltaPhasePtr = prevPhasePtr;
                 for (Uint16 j = 0; j < CFFT_SIZE/2; j++)
                     deltaPhasePtr[j] = currPhasePtr[j] - prevPhasePtr[j];

                 // The previous phase array becomes equal to the current phase buffer which
                 // will be the previous phase buffer in the next iteration.
                 Uint32 tempAddressSwitcher = (Uint32)prevPhasePtr;
                 prevPhasePtr = currPhasePtr;
                 currPhasePtr = (float*)tempAddressSwitcher;

                 for (Uint16 j = 0; j < CFFT_SIZE/2; j++)
                 {
                     // (deltaPhiPrime) remove expected phase difference
                     // deltaPhiPrime = deltaPhi - 2pi * hop  (0:CFFT_SIZE/2-1)/CFFT_SIZE
                     deltaPhasePtr[j] = deltaPhasePtr[j] - (2*M_PI*hop)*((float)j*C_1_DIV_CFFT_SIZE);

                     // (deltaPhiPrimeMod) map to -pi/pi range
                     deltaPhasePtr[j] = fmodf(deltaPhasePtr[j]+M_PI, C_2_MULT_PI) - M_PI;

                     // (trueFreq) find the true frequency
                     deltaPhasePtr[j] = C_2_MULT_PI * ((float)j*C_1_DIV_CFFT_SIZE) + deltaPhasePtr[j] * C_1_DIV_HOP;

                     // (finalPhase) find the final phase to be synthesized
                     phaseCumulativePtr[j] += hopOut * deltaPhasePtr[j];
                 }

                 // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
                 // SYNTHESIS - put time stretched data into buffer.
                 // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

                 // convert from polar format to rectangular format for IFFT
                 for (Uint16 j = 0; j < CFFT_SIZE/2; j++)
                 {
                     cout[j].r = fftMagBuff[j] * cosf(phaseCumulativePtr[j]);
                     cout[j].i = fftMagBuff[j] * sinf(phaseCumulativePtr[j]);
                 }

                 // convert from frequency domain to time domain
                 kiss_fftri(kiss_fftri_state, cout, prevInPtr); // IFFT

                 // 1.) apply the hanning window again..
                 // 2.) add new signal to the time-stretched signal to be synthesized
                 for (Uint16 j = 0; j < CFFT_SIZE; j++)
                 {
                     // 1.
                     prevInPtr[j] = prevInPtr[j] * hanningLUT[j]; // * normalizer;

                     // 2.
                     stretched[j+ifusion] += prevInPtr[j];
                 }

                 // increment fusion index by step-size for next iteration
                 ifusion += (Uint16)hopOut;
             }

             // calculate the ratio between the stretched array and the expected array size
             float interpCoef = (float)(ifusion - (Uint16)hopOut + CFFT_SIZE)*C_1_DIV_CFFT_SIZE;

             // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
             // FINALIZE
             // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

             // interpolate samples to output to the codec
             float itarget;
             float ifloor;
             float iceil;
             float sample;
             for (int i = 0; i < CFFT_SIZE; i++)
             {
                 itarget                    = (float)i*interpCoef;

                 // If the target index is not a real index, interpolate the sample.
                 // Otherwise, return the value of the target index.
                 if (itarget-(int)itarget > 0)
                 {
                     ifloor                     = floor(itarget);
                     iceil                      = ceil(itarget);

                     // weighted average of the two samples outside the target index
                     // {(iceil - itarget)*lowerSample + (itarget - ifloor)*upperSample}/2.0f
                     sample = (iceil-itarget)*stretched[(Uint16)ifloor & CFFT_SIZE_X2_MASK] +
                             (itarget-ifloor)*stretched[(Uint16)iceil & CFFT_SIZE_X2_MASK];
                     sample *= 0.5f;
                 }
                 else
                 {
                     sample = stretched[(Uint16)itarget & CFFT_SIZE_X2_MASK];
                 }

                 fftFrame->buffer[2*i]      = (int16)(sample * 0.025);  // left channel
                 fftFrame->buffer[2*i+1]    = fftFrame->buffer[2*i];    // right channel
             }

             // USER APP END -----------------------------------------------------------------

             // switch the inBuff pointers so the currentBuff becomes the previous input buffer
             Uint32 tempSwitchingPtr = (Uint32)prevInPtr;
             prevInPtr = currInPtr;
             currInPtr = (float*)tempSwitchingPtr;

             // reset the phi buffers to 0
             for (Uint16 i = 0; i < CFFT_SIZE/2; i++)
             {
                 prevPhasePtr[i]        = 0.0f;
                 phaseCumulativePtr[i]  = 0.0f;
             }

             // reset the stretched buffer
             for (Uint16 i = 0; i < 2*CFFT_SIZE; i++) stretched[i] = 0.0f;

             // Write to the LCD AFTER processing has completed
             // handle adc input to work with pitch function
             if (shift > 0.0f)
                 wr[0] = '+';
             else
                 wr[0] = '-';

             lcdCursorRow2(8);
             wr[1] = INT_TO_ASCII((Uint16)tens + 0.5f);
             wr[2] = INT_TO_ASCII((Uint16)ones); // 1's place (Ex. [1].23)
             wr[3] = '.';
             wr[4] = INT_TO_ASCII((Uint16)tenths); // 10th's place (Ex. 1.[2]3)
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
