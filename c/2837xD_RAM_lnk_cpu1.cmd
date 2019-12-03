//#############################################################################
//
// FILE:    F2837xD_FPU_ICFFT_lnk.cmd
//
// TITLE:   Linker Command File for FPU library examples that run
//          on the 2837x platform
//
//          This file includes all RAM and FLASH blocks present on the
//          2837x and depending on the active build configuration(RAM or FLASH)
//          the appropriate sections will either be loaded into RAM or FLASH
//          blocks
//
//#############################################################################
// $TI Release: C28x Floating Point Unit Library V2.01.00.00 $
// $Release Date: May 27, 2019 $
// $Copyright: Copyright (C) 2018 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################
// NOTES:
// 1. In addition to this memory linker command file, add the header linker
//    command file directly to the project. The header linker command file is
//    required to link the peripheral structures to the proper locations within
//    the memory map.
//
//    The header linker files are found in
//    c2000\C2000Ware_X_XX_XX_XX\device_support\f2837x(d/s)\headers\cmd
//
//    For BIOS applications add:      F2837x(D/S)_Headers_BIOS_cpuX.cmd
//    For nonBIOS applications add:   F2837x(D/S)_Headers_nonBIOS_cpuX.cmd
//
// 2. On reset all RAMGSx blocks are under the mastership of CPU1. The user
//     must configure the appropriate control registers to transfer mastership
//     of a RAMGSx block over to CPU2
//
// 3. Memory blocks on F2837x are uniform (ie same physical memory) in both
//    PAGE 0 and PAGE 1. That is the same memory region should not be defined
//    for both PAGE 0 and PAGE 1. Doing so will result in corruption of program
//    and/or data.
//
//    Contiguous SARAM memory blocks can be combined if required to create a
//    larger memory block.
//
//#############################################################################


// The following definitions will help to align the input buffer.For the complex FFT
// of size N, the input buffer must be aligned to a 4N word boundary. For a real FFT
// of size N, the input buffer must be aligned to a 2N word boundary. The user may define
// the macro either in the linker command file, as shown here, or
// through the project properties under,
// C2000 Linker -> Advanced Options -> Command File Preprocessing -> --define
--define CFFT_ALIGNMENT=2048
#define RAM
#if !defined(CFFT_ALIGNMENT)
#error define CFFT_ALIGNMENT under C2000 Linker -> Advanced Options -> Command File Preprocessing -> --define
#endif

MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
#if defined(RAM)
   BEGIN           : origin = 0x000000, length = 0x000002
#elif defined(FLASH)
   BEGIN           : origin = 0x080000, length = 0x000002
#endif
   RAMM0           : origin = 0x000122, length = 0x0002DE
   RAMM1           : origin = 0x000400, length = 0x000400

   RAMD0		   : origin = 0x00B000, length = 0x000800
   RAMD1		   : origin = 0x00B800, length = 0x000800

   RAMLS0          : origin = 0x008000, length = 0x001000
   //RAMLS1          : origin = 0x008800, length = 0x000800
   RAMLS1          : origin = 0x009000, length = 0x000800

   RAMGS0		   : origin = 0x00C000, length = 0x001000
   RAMGS1		   : origin = 0x00D000, length = 0x003000
   // RAMGS2		   : origin = 0x00E000, length = 0x001000
   // RAMGS3		   : origin = 0x00F000, length = 0x001000

   RESET           : origin = 0x3FFFC0, length = 0x000002

   FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
   FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
   FLASHH           : origin = 0x0A0000, length = 0x008000	/* on-chip Flash */
   FLASHI           : origin = 0x0A8000, length = 0x008000	/* on-chip Flash */
   FLASHJ           : origin = 0x0B0000, length = 0x008000	/* on-chip Flash */
   FLASHK           : origin = 0x0B8000, length = 0x002000	/* on-chip Flash */
   FLASHL           : origin = 0x0BA000, length = 0x002000	/* on-chip Flash */
   FLASHM           : origin = 0x0BC000, length = 0x002000	/* on-chip Flash */
   FLASHN           : origin = 0x0BE000, length = 0x002000	/* on-chip Flash */


PAGE 1 :
   BOOT_RSVD       : origin = 0x000002, length = 0x000120     /* Part of M0, BOOT rom will use this for stack */

   RAMLS3          : origin = 0x009800, length = 0x000800	// used
   RAMLS4          : origin = 0x00A000, length = 0x001000 	// used
   //RAMLS5          : origin = 0x00A800, length = 0x000800

   RAMGS4		   : origin = 0x010000, length = 0x001000	// used - CFFTdata1
   RAMGS5TOP	   : origin = 0x011000, length = 0x000800	// used - CFFTdata2_0x0800
   RAMGS5BOT	   : origin = 0x011800, length = 0x000800 	// used - CFFTdata7_0x0800
   RAMGS6		   : origin = 0x012000, length = 0x004000	// used - esysmem (heap)
   //RAMGS7		   : origin = 0x013000, length = 0x001000
   //RAMGS8		   : origin = 0x014000, length = 0x001000
   //RAMGS9		   : origin = 0x015000, length = 0x002000
   RAMGS10		   : origin = 0x016000, length = 0x002000 	// used - DMAACCESSABLE
   //RAMGS11		   : origin = 0x017000, length = 0x001000
   RAMGS12         : origin = 0x018000, length = 0x001000	// used - CFFTdata4
   RAMGS13         : origin = 0x019000, length = 0x001000	// used - ebss
   RAMGS14TOP      : origin = 0x01A000, length = 0x000800	// used - CFFTdata5_0x0800
   RAMGS14BOT      : origin = 0x01A800, length = 0x000800	// used - CFFTdata6_0x0800
   RAMGS16         : origin = 0x01B000, length = 0x001000	// used - CFFTdata3

   FLASHB          : origin = 0x082000, length = 0x002000	/* on-chip Flash */
}

SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0
#if defined(RAM)
   .TI.ramfunc      : > RAMM0,     PAGE = 0
   //.text            :>> RAMM1 | RAMD0 | RAMD1 | RAMLS0,  PAGE = 0
   .text            :>> RAMM1 | RAMD0 | RAMD1 | RAMLS0 | RAMGS0 | RAMGS1,  PAGE = 0
   .cinit           : > RAMLS1,    PAGE = 0

   .pinit           : > RAMLS1,    PAGE = 0
   .switch          : > RAMLS1,    PAGE = 0
   .econst          : > RAMLS4,    PAGE = 1

   // ALL SECTIONS BELOW WERE PREVIOUSLY LOCATED IN THE TEST SPECIFIC SECTION
   .cio             : > RAMLS3,    PAGE = 1

   .sysmem          : > RAMGS6,    PAGE = 1  // dynamic memory allocation memory
   .esysmem 		: > RAMGS6,    PAGE = 1  // dynamic memory allocation memory

   .stack           : > RAMLS4,    PAGE = 1
   .ebss            : > RAMGS13,   PAGE = 1
#elif defined(FLASH)
   .TI.ramfunc      :  LOAD = FLASHC,
                       RUN = RAMLS1,
                       RUN_START(_RamfuncsRunStart),
                       LOAD_START(_RamfuncsLoadStart),
                       LOAD_SIZE(_RamfuncsLoadSize),
                       PAGE = 0

   .text            : > FLASHN,    PAGE = 0
   .cinit           : > FLASHM,    PAGE = 0

   .pinit           : > FLASHM,    PAGE = 0
   .switch          : > FLASHM,    PAGE = 0
   .econst          : > FLASHB,    PAGE = 1
#if defined(USE_TABLES)
   FFT_Twiddles     : LOAD = FLASHB,
                      RUN  = RAMGS12
                      RUN_START(_FFTTwiddlesRunStart),
                      LOAD_START(_FFTTwiddlesLoadStart),
                      LOAD_SIZE(_FFTTwiddlesLoadSize),
                      PAGE = 1,
  {
     --library=c28x_fpu_dsp_library.lib<CFFT_f32_twiddleFactors.obj> (.econst)
  }

#endif
#else
#error Add either "RAM" or "FLASH" to C2000 Linker -> Advanced Options -> Command File Preprocessing -> --define
#endif //RAM

    /* Test specific sections */
   DMAACCESSABLE 	: > RAMGS10,    PAGE = 1
   CFFTdata1        : > RAMGS4,     PAGE = 1
   CFFTdata2_0x0800 : > RAMGS5TOP,  PAGE = 1
   CFFTdata3        : > RAMGS16,    PAGE = 1
   CFFTdata4        : > RAMGS12,    PAGE = 1
   CFFTdata5_0x0800 : > RAMGS14TOP, PAGE = 1
   CFFTdata6_0x0800 : > RAMGS14BOT, PAGE = 1
   CFFTdata7_0x0800 : > RAMGS5BOT,	PAGE = 1

   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */
}
/*
//===========================================================================
// End of file.
//===========================================================================
*/
