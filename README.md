# PITCHER THIS

This repository contains files relating to the final project in the course [Real-Time DSP Applications](http://www.add.ece.ufl.edu/4511/) at the University of Florida. Each student had 2 weeks to model their project in MATLAB and implement the project on the microcontroller to demo in front of the class. My project consists of three frequency domain effects that can be layered on top of each other and adjusted in real time. The effects include a pitch shifter, an adjustable low-pass filter, and a robotization effect.

## 1. Relevant Links

- YouTube Project Demonstration - video still needs to be recorded as of June 4, 2020
- [Main Source Code](https://github.com/digitaldanny/PitcherThis/blob/master/c/src/main.c)
- [Linker File](https://github.com/digitaldanny/PitcherThis/blob/master/c/2837xD_RAM_lnk_cpu1.cmd)

## 2. Software

All effects implemented on the board involve a 1024 point FFT to get frequency bins and an IFFT on the processed data to get time domain data to send back out to the codec for output. All frequency domain effects are applied between the FFT and IFFT processing stages.

- **Pitch Shifter:** The pitch shifter effect uses an on-board potentiometer to shift FFT bin samples up or down before IFFT processing. Because the energy data in each bin is shifted, the resulting time-domain samples after IFFT processing is frequency shifted with the original time properties intact. This pitch shifting method does not take into account the way that vocal harmonics shift non-linearly as pitch increases and decreases. My result is a functional pitch shifter; however, a more convincing approach to natural pitch shifting would be to use the [Phase Vocoder](https://en.wikipedia.org/wiki/Phase_vocoder) algorithm.

- **Adjustable Low-Pass Filter:** For one of the previous labs during the semester, my class was required to implement a low pass filter using an FIR filter. However, this time-domain implementation of filtering was not easily configurable during runtime. The cutoff frequency for my final project's frequency-domain low pass filter could be controlled using a potentiometer on the board. The potentiometer's max voltage related to the 1024th frequency bin and the min voltage related to the 1st frequency bin. During runtime, all FFT bin values after a point defined by the potentiometer's voltage were zeroed out so that those frequencies would not appear in the IFFT results.

- **Robotization:** This effect turns human vocals into a robot-like voice. In combination with the pitch shifter to deepen your voice, the user really sounds like they could take over the world! 

## 3. Hardware

The system that this software was designed for includes the hardware listed below.

**Development Board**
The primary dev board used for this project is Texas Instrument's [TMS320F28379D launchpad](https://www.digikey.com/product-detail/en/texas-instruments/LAUNCHXL-F28379D/296-46713-ND/7219341?utm_adgroup=Development%20Boards%2C%20Kits%2C%20Programmers&utm_source=google&utm_medium=cpc&utm_campaign=Shopping_Texas%20Instruments_0296_Co-op&utm_term=&utm_content=Development%20Boards%2C%20Kits%2C%20Programmers&gclid=CjwKCAjwt-L2BRA_EiwAacX32XiNcpFcqaNMtWq84H5sKi_dEZMdtLcYemxadN9nPPvzOA_0ODPVrRoCDdgQAvD_BwE). This dev board was chosen for its C2000 MCU, which contains a 200 MHz CPU and 200 MFLOPS FPU.

**Codec**
To handle sampling audio at 44.1 KHz, the C2000 dev board interfaced TI's [TLV320AIC23](https://www.ti.com/product/TLV320AIC23) codec via I2S.

| ![Board assembly screenshots go here](image link here) | 
|:--:| 
| ***Figure 3.1-3.2:** Front and back of the hardware used during demo.* |

## 4. Additional Project Challenges

- **Handling high-frequency sampling interrupts:** Because handling interrupts from the codec every 0.022 ms would be resource intensive, the C2000 was set up for ping-pong DMA buffers. As one DMA buffer's data (ping) was being processed for the frequency domain effects, the other DMA buffer (pong) would fill with new sample data. After processing and sampling, the buffers would switch so that the pong buffer was being processed and the ping buffer was being filled with new samples, resulting in real-time sound effects.

- **Heap memory allocation:** The [library]([https://github.com/mborgerding/kissfft](https://github.com/mborgerding/kissfft)) I used for FFT/IFFT processing was built for desktop applications, which assumes that heap memory is available for dynamic memory allocation. Fortunately, after playing with the [linker file](https://github.com/digitaldanny/PitcherThis/blob/master/c/2837xD_RAM_lnk_cpu1.cmd) for a while, I was able to provide enough memory for dynamic allocation while leaving enough memory for the rest of the application. 

## 5. Contributors

Daniel Hamilton [**(@sweatpantsdanny)**](https://github.com/sweatpantsdanny)
