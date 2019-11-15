% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
% SCRIPT SUMMARY:
% Sample input audio at Fs = 48KHz, perform a 512 point FFT, do a pitch shift
% perform a 512 point IFFT, and output the resulting audio. 
%
% The goal is to test that this doesn't cause any unexpected distortion to 
% the signal.
% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
clear all
clc

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% USER CONFIGURATIONS
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

FS          = 48000;
FFT_SIZE    = 512;
FILE        = 'ShortSoundTest.mp3';

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% SCRIPT BEGIN
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

% sample the audio at 48 KHz and convert LR data to mono
[audio,fsOriginal] = audioread(FILE);
audio48kHz = resample(audio,fsOriginal, FS);
audio48KHzMono = (audio48kHz(:,1) + audio48kHz(:,2))/2; % (L + R) / 2 = mono

% perform the 512-point FFT/IFFT on the audio.
audioProcessed = zeros(1, length(audio48KHzMono));
try
    for n = 1:FFT_SIZE-1:length(audio48KHzMono)
        nRange = (n:n+FFT_SIZE-1);

        fftSamples              = fft(audio48KHzMono(nRange), FFT_SIZE);    % FFT
        audioProcessed(nRange)  = ifft(fftSamples, FFT_SIZE);               % IFFT
    end
catch
    disp("Length of audio sample array is != multiple of 512")
end

sound(audioProcessed,FS); % test the filtered output
