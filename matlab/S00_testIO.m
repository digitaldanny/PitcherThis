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
FILE        = 'MiddleC.mp3';
START       = 1.25;
END         = 1.5;

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% SCRIPT BEGIN
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

% sample the audio at 48 KHz and convert LR data to mono
[audio,fsOriginal] = audioread(FILE, [FS*START FS*END]);
audio48kHz = resample(audio,fsOriginal, FS);
audio48KHzMono = (audio48kHz(:,1) + audio48kHz(:,2))/2; % (L + R) / 2 = mono

% figure data for plotting new FFT data
fgh = figure(); % create a figure
f = 0:2*FFT_SIZE;
plotHandle = plot(f,0);
xlabel('Frequency (in hertz)');
title('Magnitude Response');

% prompt user to input shift amount, then press ENTER to run once
while(1)
    shift = input('PITCH SHIFT (93.7 Hz): ');

    % perform the 512-point FFT/IFFT on the audio.
    audioProcessed = zeros(1, length(audio48KHzMono));
    try
        for n = 1:FFT_SIZE-1:length(audio48KHzMono)
            nRange = (n:n+FFT_SIZE-1);

            % FFT ------------------------------------------------
            bins = fft(audio48KHzMono(nRange), FFT_SIZE);
            
            % pitch shift ----------------------------------------
            processedBins = pitchShift512V1(bins, shift); 
            
            % IFFT -----------------------------------------------
            audioProcessed(nRange)= ifft(processedBins, FFT_SIZE);
        end
    catch
       disp("Length of audio sample array is != multiple of 512")
    end
    
	% plot the final processed bins (orange) vs the original bins (blue)
	set(plotHandle,'XData', 0:length(processedBins)-1,'YData',real(processedBins))

    disp("Playing processed audio..");
    sound(real(audioProcessed),FS); % test the filtered output data
    disp("-------------------------");
end
