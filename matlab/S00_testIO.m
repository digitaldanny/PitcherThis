% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
% SCRIPT SUMMARY:
% Sample input audio at Fs = 48KHz, perform a 512 point FFT, do a pitch shift
% perform a 512 point IFFT, and output the resulting audio. 
%
% The goal is to test that this doesn't cause any unexpected distortion to 
% the signal.
% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
close all
clear all
clc

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% CONSTANTS
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
FREQ_MIDDLE_C   = 261.6256;
FREQ_RES        = 93.75;

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% USER CONFIGURATIONS
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% SOURCE: 
% 'Sample' -> from FILE 
% 'PureTone' -> from generated sine wave
% 'MultiTone' -> from generated sine wave with multiple sinusoids
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
SOURCE      = 'PureTone';
FS          = 48000;
FFT_SIZE    = 512;
FILE        = 'MiddleC.mp3';
START       = 1.25;
END         = 2.25;
HANNING     = 1;
GEN_FREQ    = 375;
FUNCTION    = 'pitchShift513V2';

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% SCRIPT BEGIN
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

fftResolution = FS/FFT_SIZE;

% sample the audio at 48 KHz and convert LR data to mono
audio48KHzMono = [];
if strcmp(SOURCE, 'Sample')
    % SAMPLE 
    [audio,fsOriginal] = audioread(FILE, [FS*START FS*END]);
    audio48kHz = resample(audio,fsOriginal, FS);
    audio48KHzMono = (audio48kHz(:,1) + audio48kHz(:,2))/2; % (L + R) / 2 = mono
elseif (strcmp(SOURCE, 'PureTone'))
    % PURETONE
    audio48KHzMono = 0.25*sin(2*pi*GEN_FREQ*(FS*START:FS*END)/FS);
else 
    % MULTITONE
    audio48KHzMono = 0.1*sin(2*pi*GEN_FREQ*(FS*START:FS*END)/FS)... 
        + 0.1*sin(2*pi*3*GEN_FREQ*(FS*START:FS*END)/FS)... 
        + 0.1*sin(2*pi*5*GEN_FREQ*(FS*START:FS*END)/FS)...
        + 0.1*sin(2*pi*7*GEN_FREQ*(FS*START:FS*END)/FS);
end

% figure data for plotting new FFT data
fgh = figure(); % create a figure
f = 0:2*FFT_SIZE;
plotHandle = plot(f,0);
xlabel('Frequency (in hertz)');
title('Magnitude Response');

% prompt user to input shift amount, then press ENTER to run once
while(1)
    sumOfProcessedBins = zeros(1, FFT_SIZE);
    shift = input('PITCH SHIFT (93.7 Hz): ');
    
    disp("Playing original audio..");
    sound(real(audio48KHzMono),FS); % test the filtered output data
    pause(END - START + 0.25); % wait for the audio to finish playing before starting the processed audio
    clear sound
    disp("-------------------------");

    % perform the 512-point FFT/IFFT on the audio.
    audioProcessed = zeros(1, length(audio48KHzMono));
    try
        for n = 1:FFT_SIZE-1:length(audio48KHzMono)
            nRange = (n:n+FFT_SIZE-1);

            % FFT with hamming window ----------------------------
            winvec = hanning(FFT_SIZE);
            
            % perform FFT with or without hamming window
            if HANNING == 0
                winvec = 1;
            end
            
            bins = fft(audio48KHzMono(nRange).*winvec', FFT_SIZE);
            
            % pitch shift ----------------------------------------
            %processedBins = feval(FUNCTION, bins, shift); % calls function defined in FUNCTION configuration variable
            processedBins = pitchShift512V3(bins, shift);
            
            % IFFT -----------------------------------------------
            audioProcessed(nRange)= ifft(processedBins, FFT_SIZE);
            sumOfProcessedBins = sumOfProcessedBins + processedBins; % store total energy of the sample
        end
    catch
       disp("Length of audio sample array is != multiple of 512")
    end
    
	% plot the final processed bins (orange) vs the original bins (blue)
	set(plotHandle,...
        'XData', fftResolution*(0:length(sumOfProcessedBins)-1),... 
        'YData', real(sumOfProcessedBins))

    disp("Playing processed audio..");
    sound(real(audioProcessed),FS); % test the filtered output data
    disp("-------------------------");
end
