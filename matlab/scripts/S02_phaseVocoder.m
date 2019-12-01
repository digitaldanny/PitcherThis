% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
% SCRIPT SUMMARY:
% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
clear all
close all
clc

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% CONSTANTS
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
FREQ_MIDDLE_C   = 261.6256;
FREQ_RES        = 93.75;

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% USER CONFIGURATIONS
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% MODE
% 'Auto' -> performs a pitch step sweep from 0:24 then 24:-1:-24
% 'Manual' -> user controls the steps shifted
%
% APPROX_METHOD
% 'Gaussian' -> more accurate
% 'Parabolic' -> takes less processing time
% 'None'
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
FS              = 48000;                % sampling frequency used by Codec
FFT_SIZE        = 1024;                  % 93.75 Hz resolution models DSP board
BIG_FFT_SIZE    = 16384;                % 2.93 Hz resolution for checking pitch shifting performance
FILE            = 'MiddleC.mp3';
START           = 1.25;
END             = 1.75;
GEN_FREQ        = FREQ_MIDDLE_C;
MODE            = 'Manual';
APPROX_METHOD   = 'None';
MIN_STEP        = -12;
MAX_STEP        = 12;
FREQ            = 187.5;

% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% BEGIN SCRIPT
% +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

% sample the audio at 48 KHz and convert LR data to mono
[audio,fsOriginal] = audioread(FILE, [FS*START FS*END]);
x = resample(audio,fsOriginal, FS);
x = (x(:,1) + x(:,2))/2; % (L + R) / 2 = mono

for n = 1:FS
    x(n,1) = sin(2*pi*FREQ*n/FS);
end

fftResolution = FS/FFT_SIZE;
f = fftResolution*(0:FFT_SIZE/2-1);

% figure data for plotting new FFT data
fgh = figure();
plotHandle = plot(f,0);
xlabel('Frequency (in hertz)');
title('Board FFT Magnitude Response');

response = MIN_STEP; % if operating in Auto mode, start response as minimum step

while(1)
    auto = strcmp(MODE, 'Auto');
    
    % only take input from the user if script is running in manual mode.
    if ~auto
        response = input('Enter shift size: ');
    end
    clear sound
    
    % processing section
    frameSize = FFT_SIZE; % number of samples per frame
    hopSize = FFT_SIZE/4; % 75% frame overlay
    stepShift = response;
    y = pitchShift(x, frameSize, hopSize, stepShift);
    
    bins = zeros(1, FFT_SIZE/2);
    try
        for n = 1:FFT_SIZE:length(y)
            nRange = n:n+FFT_SIZE-1;
            newBins = fft(y(nRange), FFT_SIZE);
            bins = (bins + newBins(1:FFT_SIZE/2))/2;
        end
    catch
    end
    
    % find the max frequency using Gaussian interpolation
    if strcmp(APPROX_METHOD, 'Gaussian')
        bins = abs(bins);
        [M, km] = max(bins);
        num = log(bins(km + 1) / bins(km - 1));
        den = 2*log( (bins(km))^2 / (bins(km - 1) * bins(km + 1)) );
        delta = num/den;
        maxFreq = fftResolution*(km + delta);
        fprintf('Max frequency: %f\n', maxFreq);
    end
    
    % plot the final processed bins (orange) vs the original bins (blue)
    set(plotHandle,...
        'XData', f,... 
        'YData', abs(bins(1:FFT_SIZE/2)))
    
    sound(y, FS);
    
    if auto
        pause(END - START);
        
        % first sweep from 0:24
        if response < MAX_STEP
            response = response + 1;
        else
            break;
        end
    end
end

