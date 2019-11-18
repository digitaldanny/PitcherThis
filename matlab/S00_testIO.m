% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
% SCRIPT SUMMARY:
% Sample input audio at Fs = 48KHz, perform a 512 point FFT, do a pitch shift
% perform a 512 point IFFT, and output the resulting audio. 
%
% The goal is to test that this doesn't cause any unexpected distortion to 
% the signal.
%
% COMMANDS:
% bigfft - performs an 8192 point FFT on most recently processed audio.
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
SOURCE          = 'PureTone';
FS              = 48000;                % sampling frequency used by Codec
FFT_SIZE        = 512;                  % 93.75 Hz resolution models DSP board
BIG_FFT_SIZE    = 8192;                 % 5.86 Hz resolution for checking pitch shifting performance
DISPLAY_BIG_FFT = 0;                    % displaying a plot of the big fft takes a long time
FILE            = 'MiddleC.mp3';
START           = 1.25;
END             = 2.25;
GEN_FREQ        = 375;
FUNCTION        = 'pitchShift512V3';
OVERLAY         = 0.5;

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
f = 0:FFT_SIZE;
f2 = 0:BIG_FFT_SIZE;

% set up the plots for board/big fft spectrums
if DISPLAY_BIG_FFT
    subplot(2,1,1);
end
plotHandleSmall = plot(f,0);
xlabel('Frequency (in hertz)');
title('Board FFT Magnitude Response');

plotHandleBig = [];
if DISPLAY_BIG_FFT
    subplot(2,1,2);
    plotHandleBig = plot(f2,0);
    xlabel('Frequency (in hertz)');
    title('Big FFT Magnitude Response');
end
  
% prompt user to input shift amount, then press ENTER to run once
while(1)
    shift = 0;
    sumOfProcessedBins = zeros(1, FFT_SIZE);
    bigbins = zeros(1, BIG_FFT_SIZE);
    bigHanning = hanning(BIG_FFT_SIZE);
    
    response = input('PITCH SHIFT (93.7 Hz): ', 's');
    
    if strcmp(response, 'bigfft')
        % PERFORM AN 8192 POINT FFT ON THE MOST RECENT PROCESSED AUDIO
        % AND PLOT
        bigFftResolution = FS / BIG_FFT_SIZE;

        % perform large FFT with hanning window / plot spectrum
        try
            for n = 1:BIG_FFT_SIZE:length(audioProcessed)
                nRange = n:n+BIG_FFT_SIZE-1;
                bigbins = bigbins + fft(audioProcessed(nRange).*bigHanning', BIG_FFT_SIZE); 
            end
        catch
            disp("Processed audio length != multiple of BIG_FFT_SIZE");
        end
        
        if DISPLAY_BIG_FFT
            set(plotHandleBig,...
                'XData', bigFftResolution*(0:BIG_FFT_SIZE/2-1),... 
                'YData', real(bigbins(1:BIG_FFT_SIZE/2)))
        end
        
        % determine what the max frequency is
        maxMag = 0;
        maxBin = 0;
        for i = 1:BIG_FFT_SIZE/2 - 1
            nextMag = abs(bigbins(i));
            
            if nextMag > maxMag
                maxMag = nextMag;
                maxBin = i; 
            end
        end
        
        % wait for user response
        fprintf('Max frequency: %f Hz\n', maxBin*bigFftResolution);
        input('Click ENTER to continue..');
        
    else
        % SHIFT THE INPUT AUDIO BY INPUT STEP
        shift = str2double(response);
    
        disp("Playing original audio..");
        sound(real(audio48KHzMono),FS); % test the filtered output data
        pause(END - START + 0.25); % wait for the audio to finish playing before starting the processed audio
        clear sound
        disp("-------------------------");

        % perform the 512-point FFT/IFFT on the audio with a 50% overlay sliding window.
        audioProcessed = zeros(1, length(audio48KHzMono));
        overlayBuffer = zeros(1, FFT_SIZE);         % contains previous FFT_SIZE/2 samples
        
        winvec = hanning(FFT_SIZE);                 % 512 point hanning window
        
        nFirstHalf  = 1            : FFT_SIZE/2;    % range for the first 256 samples of 512 sample frame
        nLastHalf   = FFT_SIZE/2+1 : FFT_SIZE;      % range for the last 256 samples of 512 sample frame
        
        try
            for n = 1:FFT_SIZE:length(audio48KHzMono)
                
                % capture next 512 burst DMA transfer (models real time
                % application).
                nRange = (n:n+FFT_SIZE-1);
                frame = audio48KHzMono(nRange);
                
                % ---------------------------------------------------------
                % FIRST SLIDING WINDOW ITERATION
                % Capture the first half of the new samples to perform
                % first FFT.
                % ---------------------------------------------------------
                overlayBuffer(nLastHalf) = frame(nFirstHalf); % next 256 samples from DMA frame
                bins1 = abs(fft(overlayBuffer.*winvec', FFT_SIZE));
                overlayBuffer(nFirstHalf) = overlayBuffer(nLastHalf); % processed samples become the old samples
                
                % ---------------------------------------------------------
                % SECOND SLIDING WINDOW ITERATION
                % Capture the second half of the new samples to perform 
                % second FFT.
                % ---------------------------------------------------------
                overlayBuffer(nLastHalf) = frame(nLastHalf); % next 256 samples from DMA frame
                bins2 = abs(fft(overlayBuffer.*winvec', FFT_SIZE));
                overlayBuffer(nFirstHalf) = overlayBuffer(nLastHalf); % processed samples become the old samples
                
                bins = (bins1 + bins2)/2; % average the two FFT bins
                
                % ---------------------------------------------------------
                % APPLICATION PROCESSING (Pitch shift, autotune, etc.)
                % ---------------------------------------------------------
                processedBins = feval(FUNCTION, bins, shift);

                % ---------------------------------------------------------
                % IFFT
                % ---------------------------------------------------------
                audioProcessed(nRange)= ifft(processedBins, FFT_SIZE);
                
                sumOfProcessedBins = sumOfProcessedBins + processedBins; % store total energy of the sample
            end
        catch
           disp("Length of audio sample array is != multiple of 512")
        end

        % plot the final processed bins (orange) vs the original bins (blue)
        set(plotHandleSmall,...
            'XData', fftResolution*(0:FFT_SIZE/2-1),... 
            'YData', abs(real(sumOfProcessedBins(1:FFT_SIZE/2))))

        disp("Playing processed audio..");
        sound(real(audioProcessed),FS); % test the filtered output data
        disp("-------------------------");
    end
end
