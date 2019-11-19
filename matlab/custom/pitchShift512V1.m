% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
% FUNCTION SUMMARY: pitchShift512V1
% This function performs a basic pitch shift by the frequency resolution 
% amount. 
%
% This function performs a NON-circular pitch shift. Samples that go above 
% the 512 point boundary or below 0 are lost.
%
% Increasing or decreasing the shift amount by 1 will shift by 93.75 Hz at
% assuming the sampling frequency is 48 KHz.
%
% INPUTS;
% bins  - array of fft bins to perform pitch shift on.
% shift - INTEGER number of frequency bins to shift pitch by. 
%
% OUTPUTS:
% processed - array of fft bins that have been pitch shifted.
% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+

function [processed] = pitchShift512V1(bins, shift)
    processed = zeros(1, length(bins));

    % shift the frequency data by the shift amo
    for n = 1:length(bins)
        shiftIndex = (n - shift);

        % prevents wrap around shifting / errors.
        if shiftIndex > 0 && shiftIndex < 512
            processed(n) = bins(shiftIndex);
        end
    end
end

