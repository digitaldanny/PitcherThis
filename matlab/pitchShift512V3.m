% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
% FUNCTION SUMMARY: pitchShift512V3
% This function performs a pitch shift of approximately the input shift
% amount. Each bin passed into this function represents the energy in a 
% multiple of 93.75 Hz. Pitch shifting by non-multiples of 93.75 requires
% some interpolation of the magnitudes for two bins. This function LINEARLY
% interpolates magnitudes.
%
% This function also 0's out any FFT bin for k greater than fs/2. This is 
% an attempt to handle any aliasing.
%
% This function performs a NON-circular pitch shift. Samples that go above 
% the 512 point boundary or below 0 are lost.
%
% INPUTS;
% bins  - array of fft bins to perform pitch shift on.
% shift - FLOAT number of Hz to shift the frequency bins by.
%
% OUTPUTS:
% processed - array of fft bins that have been pitch shifted.
% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+

function [processed] = pitchShift512V3(bins, shift)
    RES = 93.75; % Hz
    len = length(bins);
    processed = zeros(1, length(bins));
 
    % linearly interpolate the magnitude adjustments required for the input
    % frequency shift.
    if shift > 0
        
        % positive shift starts from beginning of array and increments
        % forward.
        for n = 1:len/2-2
            processed(n)    = processed(n) + ((RES - shift)/RES * bins(n)); % adjust current bin
            processed(n+1)  = processed(n+1) + (shift/RES * bins(n));       % adjust next bin
        end
        
    else
        
        % negative shift starts from end of array and decrements backwards.
        for n = len/2-1 : -1 : 2
            processed(n)    = processed(n) + ((RES + shift)/RES * bins(n)); % adjust current bin
            processed(n-1)  = processed(n-1) + (abs(shift)/RES * bins(n));  % adjust previous bin
        end
        
    end
    
    % zero out the frequency bins for k > fs/2.
    for n = len/2:len-1
        processed(n) = 0;
    end
end
