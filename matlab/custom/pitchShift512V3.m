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
            
            % find indices of the two bins to adjust
            nTargetF    = (n*RES + shift)/RES; % array index between two real values
            nL          = floor(nTargetF);
            nR          = ceil(nTargetF);
            
            % find the coefficients to multiply the original magnitude by
            % for the shifted bins.
            coefX       = (n*RES + shift - nL*RES)/RES;
            coefL       = 1 - coefX;
            coefR       = coefX;
            
            % adjust the magnitudes for the two nearest bins in order to
            % interpolate the 'targeted bin'
            processed(nL) = processed(nL) + coefL * bins(n);
            processed(nR) = processed(nR) + coefR * bins(n);

        end
        
    elseif shift < 0
        
        % negative shift starts from end of array and decrements backwards.
        disp("ERROR (pitchShift512V3): NEGATIVE PITCH SHIFTING NOT IMPLEMENTED");
       
    else
        processed = bins;
    end
    
    % zero out the frequency bins for k > fs/2.
    for n = len/2:len-1
        processed(n) = 0;
    end
end
