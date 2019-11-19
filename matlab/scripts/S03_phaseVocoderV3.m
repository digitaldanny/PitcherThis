clc
clear all
close all

%===== this program is a simple phase vocoder, with:
%===== WLen is the length of the windows
%===== ni and n2: steps (in samples) for the analysis and synthesis
FILE    = 'MiddleC.mp3';
START	= 1.25;
END     = 1.75;
FS      = 48000;

[data, fs] = audioread(FILE, [FS*START FS*END]);
data = resample(data,fs, FS);
DAFx_in = data;

%----- user data -----
n1 = 512;
pit_ratio = 1.0;
WLen = 2048;
W1 = hann(WLen) ;
W2 = W1;
L = length(DAFx_in) ;
DAFx_in = [zeros(WLen, 2); DAFx_in; zeros(WLen-mod(L,n1) ,2)] / max(abs(DAFx_in));

%----- initializations -----
DAFx_out = zeros(length(DAFx_in) ,1);
grain = zeros(WLen,1);
ll = WLen/2;

omega = 2*pi*n1*[0:ll-1]' /WLen;
phi0 = zeros(ll,1) ;
rO = zeros(ll,1) ;
psi = phi0;
res = zeros (n1 ,1) ;
tic

pout = 0;
pin = 0;
pend = length(DAFx_in)-WLen;

while pin<pend
    grain = DAFx_in(pin+1:pin+WLen).* W1;
    fc = fft(fftshift(grain));
    f = fc(1:ll);
    r = abs(f);
    phi = angle(f) ;
    delta_phi = omega + princarg(phi-phi0-omega);

    delta_r = (r-rO)/n1;
    delta_psi = pit_ratio*delta_phi/n1;
    for k=1:n1
        rO = rO+delta_r;
        psi = psi+delta_psi;
        res (k) = rO'*cos(psi);
    end

    phi0 = phi;
    rO = r;
    psi = princarg(psi) ;

    %-----------------------------------------
    DAFx_out(pout+1:pout+n1) = DAFx_out(pout+1:pout+n1) + res;
    pin = pin + n1;
    pout = pout + n1;
end

toc

DAFx_out(pout+1:pout+n1)=DAFx_out(pout+1:pout+n1)+res;

pin = pin +1;
pout = pout+n1;
soundsc(DAFx_out,FS);

% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
% FUNCTION DEFINITIONS
% +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+

function phase = princarg(phasein)
    phase = mod(phasein+pi,-2*pi)+pi;
end
