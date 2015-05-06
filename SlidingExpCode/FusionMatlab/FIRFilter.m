% Input:
% x: signal input.
% t: time_stamp.
% y: signal output.
% t_s: sampled signal time_stamp (uniform rate)
function [y, t_s] = FIRFilter(x, t)
dt_resampled = median(diff(t));
t0 = min(t);
tf = max(t);
t_s = t0:dt_resampled:tf;

x_resampled = interp1(t,x,t_s);
N   = 500;              % FIR filter order
Fp  = 0.15;              % passband-edge frequency
Fs  = 1/dt_resampled;   % sampling frequency
Rp  = 0.00057565;       % Corresponds to 0.01 dB peak-to-peak ripple
Rst = 1e-4;             % Corresponds to 80 dB stopband attenuation

NUM = firceqrip(N,Fp/(Fs/2),[Rp Rst],'passedge'); % NUM = vector of coeffs
%fvtool(NUM,'Fs',Fs,'Color','White') % Visualize filter
y = filtfilt(NUM,1,x_resampled);
t_s = t_s';
end

