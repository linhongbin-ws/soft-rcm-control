function y = LPF2(x)
%LPF2 Filters input x and returns output y.

% MATLAB Code
% Generated by MATLAB(R) 9.8 and DSP System Toolbox 9.10.
% Generated on: 24-Aug-2021 12:42:23

%#codegen

% To generate C/C++ code from this function use the codegen command. Type
% 'help codegen' for more information.

persistent Hd;

if isempty(Hd)
    
    % The following code was used to design the filter coefficients:
    % % FIR least-squares Lowpass filter designed using the FIRLS function.
    %
    % % All frequency values are in Hz.
    % Fs = 100;  % Sampling Frequency
    %
    % N     = 10;  % Order
    % Fpass = 20;  % Passband Frequency
    % Fstop = 45;  % Stopband Frequency
    % Wpass = 1;   % Passband Weight
    % Wstop = 1;   % Stopband Weight
    %
    % % Calculate the coefficients using the FIRLS function.
    % b  = firls(N, [0 Fpass Fstop Fs/2]/(Fs/2), [1 1 0 0], [Wpass Wstop]);
    
    Hd = dsp.FIRFilter( ...
        'Numerator', [-0.01026727688297 0.0280586640306845 ...
        -0.00667301902390269 -0.105192216060432 0.26657465323724 ...
        0.654237693655341 0.26657465323724 -0.105192216060432 ...
        -0.00667301902390269 0.0280586640306845 -0.01026727688297]);
end

y = step(Hd,double(x));


% [EOF]
