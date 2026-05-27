%% reconstruct_2023h_demo.m
% Learning simulation for the 2023 H signal separation problem.
%
% Goal:
%   1. Build two source signals A and B.
%   2. Mix them into C = A + B.
%   3. Scan the spectrum at 5 kHz steps, similar to the firmware.
%   4. Estimate each signal's frequency, type, amplitude and phase.
%   5. Rebuild A' and B' from the estimated parameters.
%
% This script is intentionally written with plain loops and helper functions
% so the algorithm can be ported to STM32 code later.

clear; clc; close all;

%% Acquisition setup, matching the current firmware idea
fs = 2.4e6;                 % ADC sample rate used by sweep code
n = 1920;                   % 1920 samples -> 1250 Hz FFT-bin spacing
t = (0:n-1) / fs;

sweep_freqs = 20e3:5e3:300e3;
base_freqs = 20e3:5e3:100e3;

%% Test signal setup
% The problem says A/B are sine or triangle, 20 kHz to 100 kHz, and 5 kHz
% integer multiples.  Change these values to test different cases.
sigA.freq = 35e3;
sigA.vpp = 1.00;
sigA.phase = deg2rad(25);
sigA.type = "triangle";

sigB.freq = 100e3;
sigB.vpp = 1.00;
sigB.phase = deg2rad(110);
sigB.type = "sine";

A = synthSignal(sigA, t);
B = synthSignal(sigB, t);
C = A + B;

%% Spectrum scan
scan = measureSweep(C, t, sweep_freqs);

%% Separate and reconstruct
result = separateFromSweep(scan, sweep_freqs, base_freqs);

A_rec = synthSignal(result(1), t);
B_rec = synthSignal(result(2), t);
C_rec = A_rec + B_rec;

%% Print result
fprintf('Original A: %7.0f Hz, %-8s, Vpp %.3f, phase %.1f deg\n', ...
    sigA.freq, sigA.type, sigA.vpp, rad2deg(sigA.phase));
fprintf('Detected A: %7.0f Hz, %-8s, Vpp %.3f, phase %.1f deg\n\n', ...
    result(1).freq, result(1).type, result(1).vpp, rad2deg(result(1).phase));

fprintf('Original B: %7.0f Hz, %-8s, Vpp %.3f, phase %.1f deg\n', ...
    sigB.freq, sigB.type, sigB.vpp, rad2deg(sigB.phase));
fprintf('Detected B: %7.0f Hz, %-8s, Vpp %.3f, phase %.1f deg\n\n', ...
    result(2).freq, result(2).type, result(2).vpp, rad2deg(result(2).phase));

fprintf('RMS reconstruction error: %.6f V\n', rms(C - C_rec));

%% Plots
figure('Name', '2023H Signal Reconstruction Demo', 'Color', 'w');

subplot(4, 1, 1);
plot(t * 1e6, A, 'b'); hold on;
plot(t * 1e6, A_rec, '--r');
grid on;
xlim([0 120]);
ylabel('V');
title('A and Rebuilt A''');
legend('Original A', 'Rebuilt A''');

subplot(4, 1, 2);
plot(t * 1e6, B, 'b'); hold on;
plot(t * 1e6, B_rec, '--r');
grid on;
xlim([0 120]);
ylabel('V');
title('B and Rebuilt B''');
legend('Original B', 'Rebuilt B''');

subplot(4, 1, 3);
plot(t * 1e6, C, 'Color', [0.1 0.1 0.1]); hold on;
plot(t * 1e6, C_rec, '--r');
grid on;
xlim([0 120]);
ylabel('V');
title('Mixed C and Rebuilt A'' + B''');
legend('Original C', 'Rebuilt C');

subplot(4, 1, 4);
stem(sweep_freqs / 1e3, scan.vpp, 'filled');
grid on;
xlabel('Frequency (kHz)');
ylabel('Vpp');
title('Measured Spectrum Points');

%% Helper functions
function y = synthSignal(sig, t)
peak = sig.vpp / 2;
phase = 2*pi*sig.freq*t + sig.phase;

if sig.type == "sine"
    y = peak * sin(phase);
elseif sig.type == "triangle"
    y = peak * (2/pi) * asin(sin(phase));
else
    y = zeros(size(t));
end
end

function scan = measureSweep(x, t, freqs)
n = numel(x);
scan.vpp = zeros(size(freqs));
scan.phase = zeros(size(freqs));
scan.complex = zeros(size(freqs));

for k = 1:numel(freqs)
    f = freqs(k);
    basis = exp(-1j * 2*pi*f*t);
    c = (2/n) * sum(x .* basis);

    % For x = peak*sin(wt+phi), c angle is phi - 90 deg.
    scan.complex(k) = c;
    scan.vpp(k) = 2 * abs(c);
    scan.phase(k) = wrapToPiLocal(angle(c) + pi/2);
end
end

function result = separateFromSweep(scan, sweep_freqs, base_freqs)
amp_threshold = 0.08;
result = repmat(struct('freq', 0, 'vpp', 0, 'phase', 0, 'type', "unknown"), 1, 2);
count = 0;

for f = base_freqs
    idx = find(sweep_freqs == f, 1);
    if isempty(idx) || scan.vpp(idx) < amp_threshold
        continue;
    end

    if isHarmonicOfDetectedTriangle(f, result, count)
        continue;
    end

    count = count + 1;
    if count > 2
        break;
    end

    fund_vpp = scan.vpp(idx);
    fund_phase = scan.phase(idx);
    wave_type = classifyType(f, fund_vpp, scan, sweep_freqs);

    result(count).freq = f;
    result(count).phase = fund_phase;
    result(count).type = wave_type;

    if wave_type == "triangle"
        % A unit triangle's fundamental peak amplitude is 8/pi^2 of its real
        % peak amplitude.  Convert measured fundamental Vpp back to waveform Vpp.
        result(count).vpp = fund_vpp / (8/pi^2);
    else
        result(count).vpp = fund_vpp;
    end
end

if count < 2
    warning('Only detected %d signal(s). Try lowering amp_threshold or changing the test case.', count);
end
end

function tf = isHarmonicOfDetectedTriangle(freq, result, count)
tf = false;
for k = 1:count
    if result(k).type == "triangle" && ...
       (freq == 3*result(k).freq || freq == 5*result(k).freq)
        tf = true;
        return;
    end
end
end

function wave_type = classifyType(fund_freq, fund_vpp, scan, sweep_freqs)
idx3 = find(sweep_freqs == 3*fund_freq, 1);
idx5 = find(sweep_freqs == 5*fund_freq, 1);

amp3 = 0;
amp5 = 0;
if ~isempty(idx3), amp3 = scan.vpp(idx3); end
if ~isempty(idx5), amp5 = scan.vpp(idx5); end

ratio3 = amp3 / max(fund_vpp, eps);
ratio5 = amp5 / max(fund_vpp, eps);

% Triangle harmonics ideally follow 1/9 and 1/25.  Sine has almost none.
if (ratio3 > 0.05 && ratio3 < 0.20) || (ratio5 > 0.015 && ratio5 < 0.08)
    wave_type = "triangle";
else
    wave_type = "sine";
end
end

function y = wrapToPiLocal(x)
y = mod(x + pi, 2*pi) - pi;
end
